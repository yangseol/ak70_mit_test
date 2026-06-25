"""Unix datagram IPC and singleton locking for the realtime controller."""

from __future__ import annotations

import fcntl
import json
import os
import socket
import stat
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from motor_profiles import format_motor_id, get_motor_profile, normalize_motor_id


DEFAULT_SOCKET_PATH = Path("/tmp/ak_realtime_controller.sock")
DEFAULT_LOCK_PATH = Path("/tmp/ak_realtime_controller.lock")
MAX_PACKET_SIZE = 8192
ALLOWED_COMMANDS = {
    "PING",
    "STATUS",
    "ARM",
    "HEARTBEAT",
    "SET_TARGETS",
    "SET_TRAJECTORY",
    "RELEASE_IDS",
    "RELEASE_SESSION",
    "ESTOP",
    "CLEAR_TARGETS",
    "DISARM",
    "CONFIRM_HOME",
    "SHUTDOWN",
}


class ControllerAlreadyRunning(RuntimeError):
    pass


class LiveSocketExists(RuntimeError):
    pass


@dataclass
class ControllerLock:
    path: Path
    fd: int

    def close(self) -> None:
        os.close(self.fd)


@dataclass
class BoundDatagramSocket:
    path: Path
    sock: socket.socket
    st_dev: int
    st_ino: int
    created_by_this_process: bool = True

    def close_and_cleanup(self) -> None:
        self.sock.close()
        if not self.created_by_this_process:
            return
        try:
            st = self.path.stat()
        except FileNotFoundError:
            return
        if not stat.S_ISSOCK(st.st_mode):
            return
        if st.st_dev == self.st_dev and st.st_ino == self.st_ino:
            self.path.unlink()


def acquire_controller_lock(path: Path = DEFAULT_LOCK_PATH) -> ControllerLock:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o600)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        os.close(fd)
        raise ControllerAlreadyRunning(f"controller lock is held: {path}") from exc
    os.ftruncate(fd, 0)
    payload = f"pid={os.getpid()} started_at={time.time():.6f}\n"
    os.write(fd, payload.encode("ascii"))
    os.fsync(fd)
    return ControllerLock(path=path, fd=fd)


def probe_datagram_socket(path: Path, timeout_sec: float = 0.15) -> bool:
    if not path.exists():
        return False
    client = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    tmp_path = Path(f"/tmp/ak_realtime_probe_{os.getpid()}_{time.time_ns()}.sock")
    try:
        client.bind(str(tmp_path))
        client.settimeout(timeout_sec)
        client.sendto(json.dumps({"command": "PING"}).encode("utf-8"), str(path))
        data, _addr = client.recvfrom(MAX_PACKET_SIZE)
        response = json.loads(data.decode("utf-8"))
        return response.get("ok") is True
    except Exception:
        return False
    finally:
        client.close()
        try:
            tmp_path.unlink()
        except FileNotFoundError:
            pass


def bind_controller_socket(path: Path = DEFAULT_SOCKET_PATH) -> BoundDatagramSocket:
    if path.exists():
        if probe_datagram_socket(path):
            raise LiveSocketExists(f"live controller responded on {path}")
        st = path.stat()
        if stat.S_ISSOCK(st.st_mode):
            path.unlink()
        else:
            raise RuntimeError(f"stale path is not a socket: {path}")
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    old_umask = os.umask(0o177)
    try:
        sock.bind(str(path))
    finally:
        os.umask(old_umask)
    os.chmod(path, 0o600)
    st = path.stat()
    return BoundDatagramSocket(path=path, sock=sock, st_dev=st.st_dev, st_ino=st.st_ino)


def validate_message(raw: bytes) -> dict[str, Any]:
    if len(raw) > MAX_PACKET_SIZE:
        raise ValueError("packet too large")
    try:
        message = json.loads(raw.decode("utf-8"))
    except Exception as exc:
        raise ValueError("invalid JSON") from exc
    if not isinstance(message, dict):
        raise ValueError("message must be an object")
    command = message.get("command")
    if command not in ALLOWED_COMMANDS:
        raise ValueError(f"unknown command: {command!r}")
    allowed_fields = {
        "command",
        "request_id",
        "session_token",
        "session_epoch",
        "process_owner_token",
        "owner",
        "targets",
        "trajectory",
        "waypoints",
        "motor_ids",
        "expected_generations",
        "control_group_id",
        "plan_id",
        "motor_id",
        "position_deg",
        "position_rad",
        "target_deg",
        "kp",
        "kd",
        "move_sec",
        "max_following_error_deg",
        "display_mode",
        "heartbeat_seq",
        "generated_monotonic",
        "reason",
    }
    unknown = set(message) - allowed_fields
    if unknown:
        raise ValueError(f"unknown fields: {sorted(unknown)}")
    if command == "SET_TARGETS":
        targets = message.get("targets")
        if not isinstance(targets, list) or not targets:
            raise ValueError("SET_TARGETS requires non-empty targets list")
        seen: set[int] = set()
        for target in targets:
            if not isinstance(target, dict):
                raise ValueError("target must be an object")
            unknown_target = set(target) - {
                "motor_id",
                "position_deg",
                "position_rad",
                "target_deg",
                "kp",
                "kd",
                "move_sec",
                "max_following_error_deg",
                "control_group_id",
                "generation",
                "display_mode",
                "session_token",
                "session_epoch",
                "plan_id",
            }
            if unknown_target:
                raise ValueError(f"unknown target fields: {sorted(unknown_target)}")
            if ("position_deg" in target) == ("position_rad" in target):
                raise ValueError("target must include exactly one position unit")
            motor_id = normalize_motor_id(str(target.get("motor_id")))
            get_motor_profile(motor_id)
            if motor_id in seen:
                raise ValueError(f"duplicate target ID {format_motor_id(motor_id)}")
            seen.add(motor_id)
            for key in ("position_deg", "position_rad", "kp", "kd"):
                if key in target:
                    value = float(target[key])
                    if value != value or value in (float("inf"), float("-inf")):
                        raise ValueError(f"{key} must be finite")
    if command == "SET_TRAJECTORY":
        motor_id = normalize_motor_id(str(message.get("motor_id")))
        get_motor_profile(motor_id)
        waypoints = message.get("waypoints")
        if not isinstance(waypoints, list) or not waypoints:
            raise ValueError("SET_TRAJECTORY requires waypoints")
        for waypoint in waypoints:
            if not isinstance(waypoint, dict):
                raise ValueError("waypoint must be an object")
            target = float(waypoint["target_deg"])
            duration = float(waypoint.get("duration_sec", waypoint.get("duration")))
            if target != target or duration != duration or duration <= 0:
                raise ValueError("waypoint values must be finite and positive duration")
    if command in {"RELEASE_IDS", "RELEASE_SESSION", "HEARTBEAT"} and not message.get("session_token"):
        raise ValueError(f"{command} requires session_token")
    if command == "RELEASE_IDS":
        ids = message.get("motor_ids")
        if not isinstance(ids, list):
            raise ValueError("RELEASE_IDS requires motor_ids")
        for motor_id in ids:
            normalize_motor_id(str(motor_id))
    if command == "HEARTBEAT":
        int(message["heartbeat_seq"])
        float(message["generated_monotonic"])
    if command == "CONFIRM_HOME":
        normalize_motor_id(str(message.get("motor_id")))
    return message


def make_request(command: str, **kwargs: Any) -> bytes:
    payload = {"command": command, **kwargs}
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


def send_request(message: dict[str, Any], socket_path: Path = DEFAULT_SOCKET_PATH, timeout_sec: float = 1.0) -> dict[str, Any]:
    client = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    reply_path = Path(f"/tmp/ak_realtime_client_{os.getpid()}_{time.time_ns()}.sock")
    try:
        client.bind(str(reply_path))
        client.settimeout(timeout_sec)
        client.sendto(json.dumps(message).encode("utf-8"), str(socket_path))
        data, _addr = client.recvfrom(MAX_PACKET_SIZE)
        return json.loads(data.decode("utf-8"))
    finally:
        client.close()
        try:
            reply_path.unlink()
        except FileNotFoundError:
            pass
