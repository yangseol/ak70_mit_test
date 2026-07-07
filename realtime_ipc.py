"""Unix datagram IPC and singleton locking for the realtime controller."""

from __future__ import annotations

import fcntl
import json
import math
import os
import socket
import stat
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from motor_profiles import (
    AK70_KD_MAX,
    AK70_KD_MIN,
    AK70_KP_MAX,
    AK70_KP_MIN,
    format_motor_id,
    get_motor_profile,
    is_ak45,
    normalize_motor_id,
)


AK70_TARGET_LIMIT_DEG = 120.0
AK70_TORQUE_FF_MIN_NM = -25.0
AK70_TORQUE_FF_MAX_NM = 25.0


BASE_DIR = Path(__file__).resolve().parent
DEFAULT_SOCKET_PATH = BASE_DIR / ".ak_realtime_controller.sock"
DEFAULT_LOCK_PATH = BASE_DIR / ".ak_realtime_controller.lock"
MAX_PACKET_SIZE = 32768
ALLOWED_COMMANDS = {
    "PING",
    "STATUS",
    "ARM",
    "HEARTBEAT",
    "START_MOTORS",
    "SET_TARGETS",
    "SET_STREAM_TARGETS",
    "SET_TRAJECTORY",
    "SET_TORQUE_FF",
    "SET_GAINS",
    "RELEASE_IDS",
    "RELEASE_SESSION",
    "ESTOP",
    "CLEAR_TARGETS",
    "DISARM",
    "CONFIRM_HOME",
    "SAVE_SOFTWARE_ZERO",
    "RELOAD_CALIBRATION",
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
    tmp_path = BASE_DIR / f".ak_realtime_probe_{os.getpid()}_{time.time_ns()}.sock"
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
        "updates",
        "torque_ff_nm",
    }
    unknown = set(message) - allowed_fields
    if unknown:
        raise ValueError(f"unknown fields: {sorted(unknown)}")
    if command in {"SET_TARGETS", "SET_STREAM_TARGETS"}:
        targets = message.get("targets")
        if not isinstance(targets, list) or not targets:
            raise ValueError(f"{command} requires non-empty targets list")
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
                "torque_ff_nm",
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
            if "torque_ff_nm" in target:
                value = float(target["torque_ff_nm"])
                if not is_ak45(motor_id) and (not math.isfinite(value) or not AK70_TORQUE_FF_MIN_NM <= value <= AK70_TORQUE_FF_MAX_NM):
                    raise ValueError(
                        f"AK70 torque_ff_nm must be {AK70_TORQUE_FF_MIN_NM:g}..{AK70_TORQUE_FF_MAX_NM:g}"
                    )
            if not is_ak45(motor_id):
                if "kp" in target and not AK70_KP_MIN < float(target["kp"]) <= AK70_KP_MAX:
                    raise ValueError(f"AK70 Kp must be > {AK70_KP_MIN:g} and <= {AK70_KP_MAX:g}")
                if "kd" in target and not AK70_KD_MIN <= float(target["kd"]) <= AK70_KD_MAX:
                    raise ValueError(f"AK70 Kd must be {AK70_KD_MIN:g}..{AK70_KD_MAX:g}")
                target_deg_value = target.get("target_deg", target.get("position_deg"))
                if target_deg_value is not None and not -AK70_TARGET_LIMIT_DEG <= float(target_deg_value) <= AK70_TARGET_LIMIT_DEG:
                    raise ValueError(f"AK70 target_deg must be -{AK70_TARGET_LIMIT_DEG:g}..+{AK70_TARGET_LIMIT_DEG:g}")
    if command == "SET_TRAJECTORY":
        motor_id = normalize_motor_id(str(message.get("motor_id")))
        get_motor_profile(motor_id)
        if not is_ak45(motor_id):
            if "kp" in message and not AK70_KP_MIN < float(message["kp"]) <= AK70_KP_MAX:
                raise ValueError(f"AK70 Kp must be > {AK70_KP_MIN:g} and <= {AK70_KP_MAX:g}")
            if "kd" in message and not AK70_KD_MIN <= float(message["kd"]) <= AK70_KD_MAX:
                raise ValueError(f"AK70 Kd must be {AK70_KD_MIN:g}..{AK70_KD_MAX:g}")
            if "torque_ff_nm" in message:
                value = float(message["torque_ff_nm"])
                if not math.isfinite(value) or not AK70_TORQUE_FF_MIN_NM <= value <= AK70_TORQUE_FF_MAX_NM:
                    raise ValueError(
                        f"AK70 torque_ff_nm must be {AK70_TORQUE_FF_MIN_NM:g}..{AK70_TORQUE_FF_MAX_NM:g}"
                    )
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
            if not is_ak45(motor_id) and not -AK70_TARGET_LIMIT_DEG <= target <= AK70_TARGET_LIMIT_DEG:
                raise ValueError(f"AK70 target_deg must be -{AK70_TARGET_LIMIT_DEG:g}..+{AK70_TARGET_LIMIT_DEG:g}")
    if command == "SET_TORQUE_FF":
        if not message.get("session_token"):
            raise ValueError("SET_TORQUE_FF requires session_token")
        updates = message.get("updates")
        if not isinstance(updates, dict) or not updates:
            raise ValueError("SET_TORQUE_FF requires non-empty updates object")
        seen: set[int] = set()
        for raw_motor_id, raw_value in updates.items():
            motor_id = normalize_motor_id(str(raw_motor_id))
            get_motor_profile(motor_id)
            if is_ak45(motor_id):
                raise ValueError("SET_TORQUE_FF is AK70-only")
            if motor_id in seen:
                raise ValueError(f"duplicate torque FF ID {format_motor_id(motor_id)}")
            seen.add(motor_id)
            value = float(raw_value)
            if not math.isfinite(value) or not AK70_TORQUE_FF_MIN_NM <= value <= AK70_TORQUE_FF_MAX_NM:
                raise ValueError(f"AK70 torque_ff_nm must be {AK70_TORQUE_FF_MIN_NM:g}..{AK70_TORQUE_FF_MAX_NM:g}")
    if command == "SET_GAINS":
        if not message.get("session_token"):
            raise ValueError("SET_GAINS requires session_token")
        updates = message.get("updates")
        if not isinstance(updates, dict) or not updates:
            raise ValueError("SET_GAINS requires non-empty updates object")
        for raw_motor_id, raw_gains in updates.items():
            motor_id = normalize_motor_id(str(raw_motor_id))
            profile = get_motor_profile(motor_id)
            if not isinstance(raw_gains, dict) or not raw_gains:
                raise ValueError(f"SET_GAINS invalid update for {format_motor_id(motor_id)}")
            unknown_gains = set(raw_gains) - {"kp", "kd"}
            if unknown_gains:
                raise ValueError(f"SET_GAINS unknown gain fields: {sorted(unknown_gains)}")
            for key, raw_value in raw_gains.items():
                value = float(raw_value)
                if not math.isfinite(value):
                    raise ValueError(f"{key} must be finite")
                if key == "kp":
                    low_ok = value > AK70_KP_MIN if not is_ak45(motor_id) else value >= profile.kp_min
                    high = AK70_KP_MAX if not is_ak45(motor_id) else profile.kp_max
                    if not low_ok or value > high:
                        raise ValueError(f"{format_motor_id(motor_id)} Kp outside allowed range")
                if key == "kd":
                    low = AK70_KD_MIN if not is_ak45(motor_id) else profile.kd_min
                    high = AK70_KD_MAX if not is_ak45(motor_id) else profile.kd_max
                    if not low <= value <= high:
                        raise ValueError(f"{format_motor_id(motor_id)} Kd outside allowed range")
    if command in {
        "START_MOTORS",
        "SET_STREAM_TARGETS",
        "SET_GAINS",
        "SAVE_SOFTWARE_ZERO",
        "RELOAD_CALIBRATION",
        "RELEASE_IDS",
        "RELEASE_SESSION",
        "HEARTBEAT",
    } and not message.get("session_token"):
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
    reply_path = BASE_DIR / f".ak_realtime_client_{os.getpid()}_{time.time_ns()}.sock"
    reply_created = False
    try:
        client.bind(str(reply_path))
        reply_created = True
        client.settimeout(timeout_sec)
        client.sendto(json.dumps(message).encode("utf-8"), str(socket_path))
        try:
            data, _addr = client.recvfrom(MAX_PACKET_SIZE)
        except socket.timeout as exc:
            command = message.get("command", "UNKNOWN")
            raise TimeoutError(
                f"IPC request timeout: command={command}, timeout_sec={timeout_sec}, "
                f"controller_socket={socket_path}, reply_socket={reply_path}"
            ) from exc
        return json.loads(data.decode("utf-8"))
    finally:
        client.close()
        if reply_created:
            try:
                reply_path.unlink()
            except FileNotFoundError:
                pass
