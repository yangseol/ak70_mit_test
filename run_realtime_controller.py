#!/usr/bin/env python3
"""Run the singleton realtime mixed AK controller."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import can

from ak45_calibration import load_ak45_calibration
from ak_realtime_core import (
    DEFAULT_RATE_HZ,
    ControlMode,
    ControllerMode,
    RealtimeCore,
    ReleaseReason,
    joint_deg_to_raw_rad,
    targets_from_ipc,
)
from mixed_mit_packet import MitCommand, pack_checked_commands, pack_mit_command, unpack_mit_feedback
from motor_profiles import format_motor_id, get_motor_profile, is_ak45, normalize_motor_id
from realtime_ipc import (
    DEFAULT_LOCK_PATH,
    DEFAULT_SOCKET_PATH,
    MAX_PACKET_SIZE,
    ControllerAlreadyRunning,
    LiveSocketExists,
    acquire_controller_lock,
    bind_controller_socket,
    validate_message,
)


DEFAULT_CHANNEL = "can0"
MIT_ENTER_PACKET = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
EXPECTED_ZERO_TORQUE_PACKET = bytes([0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x00])


def parse_motor_ids(value: str) -> list[int]:
    ids = [normalize_motor_id(part.strip()) for part in value.split(",") if part.strip()]
    if not ids:
        raise argparse.ArgumentTypeError("at least one motor ID is required")
    if len(set(ids)) != len(ids):
        raise argparse.ArgumentTypeError("duplicate motor ID")
    for motor_id in ids:
        get_motor_profile(motor_id)
    return sorted(ids)


def parse_control_mode(value: str) -> ControlMode:
    try:
        return ControlMode(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid control mode: {value}") from exc


def verify_zero_torque_packet() -> None:
    packet = pack_mit_command(0x001, 0.0, 0.0, 0.0, 0.0, 0.0)
    if packet != EXPECTED_ZERO_TORQUE_PACKET:
        raise RuntimeError(f"zero torque packet mismatch: {packet.hex(' ')}")


class RealtimeController:
    def __init__(
        self,
        channel: str,
        motor_ids: list[int],
        rate_hz: float,
        dry_run: bool,
        control_mode: ControlMode,
        calibration_path: Path,
        process_owner_token: str | None = None,
    ) -> None:
        self.channel = channel
        self.motor_ids = motor_ids
        self.rate_hz = rate_hz
        self.dry_run = dry_run
        self.core = RealtimeCore(
            motor_ids=motor_ids,
            rate_hz=rate_hz,
            control_mode=control_mode,
            calibration_path=calibration_path,
        )
        self.running = True
        self.bus: can.BusABC | None = None
        self.process_owner_token = process_owner_token

    def open_bus(self) -> None:
        verify_zero_torque_packet()
        if self.dry_run:
            return
        self.bus = can.Bus(interface="socketcan", channel=self.channel)

    def close_bus(self) -> None:
        if self.bus is not None:
            self.bus.shutdown()
            self.bus = None

    def enter_mit_for(self, motor_ids: list[int]) -> None:
        if self.bus is None:
            for motor_id in motor_ids:
                self.core.motors[motor_id].mit_entered = True
            return
        for motor_id in sorted(set(motor_ids)):
            runtime = self.core.motors[motor_id]
            if runtime.mit_entered:
                continue
            self.bus.send(can.Message(arbitration_id=motor_id, data=MIT_ENTER_PACKET, is_extended_id=False))
            runtime.mit_entered = True
            time.sleep(0.02)

    def send_zero_torque_for(self, motor_ids: list[int], reason: ReleaseReason) -> None:
        self.try_send_zero_torque_for(motor_ids, reason)

    def try_send_zero_torque_for(self, motor_ids: list[int], reason: ReleaseReason) -> tuple[list[str], list[str]]:
        sent: list[str] = []
        failed: list[str] = []
        if not motor_ids:
            return sent, failed
        verify_zero_torque_packet()
        if self.bus is None:
            if self.dry_run:
                return [format_motor_id(motor_id) for motor_id in sorted(set(motor_ids))], []
            return [], [format_motor_id(motor_id) for motor_id in sorted(set(motor_ids))]
        for motor_id in sorted(set(motor_ids)):
            if is_ak45(motor_id) and reason != ReleaseReason.LEGACY_FINITE_CLI:
                continue
            try:
                self.bus.send(
                    can.Message(
                        arbitration_id=motor_id,
                        data=pack_mit_command(motor_id, 0.0, 0.0, 0.0, 0.0, 0.0),
                        is_extended_id=False,
                    )
                )
                sent.append(format_motor_id(motor_id))
            except Exception:
                failed.append(format_motor_id(motor_id))
        return sent, failed

    def drain_release_events(self) -> tuple[list[str], list[str]]:
        sent_all: list[str] = []
        failed_all: list[str] = []
        for reason, motor_ids in self.core.consume_release_events():
            sent, failed = self.try_send_zero_torque_for(motor_ids, reason)
            sent_all.extend(sent)
            failed_all.extend(failed)
        return sorted(set(sent_all)), sorted(set(failed_all))

    def wait_for_fresh_feedback(self, motor_ids: list[int], timeout_sec: float = 0.6) -> None:
        if self.dry_run:
            now = time.monotonic()
            for motor_id in motor_ids:
                runtime = self.core.motors[motor_id]
                if not self.core.has_fresh_feedback(motor_id, now):
                    self.core.on_feedback(motor_id, runtime.feedback_position_rad if runtime.feedback_position_rad is not None else 0.0, now=now)
            return
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.receive_feedback_nonblocking()
            now = time.monotonic()
            if all(self.core.has_fresh_feedback(motor_id, now) for motor_id in motor_ids):
                return
            time.sleep(0.005)
        missing = [format_motor_id(motor_id) for motor_id in motor_ids if not self.core.has_fresh_feedback(motor_id)]
        raise RuntimeError(f"FEEDBACK_REQUIRED: fresh feedback missing for {', '.join(missing)}")

    def _base_response(self, message: dict, ok: bool, **payload) -> dict:
        response = {
            "ok": ok,
            "request_id": message.get("request_id"),
            "command": message.get("command"),
        }
        response.update(payload)
        if "status" not in response:
            response["status"] = self.core.status()
        return response

    def handle_message(self, message: dict) -> dict:
        command = message["command"]
        if command == "PING":
            return self._base_response(message, True, response="PONG")
        if command == "STATUS":
            return self._base_response(message, True)
        if command == "ARM":
            info = self.core.arm(
                owner=str(message.get("owner", "ipc")),
                session_token=message.get("session_token"),
            )
            self.enter_mit_for([m for m in self.motor_ids if not is_ak45(m)])
            return self._base_response(message, True, **info)
        if command == "HEARTBEAT":
            return self.core.heartbeat(
                session_token=str(message["session_token"]),
                heartbeat_seq=int(message["heartbeat_seq"]),
                generated_monotonic=float(message["generated_monotonic"]),
                request_id=message.get("request_id"),
            )
        if command == "DISARM":
            active = list(self.core.latest_targets)
            self.core.disarm()
            self.send_zero_torque_for([m for m in active if not is_ak45(m)], ReleaseReason.RELEASE_IDS)
            return self._base_response(message, True)
        if command == "CLEAR_TARGETS":
            self.core.clear_targets()
            return self._base_response(message, True)
        if command == "SET_TARGETS":
            targets = targets_from_ipc(message, self.core)
            self.enter_mit_for([target.motor_id for target in targets if not is_ak45(target.motor_id)])
            self.wait_for_fresh_feedback([
                target.motor_id
                for target in targets
                if not is_ak45(target.motor_id) and self.core.needs_fresh_feedback_for_target(target.motor_id, target.session_token)
            ])
            self.core.set_latest_targets(targets)
            return self._base_response(message, True, target_generation=self.core.target_generation)
        if command == "SET_TRAJECTORY":
            motor_id = normalize_motor_id(str(message["motor_id"]))
            self.enter_mit_for([motor_id])
            if self.core.needs_fresh_feedback_for_target(motor_id, message.get("session_token")):
                self.wait_for_fresh_feedback([motor_id])
            from ak_realtime_core import TrajectoryWaypoint

            self.core.set_trajectory(
                motor_id=motor_id,
                waypoints=[
                    TrajectoryWaypoint(float(w["target_deg"]), float(w.get("duration_sec", w.get("duration"))))
                    for w in message["waypoints"]
                ],
                kp=float(message.get("kp", get_motor_profile(motor_id).default_kp)),
                kd=float(message.get("kd", get_motor_profile(motor_id).default_kd)),
                max_following_error_deg=float(message.get("max_following_error_deg", 60.0)),
                control_group_id=str(message.get("control_group_id") or f"trajectory-{message.get('plan_id')}"),
                plan_id=str(message.get("plan_id")),
                session_token=message.get("session_token"),
                session_epoch=message.get("session_epoch"),
                display_mode=str(message.get("display_mode", "target")),
            )
            return self._base_response(message, True, plan_id=message.get("plan_id"))
        if command == "RELEASE_IDS":
            result = self.core.release_ids(
                message.get("motor_ids", []),
                session_token=message.get("session_token"),
                expected_generations=message.get("expected_generations"),
            )
            sent, failed = self.drain_release_events()
            result["zero_torque_sent_ids"] = sent
            result["zero_torque_failed_ids"] = failed
            return self._base_response(message, not result["errors"], **result)
        if command == "RELEASE_SESSION":
            result = self.core.release_session(str(message["session_token"]))
            sent, failed = self.drain_release_events()
            result["zero_torque_sent_ids"] = sent
            result["zero_torque_failed_ids"] = failed
            return self._base_response(message, not result["errors"], **result)
        if command == "ESTOP":
            result = self.core.estop()
            sent, failed = self.drain_release_events()
            result["zero_torque_sent_ids"] = sent
            result["zero_torque_failed_ids"] = failed
            return self._base_response(message, True, **result)
        if command == "CONFIRM_HOME":
            motor_id = normalize_motor_id(str(message["motor_id"]))
            calibration = load_ak45_calibration()
            self.core.confirm_home(motor_id, raw_pos_rad=0.0, calibration=calibration)
            return self._base_response(message, True)
        if command == "SHUTDOWN":
            if self.process_owner_token and message.get("process_owner_token") != self.process_owner_token:
                return self._base_response(message, False, error="process owner token mismatch")
            result = self.core.shutdown()
            sent, failed = self.drain_release_events()
            result["zero_torque_sent_ids"] = sent
            result["zero_torque_failed_ids"] = failed
            self.running = False
            return self._base_response(message, True, response="SHUTDOWN", **result)
        return self._base_response(message, False, error="unhandled command")

    def control_cycle(self) -> None:
        commands = self.core.compute_cycle_commands()
        self.drain_release_events()
        if not commands or self.bus is None:
            return
        mit_commands = [
            MitCommand(target.motor_id, target.position_rad, 0.0, target.kp, target.kd, 0.0)
            for target in commands.values()
        ]
        packets = pack_checked_commands(mit_commands)
        try:
            for motor_id, packet in packets.items():
                self.bus.send(can.Message(arbitration_id=motor_id, data=packet, is_extended_id=False))
        except Exception:
            self.core.on_bus_off_or_reconnect()
            self.drain_release_events()

    def receive_feedback_nonblocking(self) -> None:
        if self.bus is None:
            return
        while True:
            try:
                msg = self.bus.recv(timeout=0.0)
            except Exception:
                self.core.on_bus_off_or_reconnect()
                return
            if msg is None:
                return
            motor_id = normalize_motor_id(msg.arbitration_id)
            if motor_id not in self.core.motors:
                continue
            try:
                feedback = unpack_mit_feedback(motor_id, bytes(msg.data))
            except Exception:
                continue
            self.core.on_feedback(motor_id, feedback.position)

    def serve(self, bound_socket) -> int:
        self.open_bus()
        self.core.on_controller_restart()
        period = 1.0 / self.rate_hz
        next_cycle = time.monotonic()
        bound_socket.sock.setblocking(False)
        exit_code = 0
        try:
            while self.running:
                while True:
                    try:
                        data, addr = bound_socket.sock.recvfrom(MAX_PACKET_SIZE)
                    except BlockingIOError:
                        break
                    try:
                        message = validate_message(data)
                        response = self.handle_message(message)
                    except Exception as exc:
                        try:
                            raw = json.loads(data.decode("utf-8"))
                            request_id = raw.get("request_id")
                            command = raw.get("command")
                        except Exception:
                            request_id = None
                            command = None
                        response = {"ok": False, "request_id": request_id, "command": command, "error": str(exc), "status": self.core.status()}
                    if addr:
                        bound_socket.sock.sendto(json.dumps(response).encode("utf-8"), addr)

                self.receive_feedback_nonblocking()
                now = time.monotonic()
                if now >= next_cycle:
                    self.control_cycle()
                    next_cycle = now + period
                else:
                    time.sleep(min(0.005, next_cycle - now))
            return exit_code
        except KeyboardInterrupt:
            exit_code = 130
            self.release_active_on_exit("KeyboardInterrupt")
            return exit_code
        except Exception:
            exit_code = 1
            self.release_active_on_exit("controller exception")
            raise
        finally:
            if exit_code != 0:
                self.drain_release_events()
            self.close_bus()

    def release_active_on_exit(self, reason: str) -> None:
        active = [motor_id for motor_id in self.core.latest_targets if not is_ak45(motor_id)]
        if not active:
            return
        self.core.latest_targets.clear()
        self.core.plans.clear()
        self.core.mode = ControllerMode.DISARMED
        self.core.release_events.append((ReleaseReason.SHUTDOWN, active))
        print(f"controller exit release: {reason}: {', '.join(format_motor_id(m) for m in active)}")


def run_controller(
    channel: str,
    motor_ids: list[int],
    rate_hz: float,
    socket_path: Path,
    lock_path: Path,
    dry_run: bool,
    control_mode: ControlMode,
    calibration_path: Path,
    process_owner_token: str | None,
) -> int:
    lock = None
    bound = None
    try:
        lock = acquire_controller_lock(lock_path)
        bound = bind_controller_socket(socket_path)
        print(f"controller socket: {socket_path}")
        print(f"selected motors: {', '.join(format_motor_id(i) for i in motor_ids)}")
        print(f"control mode: {control_mode.value}")
        print("dry-run: no CAN bus opened" if dry_run else f"channel: {channel}")
        controller = RealtimeController(channel, motor_ids, rate_hz, dry_run, control_mode, calibration_path, process_owner_token)
        return controller.serve(bound)
    except ControllerAlreadyRunning as exc:
        print(f"Error: {exc}")
        return 2
    except LiveSocketExists as exc:
        print(f"Error: {exc}")
        return 2
    finally:
        if bound is not None:
            bound.close_and_cleanup()
        if lock is not None:
            lock.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Run singleton realtime mixed AK controller.")
    parser.add_argument("--channel", default=DEFAULT_CHANNEL)
    parser.add_argument(
        "--motor-ids",
        type=parse_motor_ids,
        default=parse_motor_ids("0x001,0x002,0x003,0x004,0x005,0x006,0x007,0x008,0x009,0x00A,0x00B,0x00C"),
    )
    parser.add_argument("--rate-hz", type=float, default=DEFAULT_RATE_HZ)
    parser.add_argument("--socket-path", type=Path, default=DEFAULT_SOCKET_PATH)
    parser.add_argument("--lock-path", type=Path, default=DEFAULT_LOCK_PATH)
    parser.add_argument("--control-mode", type=parse_control_mode, default=ControlMode.DEFAULT)
    parser.add_argument("--calibration-path", type=Path, default=Path("motor_calibration.yaml"))
    parser.add_argument("--process-owner-token")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if not math.isfinite(args.rate_hz) or not 10.0 <= args.rate_hz <= 100.0:
        parser.error("--rate-hz must be finite and 10..100")
    raise SystemExit(
        run_controller(
            args.channel,
            args.motor_ids,
            args.rate_hz,
            args.socket_path,
            args.lock_path,
            args.dry_run,
            args.control_mode,
            args.calibration_path,
            args.process_owner_token,
        )
    )


if __name__ == "__main__":
    main()
