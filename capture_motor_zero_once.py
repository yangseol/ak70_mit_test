from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Any

import can
import yaml

from mit_packet import analyze_feedback_candidate, pack_mit_command


DEFAULT_CHANNEL = "can0"
DEFAULT_INTERFACE = "socketcan"
DEFAULT_MOTOR_ID = 0x00A
DEFAULT_CALIBRATION_PATH = "motor_calibration.yaml"
DEFAULT_NOTES = "Software zero captured by capture_motor_zero_once.py"
RECEIVE_DEADLINE_SEC = 1.0
RECV_POLL_TIMEOUT_SEC = 0.05
EXPECTED_ZERO_TORQUE_PACKET = bytes([0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x00])


def parse_motor_id(value: str) -> int:
    text = value.strip()
    if not text:
        raise argparse.ArgumentTypeError("motor id must not be empty")

    try:
        if text.lower().startswith("0x"):
            return int(text, 16)
        return int(text, 10)
    except ValueError:
        try:
            return int(text, 16)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(f"invalid motor id: {value!r}") from exc


def format_motor_id(motor_id: int) -> str:
    return f"0x{motor_id:03X}"


def default_motor_name(motor_id: int) -> str:
    return f"motor_{motor_id:03X}"


def format_packet(packet: bytes) -> str:
    return packet.hex(" ").upper()


def confirm_before_opening_bus(channel: str, motor_id: int) -> bool:
    print(f"channel: {channel}")
    print(f"motor_id: {format_motor_id(motor_id)}")
    print("This will send ONE zero-torque command to capture current raw position as software zero.")
    confirmation = input("Type YES to continue: ")
    return confirmation == "YES"


def receive_feedback(bus: can.BusABC, motor_id: int, packet: bytes) -> bytes | None:
    deadline = time.monotonic() + RECEIVE_DEADLINE_SEC

    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            break

        msg = bus.recv(timeout=min(RECV_POLL_TIMEOUT_SEC, remaining))
        if msg is None:
            continue

        if msg.arbitration_id != motor_id:
            continue

        if len(msg.data) != 8:
            continue

        raw_bytes = bytes(msg.data)

        if raw_bytes == packet:
            print("[Rx SKIP] local echo packet")
            continue

        if raw_bytes[0] != (motor_id & 0xFF):
            continue

        return raw_bytes

    return None


def load_calibration_file(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"motors": {}}

    with path.open("r", encoding="utf-8") as f:
        calibration = yaml.safe_load(f)

    if calibration is None:
        return {"motors": {}}
    if not isinstance(calibration, dict):
        raise ValueError(f"Invalid calibration YAML root: {path}")

    motors = calibration.setdefault("motors", {})
    if not isinstance(motors, dict):
        raise ValueError(f"Invalid calibration YAML motors section: {path}")

    return calibration


def write_calibration_file(path: Path, calibration: dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(calibration, f, sort_keys=False, allow_unicode=True)


def build_motor_calibration_entry(name: str, raw_p_uint: int, raw_pos_rad: float, notes: str) -> dict[str, Any]:
    return {
        "name": name,
        "raw_zero_p_uint": f"0x{raw_p_uint:04X}",
        "raw_zero_pos_rad": float(raw_pos_rad),
        "zero_command_persistent": False,
        "direction_sign": 1,
        "notes": notes,
    }


def print_captured_position(motor_id: int, raw_p_uint: int, raw_pos_rad: float) -> None:
    print("[Captured Raw Position]")
    print(f"motor_id: {format_motor_id(motor_id)}")
    print(f"raw_zero_p_uint: 0x{raw_p_uint:04X}")
    print(f"raw_zero_pos_rad: {raw_pos_rad:.6f}")


def save_captured_zero(motor_id: int, name: str, raw_p_uint: int, raw_pos_rad: float, notes: str) -> int:
    calibration_path = Path(DEFAULT_CALIBRATION_PATH)
    motor_key = format_motor_id(motor_id)

    try:
        calibration = load_calibration_file(calibration_path)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"Failed to load {calibration_path}: {exc}")
        return 1

    motors = calibration["motors"]
    if motor_key in motors:
        print(f"WARNING: existing calibration for {motor_key} will be updated if you type SAVE.")

    print("Save this as software zero in motor_calibration.yaml?")
    confirmation = input("Type SAVE to write file: ")
    if confirmation != "SAVE":
        print("Aborted. YAML file was not modified.")
        return 0

    motors[motor_key] = build_motor_calibration_entry(name, raw_p_uint, raw_pos_rad, notes)

    try:
        write_calibration_file(calibration_path, calibration)
    except (OSError, yaml.YAMLError) as exc:
        print(f"Failed to write {calibration_path}: {exc}")
        return 1

    print(f"Saved software zero for {motor_key} to {calibration_path}")
    return 0


def capture_once(channel: str, motor_id: int) -> dict[str, float | int] | None:
    packet = pack_mit_command(0.0, 0.0, 0.0, 0.0, 0.0)
    if packet != EXPECTED_ZERO_TORQUE_PACKET:
        print(
            "Refusing to transmit: zero-torque packet mismatch "
            f"actual={format_packet(packet)} expected={format_packet(EXPECTED_ZERO_TORQUE_PACKET)}"
        )
        return None

    if not confirm_before_opening_bus(channel, motor_id):
        print("Aborted. No CAN bus opened and no command transmitted.")
        return None

    bus = None
    try:
        bus = can.Bus(interface=DEFAULT_INTERFACE, channel=channel)
        msg = can.Message(arbitration_id=motor_id, data=packet, is_extended_id=False)
        bus.send(msg)

        raw_bytes = receive_feedback(bus, motor_id, packet)
        if raw_bytes is None:
            print("No feedback received")
            return None

        candidate = analyze_feedback_candidate(raw_bytes)
        if candidate is None:
            print("No feedback received")
            return None

        return {
            "raw_zero_p_uint": candidate["p_uint"],
            "raw_zero_pos_rad": candidate["candidate_position_rad"],
        }
    finally:
        if bus is not None:
            bus.shutdown()


def run_once(channel: str, motor_id: int, name: str, notes: str) -> int:
    captured = capture_once(channel, motor_id)
    if captured is None:
        return 0

    raw_p_uint = int(captured["raw_zero_p_uint"])
    raw_pos_rad = float(captured["raw_zero_pos_rad"])
    print_captured_position(motor_id, raw_p_uint, raw_pos_rad)
    return save_captured_zero(motor_id, name, raw_p_uint, raw_pos_rad, notes)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Capture the current raw AK70-10 position as software zero."
    )
    parser.add_argument("--channel", default=DEFAULT_CHANNEL)
    parser.add_argument("--motor-id", type=parse_motor_id, default=DEFAULT_MOTOR_ID)
    parser.add_argument("--name", default=None)
    parser.add_argument("--notes", default=DEFAULT_NOTES)
    args = parser.parse_args()

    motor_name = args.name if args.name is not None else default_motor_name(args.motor_id)
    raise SystemExit(run_once(args.channel, args.motor_id, motor_name, args.notes))


if __name__ == "__main__":
    main()
