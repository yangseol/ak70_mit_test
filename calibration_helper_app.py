from __future__ import annotations

import argparse
import math
import time
from pathlib import Path
from typing import Any

import can
import yaml

from calibration import apply_software_offset, load_motor_calibration
from mit_packet import analyze_feedback_candidate, pack_mit_command


DEFAULT_CHANNEL = "can0"
DEFAULT_INTERFACE = "socketcan"
DEFAULT_CALIBRATION_PATH = "motor_calibration.yaml"
DEFAULT_CAPTURE_NOTES = "Software zero captured by calibration_helper_app.py"
DETECT_START_ID = 0x001
DETECT_END_ID = 0x00A
CANDIDATE_DEADLINE_SEC = 0.03
DETECT_RECV_TIMEOUT_SEC = 0.005
READ_DEADLINE_SEC = 1.0
READ_RECV_TIMEOUT_SEC = 0.05
EXPECTED_ZERO_TORQUE_PACKET = bytes([0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x00])


def format_motor_id(motor_id: int) -> str:
    return f"0x{motor_id:03X}"


def format_packet(packet: bytes) -> str:
    return packet.hex(" ").upper()


def build_zero_torque_packet() -> bytes | None:
    packet = pack_mit_command(0.0, 0.0, 0.0, 0.0, 0.0)
    if packet == EXPECTED_ZERO_TORQUE_PACKET:
        return packet

    print(
        "Refusing to transmit: zero-torque packet mismatch "
        f"actual={format_packet(packet)} expected={format_packet(EXPECTED_ZERO_TORQUE_PACKET)}"
    )
    return None


def has_valid_detection_response(bus: can.BusABC, candidate_id: int, packet: bytes) -> bool:
    deadline = time.monotonic() + CANDIDATE_DEADLINE_SEC

    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            break

        msg = bus.recv(timeout=min(DETECT_RECV_TIMEOUT_SEC, remaining))
        if msg is None:
            continue

        raw_bytes = bytes(msg.data)
        if raw_bytes == packet:
            continue

        if msg.arbitration_id != candidate_id:
            continue

        if len(msg.data) != 8:
            continue

        if raw_bytes[0] != (candidate_id & 0xFF):
            continue

        return True

    return False


def auto_detect_motor_id(channel: str, interface: str) -> int | None:
    packet = build_zero_torque_packet()
    if packet is None:
        return None

    bus = None
    try:
        bus = can.Bus(interface=interface, channel=channel)
        for candidate_id in range(DETECT_START_ID, DETECT_END_ID + 1):
            msg = can.Message(arbitration_id=candidate_id, data=packet, is_extended_id=False)
            bus.send(msg)

            if has_valid_detection_response(bus, candidate_id, packet):
                return candidate_id
    finally:
        if bus is not None:
            bus.shutdown()

    return None


def receive_feedback_once(bus: can.BusABC, motor_id: int, packet: bytes) -> bytes | None:
    deadline = time.monotonic() + READ_DEADLINE_SEC

    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            break

        msg = bus.recv(timeout=min(READ_RECV_TIMEOUT_SEC, remaining))
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


def read_raw_position_once(channel: str, interface: str, motor_id: int) -> float | None:
    packet = build_zero_torque_packet()
    if packet is None:
        return None

    bus = None
    try:
        bus = can.Bus(interface=interface, channel=channel)
        msg = can.Message(arbitration_id=motor_id, data=packet, is_extended_id=False)
        bus.send(msg)

        raw_bytes = receive_feedback_once(bus, motor_id, packet)
        if raw_bytes is None:
            print("No feedback received")
            return None

        candidate = analyze_feedback_candidate(raw_bytes)
        if candidate is None:
            print("No feedback received")
            return None

        return float(candidate["candidate_position_rad"])
    finally:
        if bus is not None:
            bus.shutdown()


def capture_raw_zero_once(channel: str, interface: str, motor_id: int) -> dict[str, float | int] | None:
    packet = build_zero_torque_packet()
    if packet is None:
        return None

    bus = None
    try:
        bus = can.Bus(interface=interface, channel=channel)
        msg = can.Message(arbitration_id=motor_id, data=packet, is_extended_id=False)
        bus.send(msg)

        raw_bytes = receive_feedback_once(bus, motor_id, packet)
        if raw_bytes is None:
            print("No feedback received")
            return None

        candidate = analyze_feedback_candidate(raw_bytes)
        if candidate is None:
            print("No feedback received")
            return None

        return {
            "raw_zero_p_uint": int(candidate["p_uint"]),
            "raw_zero_pos_rad": float(candidate["candidate_position_rad"]),
        }
    finally:
        if bus is not None:
            bus.shutdown()


def motor_calibration_key(motor_id: int) -> str:
    return format_motor_id(motor_id)


def find_motor_calibration(calibration: dict[str, Any], motor_id: int) -> dict[str, Any] | None:
    motors = calibration.get("motors", {})
    if not isinstance(motors, dict):
        return None
    entry = motors.get(motor_calibration_key(motor_id))
    return entry if isinstance(entry, dict) else None


def print_read_result(raw_pos_rad: float, motor_id: int) -> None:
    try:
        calibration = load_motor_calibration()
    except Exception as exc:
        print("[Read Result]")
        print(f"raw_pos_rad: {raw_pos_rad:.6f}")
        print(f"Calibration could not be loaded: {exc}")
        print("joint_rad was not calculated.")
        return

    if find_motor_calibration(calibration, motor_id) is None:
        print("[Read Result]")
        print(f"raw_pos_rad: {raw_pos_rad:.6f}")
        print(f"Calibration not found for motor ID: {format_motor_id(motor_id)}")
        print("joint_rad was not calculated.")
        return

    try:
        joint_rad = apply_software_offset(raw_pos_rad, motor_id, calibration)
    except KeyError:
        print("[Read Result]")
        print(f"raw_pos_rad: {raw_pos_rad:.6f}")
        print(f"Calibration not found for motor ID: {format_motor_id(motor_id)}")
        print("joint_rad was not calculated.")
        return

    joint_deg = math.degrees(joint_rad)
    print("[Read Result]")
    print(f"raw_pos_rad: {raw_pos_rad:.6f} | joint_rad: {joint_rad:.6f} | joint_deg: {joint_deg:.2f}")


def print_calibration_entry(entry: dict[str, Any]) -> None:
    preferred_keys = [
        "name",
        "raw_zero_p_uint",
        "raw_zero_pos_rad",
        "zero_command_persistent",
        "direction_sign",
        "notes",
    ]

    printed = set()
    for key in preferred_keys:
        if key in entry:
            print(f"{key}: {entry[key]}")
            printed.add(key)

    for key, value in entry.items():
        if key not in printed:
            print(f"{key}: {value}")


def load_calibration_file(path: str = DEFAULT_CALIBRATION_PATH) -> dict[str, Any]:
    calibration_path = Path(path)
    if not calibration_path.exists():
        return {"motors": {}}

    with calibration_path.open("r", encoding="utf-8") as f:
        calibration = yaml.safe_load(f)

    if calibration is None:
        return {"motors": {}}
    if not isinstance(calibration, dict):
        raise ValueError(f"Invalid calibration YAML root: {path}")

    motors = calibration.setdefault("motors", {})
    if not isinstance(motors, dict):
        raise ValueError(f"Invalid calibration YAML motors section: {path}")

    return calibration


def write_calibration_file(path: str, calibration: dict[str, Any]) -> None:
    calibration_path = Path(path)
    with calibration_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(calibration, f, allow_unicode=True, sort_keys=False)


def build_calibration_entry(name: str, raw_p_uint: int, raw_pos_rad: float, notes: str) -> dict[str, Any]:
    return {
        "name": name,
        "raw_zero_p_uint": f"0x{raw_p_uint:04X}",
        "raw_zero_pos_rad": float(raw_pos_rad),
        "zero_command_persistent": False,
        "direction_sign": 1,
        "notes": notes,
    }


def default_motor_name(motor_id: int) -> str:
    return f"motor_{motor_id:03X}"


def capture_current_position_as_zero(channel: str, interface: str, selected_motor_id: int | None) -> None:
    if selected_motor_id is None:
        print("Error: No motor selected. Please run auto-detection first.")
        return

    captured = capture_raw_zero_once(channel, interface, selected_motor_id)
    if captured is None:
        return

    raw_p_uint = int(captured["raw_zero_p_uint"])
    raw_pos_rad = float(captured["raw_zero_pos_rad"])
    motor_key = format_motor_id(selected_motor_id)

    print("[Captured Raw Position]")
    print(f"motor_id: {motor_key}")
    print(f"raw_zero_p_uint: 0x{raw_p_uint:04X}")
    print(f"raw_zero_pos_rad: {raw_pos_rad:.6f}")

    try:
        calibration = load_calibration_file(DEFAULT_CALIBRATION_PATH)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"Failed to load {DEFAULT_CALIBRATION_PATH}: {exc}")
        return

    motors = calibration["motors"]
    if motor_key in motors:
        print(f"WARNING: existing calibration for {motor_key} will be updated if you type SAVE.")

    print("This will update motor_calibration.yaml.")
    confirmation = input("Type SAVE to write this as software zero: ")
    if confirmation != "SAVE":
        print("Aborted. motor_calibration.yaml was not modified.")
        return

    motors[motor_key] = build_calibration_entry(
        name=default_motor_name(selected_motor_id),
        raw_p_uint=raw_p_uint,
        raw_pos_rad=raw_pos_rad,
        notes=DEFAULT_CAPTURE_NOTES,
    )

    try:
        write_calibration_file(DEFAULT_CALIBRATION_PATH, calibration)
    except (OSError, yaml.YAMLError) as exc:
        print(f"Failed to write {DEFAULT_CALIBRATION_PATH}: {exc}")
        return

    print(f"Saved software zero for {motor_key} to {DEFAULT_CALIBRATION_PATH}")


def show_saved_calibration(selected_motor_id: int | None) -> None:
    try:
        calibration = load_motor_calibration()
    except Exception as exc:
        print("[Saved Calibration]")
        print(f"Calibration could not be loaded: {exc}")
        return

    motors = calibration.get("motors", {})
    if not isinstance(motors, dict) or not motors:
        print("[Saved Calibration]")
        print("No saved motor calibration entries found.")
        return

    print("[Saved Calibration]")
    if selected_motor_id is not None:
        selected_key = format_motor_id(selected_motor_id)
        print(f"Selected motor: {selected_key}")
        entry = motors.get(selected_key)
        if isinstance(entry, dict):
            print()
            print_calibration_entry(entry)
            return

        print(f"Calibration not found for selected motor: {selected_key}")
        print()

    for motor_key in sorted(motors):
        print(f"{motor_key}:")
        entry = motors[motor_key]
        if isinstance(entry, dict):
            print_calibration_entry(entry)
        else:
            print(entry)
        print()


def print_menu(selected_motor_id: int | None) -> None:
    selected_text = format_motor_id(selected_motor_id) if selected_motor_id is not None else "None"
    print("=========================================")
    print("=== T-Motor AK70-10 Calibration Helper ===")
    print(f"Current Selected Motor: [ {selected_text} ]")
    print("=========================================")
    print("[1] Auto-detect connected motor ID")
    print("[2] Read current joint angle")
    print("[3] Show saved calibration")
    print("[4] Capture current position as software zero")
    print("[5] Exit")
    print("-----------------------------------------")


def run_app(channel: str, interface: str) -> int:
    selected_motor_id: int | None = None
    running = True

    while running:
        print_menu(selected_motor_id)
        selection = input("Select menu [1-5]: ").strip()

        if selection == "1":
            detected_id = auto_detect_motor_id(channel, interface)
            if detected_id is None:
                print(f"No motor detected in range {format_motor_id(DETECT_START_ID)}~{format_motor_id(DETECT_END_ID)}")
            else:
                selected_motor_id = detected_id
                print(f"Detected and selected motor ID: {format_motor_id(selected_motor_id)}")
        elif selection == "2":
            if selected_motor_id is None:
                print("Error: No motor selected. Please run auto-detection first.")
                continue

            raw_pos_rad = read_raw_position_once(channel, interface, selected_motor_id)
            if raw_pos_rad is not None:
                print_read_result(raw_pos_rad, selected_motor_id)
        elif selection == "3":
            show_saved_calibration(selected_motor_id)
        elif selection == "4":
            capture_current_position_as_zero(channel, interface, selected_motor_id)
        elif selection == "5":
            running = False
        else:
            print("Invalid selection.")

    return 0


def main() -> None:
    parser = argparse.ArgumentParser(description="Read-only AK70-10 calibration helper dashboard.")
    parser.add_argument("--channel", default=DEFAULT_CHANNEL)
    parser.add_argument("--interface", default=DEFAULT_INTERFACE)
    args = parser.parse_args()

    raise SystemExit(run_app(args.channel, args.interface))


if __name__ == "__main__":
    main()
