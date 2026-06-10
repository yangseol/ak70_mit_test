"""여러 AK70 모터를 zero-torque read로 한 번씩 읽고 joint state를 출력하는 helper."""

from __future__ import annotations

import argparse
import math
import time

import can

from calibration import apply_software_offset, load_motor_calibration
from mit_packet import analyze_feedback_candidate, pack_mit_command


DEFAULT_CHANNEL = "can0"
DEFAULT_INTERFACE = "socketcan"
DEFAULT_MOTOR_IDS = (0x005, 0x007, 0x00A)
RECEIVE_DEADLINE_SEC = 1.0
RECV_POLL_TIMEOUT_SEC = 0.05
DRAIN_DURATION_SEC = 0.20
DRAIN_POLL_TIMEOUT_SEC = 0.01
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


def parse_motor_ids(value: str) -> list[int]:
    motor_ids = [parse_motor_id(part) for part in value.split(",")]
    if not motor_ids:
        raise argparse.ArgumentTypeError("at least one motor id is required")
    return motor_ids


def format_motor_id(motor_id: int) -> str:
    return f"0x{motor_id:03X}"


def format_motor_ids(motor_ids: list[int]) -> str:
    return ",".join(format_motor_id(motor_id) for motor_id in motor_ids)


def format_packet(packet: bytes) -> str:
    return packet.hex(" ").upper()


def confirm_before_opening_bus(channel: str, motor_ids: list[int]) -> bool:
    print("SAFETY WARNING")
    print("- zero-torque only")
    print("- no 0xFE")
    print("- no position control")
    print("- no nudge")
    print(f"channel: {channel}")
    print(f"motor_ids: {format_motor_ids(motor_ids)}")
    print("This will send ONE zero-torque MIT command to each listed motor.")
    confirmation = input("Type YES to continue: ")
    return confirmation == "YES"


def drain_rx_queue(bus: can.BusABC) -> int:
    deadline = time.monotonic() + DRAIN_DURATION_SEC
    drained = 0

    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            break

        msg = bus.recv(timeout=min(DRAIN_POLL_TIMEOUT_SEC, remaining))
        if msg is None:
            continue
        drained += 1

    return drained


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
            print(f"{format_motor_id(motor_id)} | [Rx SKIP] local echo packet")
            continue

        if raw_bytes[0] != (motor_id & 0xFF):
            continue

        return raw_bytes

    return None


def print_joint_state(raw_bytes: bytes, motor_id: int, calibration: dict) -> bool:
    candidate = analyze_feedback_candidate(raw_bytes)
    if candidate is None:
        print(f"{format_motor_id(motor_id)} | NO FEEDBACK")
        return False

    raw_pos = candidate["candidate_position_rad"]
    velocity_rad_s = candidate["candidate_velocity_rads"]
    effort = candidate["candidate_effort_from_torque_limit"]
    status_byte6 = candidate["status_byte6"]
    status_byte7 = candidate["status_byte7"]

    try:
        joint_position_rad = apply_software_offset(raw_pos, motor_id, calibration)
    except KeyError:
        print(
            f"{format_motor_id(motor_id)} | "
            f"raw_pos_rad: {raw_pos:.6f} | calibration not found"
        )
        return False

    joint_position_deg = math.degrees(joint_position_rad)
    print(
        f"{format_motor_id(motor_id)} | "
        f"raw_pos_rad: {raw_pos:.6f} | "
        f"joint_rad: {joint_position_rad:.6f} | "
        f"joint_deg: {joint_position_deg:.3f} | "
        f"vel_rad_s: {velocity_rad_s:.6f} | "
        f"effort: {effort:.6f} | "
        f"status: {status_byte6:02X} {status_byte7:02X}"
    )
    return True


def read_motor_once(bus: can.BusABC, motor_id: int, packet: bytes, calibration: dict) -> bool:
    drained = drain_rx_queue(bus)
    if drained:
        print(f"{format_motor_id(motor_id)} | drained {drained} stale RX frame(s)")

    msg = can.Message(arbitration_id=motor_id, data=packet, is_extended_id=False)
    bus.send(msg)

    raw_bytes = receive_feedback(bus, motor_id, packet)
    if raw_bytes is None:
        print(f"{format_motor_id(motor_id)} | NO FEEDBACK")
        return False

    return print_joint_state(raw_bytes, motor_id, calibration)


def run_once(channel: str, motor_ids: list[int]) -> int:
    packet = pack_mit_command(0.0, 0.0, 0.0, 0.0, 0.0)
    if packet != EXPECTED_ZERO_TORQUE_PACKET:
        print(
            "Refusing to transmit: zero-torque packet mismatch "
            f"actual={format_packet(packet)} expected={format_packet(EXPECTED_ZERO_TORQUE_PACKET)}"
        )
        return 1

    if not confirm_before_opening_bus(channel, motor_ids):
        print("Aborted. No CAN bus opened and no command transmitted.")
        return 0

    calibration = load_motor_calibration()
    results: list[bool] = []

    bus = None
    try:
        bus = can.Bus(interface=DEFAULT_INTERFACE, channel=channel)
        for motor_id in motor_ids:
            results.append(read_motor_once(bus, motor_id, packet, calibration))
    finally:
        if bus is not None:
            bus.shutdown()

    if all(results):
        print("Overall result: all requested motors read successfully")
        return 0
    else:
        print("Overall result: one or more motors were not read successfully")
        return 2


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Send one zero-torque command per AK70-10 motor and read calibrated joint states."
    )
    parser.add_argument("--channel", default=DEFAULT_CHANNEL)
    parser.add_argument(
        "--motor-ids",
        type=parse_motor_ids,
        default=list(DEFAULT_MOTOR_IDS),
        help="Comma-separated motor IDs, for example: 0x005,0x007,0x00A",
    )
    args = parser.parse_args()

    raise SystemExit(run_once(channel=args.channel, motor_ids=args.motor_ids))


if __name__ == "__main__":
    main()
