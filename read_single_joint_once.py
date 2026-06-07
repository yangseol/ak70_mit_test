from __future__ import annotations

import argparse
import time

import can

from calibration import apply_software_offset, load_motor_calibration
from mit_packet import analyze_feedback_candidate, pack_mit_command


DEFAULT_CHANNEL = "can0"
DEFAULT_INTERFACE = "socketcan"
DEFAULT_MOTOR_ID = 0x00A
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


def format_packet(packet: bytes) -> str:
    return packet.hex(" ").upper()


def confirm_before_opening_bus(channel: str, motor_id: int) -> bool:
    print(f"channel: {channel}")
    print(f"motor_id: {format_motor_id(motor_id)}")
    print("This will send ONE zero-torque command to read the calibrated joint angle.")
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


def print_joint_state(raw_bytes: bytes, motor_id: int, calibration: dict) -> None:
    candidate = analyze_feedback_candidate(raw_bytes)
    if candidate is None:
        print("No feedback received")
        return

    raw_pos = candidate["candidate_position_rad"]
    joint_position_rad = apply_software_offset(raw_pos, motor_id, calibration)
    velocity_rad_s = candidate["candidate_velocity_rads"]
    effort = candidate["candidate_effort_from_torque_limit"]
    status_byte6 = candidate["status_byte6"]
    status_byte7 = candidate["status_byte7"]

    print("[Joint State Read Success]")
    print(
        f"raw_pos_rad: {raw_pos:.6f} | "
        f"joint_rad: {joint_position_rad:.6f} | "
        f"vel_rad_s: {velocity_rad_s:.6f} | "
        f"effort: {effort:.6f} | "
        f"status: {status_byte6:02X} {status_byte7:02X}"
    )


def run_once(channel: str, motor_id: int) -> int:
    packet = pack_mit_command(0.0, 0.0, 0.0, 0.0, 0.0)
    if packet != EXPECTED_ZERO_TORQUE_PACKET:
        print(
            "Refusing to transmit: zero-torque packet mismatch "
            f"actual={format_packet(packet)} expected={format_packet(EXPECTED_ZERO_TORQUE_PACKET)}"
        )
        return 1

    if not confirm_before_opening_bus(channel, motor_id):
        print("Aborted. No CAN bus opened and no command transmitted.")
        return 0

    calibration = load_motor_calibration()

    bus = None
    try:
        bus = can.Bus(interface=DEFAULT_INTERFACE, channel=channel)
        msg = can.Message(arbitration_id=motor_id, data=packet, is_extended_id=False)
        bus.send(msg)

        raw_bytes = receive_feedback(bus, motor_id, packet)
        if raw_bytes is None:
            print("No feedback received")
            return 0

        print_joint_state(raw_bytes, motor_id, calibration)
        return 0
    finally:
        if bus is not None:
            bus.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Send one zero-torque command and read one calibrated AK70-10 joint angle."
    )
    parser.add_argument("--channel", default=DEFAULT_CHANNEL)
    parser.add_argument("--motor-id", type=parse_motor_id, default=DEFAULT_MOTOR_ID)
    args = parser.parse_args()

    raise SystemExit(run_once(channel=args.channel, motor_id=args.motor_id))


if __name__ == "__main__":
    main()
