from __future__ import annotations

import argparse
import time

import can

from mit_packet import pack_mit_command


DEFAULT_CHANNEL = "can0"
DEFAULT_INTERFACE = "socketcan"
DEFAULT_START_ID = 0x001
DEFAULT_END_ID = 0x00A
CANDIDATE_DEADLINE_SEC = 0.03
RECV_POLL_TIMEOUT_SEC = 0.005
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


def validate_zero_torque_packet(packet: bytes) -> bool:
    if packet == EXPECTED_ZERO_TORQUE_PACKET:
        return True

    print(
        "Refusing to transmit: zero-torque packet mismatch "
        f"actual={format_packet(packet)} expected={format_packet(EXPECTED_ZERO_TORQUE_PACKET)}"
    )
    return False


def has_valid_response(bus: can.BusABC, candidate_id: int, packet: bytes) -> bool:
    deadline = time.monotonic() + CANDIDATE_DEADLINE_SEC

    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            break

        msg = bus.recv(timeout=min(RECV_POLL_TIMEOUT_SEC, remaining))
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


def detect_motor_id(channel: str, interface: str, start_id: int, end_id: int) -> int | None:
    packet = pack_mit_command(0.0, 0.0, 0.0, 0.0, 0.0)
    if not validate_zero_torque_packet(packet):
        return None

    bus = None
    try:
        bus = can.Bus(interface=interface, channel=channel)
        for candidate_id in range(start_id, end_id + 1):
            msg = can.Message(arbitration_id=candidate_id, data=packet, is_extended_id=False)
            bus.send(msg)

            if has_valid_response(bus, candidate_id, packet):
                return candidate_id
    finally:
        if bus is not None:
            bus.shutdown()

    return None


def run_once(channel: str, interface: str, start_id: int, end_id: int) -> int:
    if start_id > end_id:
        print("Invalid ID range: start-id must be less than or equal to end-id")
        return 1

    detected_id = detect_motor_id(channel, interface, start_id, end_id)
    if detected_id is None:
        print(f"No motor detected in range {format_motor_id(start_id)}~{format_motor_id(end_id)}")
        return 0

    print(f"Detected motor ID: {format_motor_id(detected_id)}")
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Detect one connected AK70-10 motor by sending one zero-torque knock per candidate ID."
    )
    parser.add_argument("--channel", default=DEFAULT_CHANNEL)
    parser.add_argument("--interface", default=DEFAULT_INTERFACE)
    parser.add_argument("--start-id", type=parse_motor_id, default=DEFAULT_START_ID)
    parser.add_argument("--end-id", type=parse_motor_id, default=DEFAULT_END_ID)
    args = parser.parse_args()

    raise SystemExit(run_once(args.channel, args.interface, args.start_id, args.end_id))


if __name__ == "__main__":
    main()
