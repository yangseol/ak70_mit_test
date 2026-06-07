from __future__ import annotations

import argparse
import time

import can

from mit_packet import analyze_feedback_candidate, pack_mit_command


DEFAULT_CHANNEL = "can0"
DEFAULT_INTERFACE = "socketcan"
DEFAULT_MOTOR_ID = 0x00A
ZERO_TORQUE_EXPECTED_PACKET = bytes([0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x00])
RECEIVE_TIMEOUT_SEC = 1.0


def format_data_bytes(data: bytes) -> str:
    return ", ".join(f"{byte:02X}" for byte in data)


def build_zero_torque_packet() -> bytes:
    return pack_mit_command(p=0.0, v=0.0, kp=0.0, kd=0.0, t=0.0)


def print_preflight(channel: str, motor_id: int, packet: bytes) -> None:
    print(f"channel: {channel}")
    print(f"motor_id: 0x{motor_id:03X}")
    print(f"packet hex: {packet.hex(' ').upper()}")
    print("This will send exactly ONE zero-torque MIT command.")


def print_feedback(raw: bytes, parsed: dict) -> None:
    print("[Rx Real Feedback Detected]")
    print(f"- Raw Hex: {raw.hex().upper()}")
    print(
        "- Parsed: "
        f"ID=0x{parsed['motor_id']:02X}, "
        f"p_uint=0x{parsed['p_uint']:04X}, "
        f"pos_rad={parsed['candidate_position_rad']:.6f}, "
        f"v_uint=0x{parsed['v_uint']:03X}, "
        f"vel_rad_s={parsed['candidate_velocity_rads']:.6f}, "
        f"effort={parsed['candidate_effort_from_torque_limit']:.6f}, "
        f"status={parsed['status_byte6']:02X} {parsed['status_byte7']:02X}"
    )


def receive_feedback_candidate(bus: can.BusABC, motor_id: int, packet: bytes, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec

    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break

        msg = bus.recv(timeout=remaining)
        if msg is None:
            continue
        if msg.arbitration_id != motor_id:
            continue
        if len(msg.data) != 8:
            continue

        raw = bytes(msg.data)
        if raw == packet:
            print("[Rx SKIP] Local echo of transmitted zero-torque command")
            continue
        if raw[0] != (motor_id & 0xFF):
            print(f"[Rx SKIP] Unexpected data for motor_id=0x{motor_id:03X}: {raw.hex().upper()}")
            continue

        parsed = analyze_feedback_candidate(raw)
        if parsed is None:
            continue

        print_feedback(raw, parsed)
        return True

    return False


def run_once(channel: str = DEFAULT_CHANNEL, motor_id: int = DEFAULT_MOTOR_ID) -> int:
    packet = build_zero_torque_packet()
    if packet != ZERO_TORQUE_EXPECTED_PACKET:
        print(
            "ERROR: zero-torque packet mismatch; refusing to transmit. "
            f"actual={packet.hex(' ').upper()} "
            f"expected={ZERO_TORQUE_EXPECTED_PACKET.hex(' ').upper()}"
        )
        return 1

    print_preflight(channel, motor_id, packet)
    confirmation = input("Type YES to transmit: ")
    if confirmation != "YES":
        print("Aborted. No CAN bus opened and no command transmitted.")
        return 0

    bus = None
    try:
        bus = can.Bus(interface=DEFAULT_INTERFACE, channel=channel)
        msg = can.Message(arbitration_id=motor_id, data=packet, is_extended_id=False)
        bus.send(msg)
        print(f"[Tx] ID: 0x{motor_id:03X} | Data: [{format_data_bytes(packet)}]")

        found = receive_feedback_candidate(bus, motor_id, packet, RECEIVE_TIMEOUT_SEC)
        if not found:
            print("No real feedback received within timeout")
        return 0
    except can.CanError as exc:
        print(f"[CanError] {exc}")
        return 1
    finally:
        if bus is not None:
            bus.shutdown()


def _parse_motor_id(value: str) -> int:
    return int(value, 0)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Send exactly one zero-torque MIT command and observe one AK70-10 response candidate."
    )
    parser.add_argument("--channel", default=DEFAULT_CHANNEL, help="SocketCAN channel name.")
    parser.add_argument("--motor-id", type=_parse_motor_id, default=DEFAULT_MOTOR_ID, help="Motor arbitration ID.")
    args = parser.parse_args()

    raise SystemExit(run_once(channel=args.channel, motor_id=args.motor_id))


if __name__ == "__main__":
    main()
