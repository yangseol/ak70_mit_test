"""AK70 모터를 움직이지 않고 software zero 기준 원점 이동 계획만 판정하는 helper."""

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
DEFAULT_TOLERANCE_RAD = 0.05
DEFAULT_MAX_START_ERROR_RAD = 0.30
RECEIVE_DEADLINE_SEC = 1.0
RECV_POLL_TIMEOUT_SEC = 0.05
DRAIN_DURATION_SEC = 0.20
DRAIN_POLL_TIMEOUT_SEC = 0.01
EXPECTED_ZERO_TORQUE_PACKET = bytes([0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x00])

SKIP_AT_ZERO = "SKIP_AT_ZERO"
READY_FOR_LIMITED_NUDGE = "READY_FOR_LIMITED_NUDGE"
BLOCKED_TOO_FAR = "BLOCKED_TOO_FAR"
NO_FEEDBACK = "NO_FEEDBACK"
NO_CALIBRATION = "NO_CALIBRATION"


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


def confirm_before_opening_bus(
    channel: str,
    motor_ids: list[int],
    tolerance_rad: float,
    max_start_error_rad: float,
) -> bool:
    print("SAFETY WARNING")
    print("- read-only zero-torque planning tool")
    print("- no 0xFE")
    print("- no position control")
    print("- no nudge")
    print("- no motion command")
    print(f"channel: {channel}")
    print(f"motor_ids: {format_motor_ids(motor_ids)}")
    print(f"tolerance_rad: {tolerance_rad:.6f}")
    print(f"max_start_error_rad: {max_start_error_rad:.6f}")
    print("This will send ONE zero-torque MIT command to each listed motor for planning only.")
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


def choose_action(joint_rad: float, tolerance_rad: float, max_start_error_rad: float) -> str:
    abs_joint_rad = abs(joint_rad)
    if abs_joint_rad <= tolerance_rad:
        return SKIP_AT_ZERO
    if abs_joint_rad <= max_start_error_rad:
        return READY_FOR_LIMITED_NUDGE
    return BLOCKED_TOO_FAR


def print_no_feedback(motor_id: int) -> str:
    print(f"[Move Plan] ID: {format_motor_id(motor_id)} | action: {NO_FEEDBACK}")
    return NO_FEEDBACK


def print_no_calibration(motor_id: int, raw_pos: float) -> str:
    print(
        f"[Move Plan] ID: {format_motor_id(motor_id)} | "
        f"raw_pos_rad: {raw_pos:.6f} | action: {NO_CALIBRATION}"
    )
    return NO_CALIBRATION


def print_move_plan(
    motor_id: int,
    joint_rad: float,
    tolerance_rad: float,
    max_start_error_rad: float,
) -> str:
    joint_deg = math.degrees(joint_rad)
    target_rad = 0.0
    error_to_zero_rad = target_rad - joint_rad
    action = choose_action(joint_rad, tolerance_rad, max_start_error_rad)

    print(
        f"[Move Plan] ID: {format_motor_id(motor_id)} | "
        f"joint_rad: {joint_rad:.6f} | "
        f"joint_deg: {joint_deg:.2f} | "
        f"target_rad: {target_rad:.6f} | "
        f"error_to_zero_rad: {error_to_zero_rad:+.6f} | "
        f"action: {action}"
    )
    return action


def read_and_plan_motor(
    bus: can.BusABC,
    motor_id: int,
    packet: bytes,
    calibration: dict,
    tolerance_rad: float,
    max_start_error_rad: float,
) -> str:
    drained = drain_rx_queue(bus)
    if drained:
        print(f"{format_motor_id(motor_id)} | drained {drained} stale RX frame(s)")

    msg = can.Message(arbitration_id=motor_id, data=packet, is_extended_id=False)
    bus.send(msg)

    raw_bytes = receive_feedback(bus, motor_id, packet)
    if raw_bytes is None:
        return print_no_feedback(motor_id)

    candidate = analyze_feedback_candidate(raw_bytes)
    if candidate is None:
        return print_no_feedback(motor_id)

    raw_pos = candidate["candidate_position_rad"]
    try:
        joint_rad = apply_software_offset(raw_pos, motor_id, calibration)
    except KeyError:
        return print_no_calibration(motor_id, raw_pos)

    return print_move_plan(motor_id, joint_rad, tolerance_rad, max_start_error_rad)


def print_summary(actions: list[str]) -> None:
    total_motors = len(actions)
    no_feedback_count = actions.count(NO_FEEDBACK)
    no_calibration_count = actions.count(NO_CALIBRATION)
    skip_count = actions.count(SKIP_AT_ZERO)
    ready_count = actions.count(READY_FOR_LIMITED_NUDGE)
    blocked_count = actions.count(BLOCKED_TOO_FAR)
    readable_motors = total_motors - no_feedback_count - no_calibration_count

    print("[Summary]")
    print(f"total motors: {total_motors}")
    print(f"readable motors: {readable_motors}")
    print(f"{SKIP_AT_ZERO} count: {skip_count}")
    print(f"{READY_FOR_LIMITED_NUDGE} count: {ready_count}")
    print(f"{BLOCKED_TOO_FAR} count: {blocked_count}")
    print(f"{NO_FEEDBACK} count: {no_feedback_count}")
    print(f"{NO_CALIBRATION} count: {no_calibration_count}")

    if blocked_count:
        print(
            "CRITICAL NOTICE: Movement blocked. One or more motors are too far from "
            "software zero. Do NOT execute automated homing."
        )
    elif no_feedback_count == 0 and no_calibration_count == 0:
        # This is not movement authorization; it only marks candidates for a
        # separate later limited nudge test.
        print("Overall status: Safe for subsequent limited nudge tests.")


def run_once(
    channel: str,
    motor_ids: list[int],
    tolerance_rad: float,
    max_start_error_rad: float,
) -> int:
    packet = pack_mit_command(0.0, 0.0, 0.0, 0.0, 0.0)
    if packet != EXPECTED_ZERO_TORQUE_PACKET:
        print(
            "Refusing to transmit: zero-torque packet mismatch "
            f"actual={format_packet(packet)} expected={format_packet(EXPECTED_ZERO_TORQUE_PACKET)}"
        )
        return 1

    if not confirm_before_opening_bus(channel, motor_ids, tolerance_rad, max_start_error_rad):
        print("Aborted. No CAN bus opened and no command transmitted.")
        return 0

    calibration = load_motor_calibration()
    actions: list[str] = []

    bus = None
    try:
        bus = can.Bus(interface=DEFAULT_INTERFACE, channel=channel)
        for motor_id in motor_ids:
            action = read_and_plan_motor(
                bus=bus,
                motor_id=motor_id,
                packet=packet,
                calibration=calibration,
                tolerance_rad=tolerance_rad,
                max_start_error_rad=max_start_error_rad,
            )
            actions.append(action)
    finally:
        if bus is not None:
            bus.shutdown()

    print_summary(actions)

    if (
        BLOCKED_TOO_FAR in actions
        or NO_FEEDBACK in actions
        or NO_CALIBRATION in actions
    ):
        return 2
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Read AK70-10 software-zero offsets and print a no-motion zero plan."
    )
    parser.add_argument("--channel", default=DEFAULT_CHANNEL)
    parser.add_argument(
        "--motor-ids",
        type=parse_motor_ids,
        default=list(DEFAULT_MOTOR_IDS),
        help="Comma-separated motor IDs, for example: 0x005,0x007,0x00A",
    )
    parser.add_argument("--tolerance-rad", type=float, default=DEFAULT_TOLERANCE_RAD)
    parser.add_argument("--max-start-error-rad", type=float, default=DEFAULT_MAX_START_ERROR_RAD)
    args = parser.parse_args()

    raise SystemExit(
        run_once(
            channel=args.channel,
            motor_ids=args.motor_ids,
            tolerance_rad=args.tolerance_rad,
            max_start_error_rad=args.max_start_error_rad,
        )
    )


if __name__ == "__main__":
    main()
