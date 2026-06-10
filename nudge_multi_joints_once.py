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
DEFAULT_KP = 2.0
DEFAULT_KD = 0.1
DEFAULT_PULSES = 5
DEFAULT_INTERVAL_SEC = 0.02
RECEIVE_DEADLINE_SEC = 1.0
RECV_POLL_TIMEOUT_SEC = 0.05
DRAIN_DURATION_SEC = 0.20
DRAIN_POLL_TIMEOUT_SEC = 0.01
ENTER_SLEEP_SEC = 0.05
FINAL_SETTLING_SEC = 0.10
EXPECTED_ZERO_TORQUE_PACKET = bytes([0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x00])
MIT_ENTER_PACKET = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])

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
    return ", ".join(format_motor_id(motor_id) for motor_id in motor_ids)


def format_packet(packet: bytes) -> str:
    return packet.hex(" ").upper()


def validate_cli_values(
    tolerance_rad: float,
    max_start_error_rad: float,
    kp: float,
    kd: float,
    pulses: int,
    interval_sec: float,
) -> bool:
    errors: list[str] = []
    if not 0.0 < kp <= 5.0:
        errors.append("Invalid kp: must be > 0.0 and <= 5.0")
    if not 0.0 <= kd <= 2.0:
        errors.append("Invalid kd: must be >= 0.0 and <= 2.0")
    if not 1 <= pulses <= 20:
        errors.append("Invalid pulses: must be >= 1 and <= 20")
    if interval_sec < 0.005:
        errors.append("Invalid interval_sec: must be >= 0.005")
    if tolerance_rad <= 0.0:
        errors.append("Invalid tolerance_rad: must be > 0.0")
    if max_start_error_rad <= tolerance_rad:
        errors.append("Invalid max_start_error_rad: must be > tolerance_rad")

    for error in errors:
        print(error)
    return not errors


def get_raw_zero_pos_rad(motor_id: int, calibration: dict) -> float:
    motor_key = format_motor_id(motor_id)
    try:
        return float(calibration["motors"][motor_key]["raw_zero_pos_rad"])
    except KeyError as exc:
        raise KeyError(f"No raw_zero_pos_rad calibration found for motor_id={motor_key}") from exc


def confirm_before_opening_bus(
    channel: str,
    motor_ids: list[int],
    tolerance_rad: float,
    max_start_error_rad: float,
    kp: float,
    kd: float,
    pulses: int,
    interval_sec: float,
) -> bool:
    print("SAFETY WARNING")
    print("- sequential limited nudge only")
    print("- no 0xFE")
    print("- no calibration write")
    print("- no simultaneous motion")
    print("- no continuous loop")
    print(f"channel: {channel}")
    print(f"motor_ids: {format_motor_ids(motor_ids)}")
    print(f"tolerance_rad: {tolerance_rad:.6f}")
    print(f"max_start_error_rad: {max_start_error_rad:.6f}")
    print(f"kp: {kp}")
    print(f"kd: {kd}")
    print(f"pulses: {pulses}")
    print(f"interval_sec: {interval_sec}")
    print("This will first send one zero-torque read command per motor.")
    confirmation = input("Type YES to continue: ")
    return confirmation == "YES"


def confirm_before_nudge(target_motor_ids: list[int]) -> bool:
    print("Position commands are still blocked until this final confirmation.")
    print(f"Ready nudge targets: {format_motor_ids(target_motor_ids)}")
    confirmation = input("Type NUDGE to start sequential limited nudge: ")
    return confirmation == "NUDGE"


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


def receive_feedback(bus: can.BusABC, motor_id: int, echo_packet: bytes) -> bytes | None:
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

        if raw_bytes == echo_packet:
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


def read_motor_plan(
    bus: can.BusABC,
    motor_id: int,
    zero_torque_packet: bytes,
    calibration: dict,
    tolerance_rad: float,
    max_start_error_rad: float,
) -> dict:
    drained = drain_rx_queue(bus)
    if drained:
        print(f"{format_motor_id(motor_id)} | drained {drained} stale RX frame(s)")

    msg = can.Message(arbitration_id=motor_id, data=zero_torque_packet, is_extended_id=False)
    bus.send(msg)

    raw_bytes = receive_feedback(bus, motor_id, zero_torque_packet)
    if raw_bytes is None:
        return {"motor_id": motor_id, "action": NO_FEEDBACK}

    candidate = analyze_feedback_candidate(raw_bytes)
    if candidate is None:
        return {"motor_id": motor_id, "action": NO_FEEDBACK}

    raw_pos = float(candidate["candidate_position_rad"])
    try:
        joint_rad = apply_software_offset(raw_pos, motor_id, calibration)
        raw_zero_pos_rad = get_raw_zero_pos_rad(motor_id, calibration)
    except KeyError:
        return {
            "motor_id": motor_id,
            "raw_pos_rad": raw_pos,
            "action": NO_CALIBRATION,
        }

    return {
        "motor_id": motor_id,
        "raw_pos_rad": raw_pos,
        "raw_zero_pos_rad": raw_zero_pos_rad,
        "joint_rad": joint_rad,
        "action": choose_action(joint_rad, tolerance_rad, max_start_error_rad),
    }


def read_all_plans(
    bus: can.BusABC,
    motor_ids: list[int],
    zero_torque_packet: bytes,
    calibration: dict,
    tolerance_rad: float,
    max_start_error_rad: float,
) -> list[dict]:
    plans: list[dict] = []
    for motor_id in motor_ids:
        plans.append(
            read_motor_plan(
                bus=bus,
                motor_id=motor_id,
                zero_torque_packet=zero_torque_packet,
                calibration=calibration,
                tolerance_rad=tolerance_rad,
                max_start_error_rad=max_start_error_rad,
            )
        )
    return plans


def print_plan(title: str, plans: list[dict]) -> None:
    print(title)
    for plan in plans:
        motor_id = plan["motor_id"]
        action = plan["action"]
        if "joint_rad" in plan:
            joint_rad = plan["joint_rad"]
            joint_deg = math.degrees(joint_rad)
            print(
                f"ID: {format_motor_id(motor_id)} | "
                f"joint_rad: {joint_rad:.6f} | "
                f"joint_deg: {joint_deg:.2f} | "
                f"action: {action}"
            )
        elif "raw_pos_rad" in plan:
            print(
                f"ID: {format_motor_id(motor_id)} | "
                f"raw_pos_rad: {plan['raw_pos_rad']:.6f} | "
                f"action: {action}"
            )
        else:
            print(f"ID: {format_motor_id(motor_id)} | action: {action}")


def has_interlock_failure(plans: list[dict]) -> bool:
    blocked_actions = {BLOCKED_TOO_FAR, NO_FEEDBACK, NO_CALIBRATION}
    return any(plan["action"] in blocked_actions for plan in plans)


def get_ready_plans(plans: list[dict]) -> list[dict]:
    return [plan for plan in plans if plan["action"] == READY_FOR_LIMITED_NUDGE]


def send_limited_nudge(
    bus: can.BusABC,
    plan: dict,
    kp: float,
    kd: float,
    pulses: int,
    interval_sec: float,
) -> None:
    motor_id = plan["motor_id"]
    target_joint_rad = 0.0
    target_raw_pos_rad = plan["raw_zero_pos_rad"]

    print(
        f"[Nudge] ID: {format_motor_id(motor_id)} | "
        f"target_joint_rad: {target_joint_rad:.6f} | "
        f"target_raw_pos_rad: {target_raw_pos_rad:.6f} | "
        f"kp: {kp} | kd: {kd} | pulses: {pulses}"
    )

    enter_msg = can.Message(arbitration_id=motor_id, data=MIT_ENTER_PACKET, is_extended_id=False)
    bus.send(enter_msg)
    time.sleep(ENTER_SLEEP_SEC)

    position_packet = pack_mit_command(target_raw_pos_rad, 0.0, kp, kd, 0.0)
    position_msg = can.Message(arbitration_id=motor_id, data=position_packet, is_extended_id=False)

    for pulse_index in range(1, pulses + 1):
        bus.send(position_msg)
        print(
            f"[Pulse {pulse_index}/{pulses}] ID: {format_motor_id(motor_id)} | "
            f"Data: {format_packet(position_packet)}"
        )
        time.sleep(interval_sec)

    drain_rx_queue(bus)


def print_nudge_targets(ready_plans: list[dict]) -> None:
    print("[Nudge Targets]")
    print(format_motor_ids([plan["motor_id"] for plan in ready_plans]))


def print_summary(initial_plans: list[dict], ready_plans: list[dict], final_plans: list[dict]) -> None:
    initial_actions = [plan["action"] for plan in initial_plans]
    final_actions = [plan["action"] for plan in final_plans]

    print("[Summary]")
    print(f"total motors: {len(initial_plans)}")
    print(f"skipped count: {initial_actions.count(SKIP_AT_ZERO)}")
    print(f"nudged count: {len(ready_plans)}")
    print(f"blocked count: {initial_actions.count(BLOCKED_TOO_FAR)}")
    print(f"no feedback count: {initial_actions.count(NO_FEEDBACK)}")
    print(f"no calibration count: {initial_actions.count(NO_CALIBRATION)}")
    print(f"final {SKIP_AT_ZERO} count: {final_actions.count(SKIP_AT_ZERO)}")
    print(f"final {READY_FOR_LIMITED_NUDGE} count: {final_actions.count(READY_FOR_LIMITED_NUDGE)}")
    print(f"final {BLOCKED_TOO_FAR} count: {final_actions.count(BLOCKED_TOO_FAR)}")


def run_once(
    channel: str,
    motor_ids: list[int],
    tolerance_rad: float,
    max_start_error_rad: float,
    kp: float,
    kd: float,
    pulses: int,
    interval_sec: float,
) -> int:
    if not validate_cli_values(tolerance_rad, max_start_error_rad, kp, kd, pulses, interval_sec):
        return 2

    zero_torque_packet = pack_mit_command(0.0, 0.0, 0.0, 0.0, 0.0)
    if zero_torque_packet != EXPECTED_ZERO_TORQUE_PACKET:
        print(
            "Refusing to transmit: zero-torque packet mismatch "
            f"actual={format_packet(zero_torque_packet)} "
            f"expected={format_packet(EXPECTED_ZERO_TORQUE_PACKET)}"
        )
        return 2

    if not confirm_before_opening_bus(
        channel=channel,
        motor_ids=motor_ids,
        tolerance_rad=tolerance_rad,
        max_start_error_rad=max_start_error_rad,
        kp=kp,
        kd=kd,
        pulses=pulses,
        interval_sec=interval_sec,
    ):
        print("Aborted. No CAN bus opened and no command transmitted.")
        return 0

    calibration = load_motor_calibration()

    bus = None
    try:
        bus = can.Bus(interface=DEFAULT_INTERFACE, channel=channel)
        initial_plans = read_all_plans(
            bus=bus,
            motor_ids=motor_ids,
            zero_torque_packet=zero_torque_packet,
            calibration=calibration,
            tolerance_rad=tolerance_rad,
            max_start_error_rad=max_start_error_rad,
        )
        print_plan("[Initial Plan]", initial_plans)

        if has_interlock_failure(initial_plans):
            print("INTERLOCK BLOCKED: No position commands were sent.")
            print_summary(initial_plans, [], initial_plans)
            return 2

        ready_plans = get_ready_plans(initial_plans)
        if not ready_plans:
            print("All motors are already near software zero. No nudge needed.")
            print_summary(initial_plans, [], initial_plans)
            return 0

        print_nudge_targets(ready_plans)
        if not confirm_before_nudge([plan["motor_id"] for plan in ready_plans]):
            print("Aborted before nudge. No position commands were sent.")
            print_summary(initial_plans, [], initial_plans)
            return 0

        # Defaults are practical for bench testing a loose motor. Reduce gains
        # before using this helper on an assembled robot.
        for plan in ready_plans:
            send_limited_nudge(
                bus=bus,
                plan=plan,
                kp=kp,
                kd=kd,
                pulses=pulses,
                interval_sec=interval_sec,
            )

        time.sleep(FINAL_SETTLING_SEC)
        drain_rx_queue(bus)
        final_plans = read_all_plans(
            bus=bus,
            motor_ids=motor_ids,
            zero_torque_packet=zero_torque_packet,
            calibration=calibration,
            tolerance_rad=tolerance_rad,
            max_start_error_rad=max_start_error_rad,
        )
        print_plan("[Final Plan]", final_plans)
        print_summary(initial_plans, ready_plans, final_plans)

        final_actions = [plan["action"] for plan in final_plans]
        if NO_FEEDBACK in final_actions or NO_CALIBRATION in final_actions:
            return 2
        return 0
    finally:
        if bus is not None:
            bus.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Sequentially nudge only READY AK70-10 joints toward software zero once."
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
    parser.add_argument("--kp", type=float, default=DEFAULT_KP)
    parser.add_argument("--kd", type=float, default=DEFAULT_KD)
    parser.add_argument("--pulses", type=int, default=DEFAULT_PULSES)
    parser.add_argument("--interval-sec", type=float, default=DEFAULT_INTERVAL_SEC)
    args = parser.parse_args()

    raise SystemExit(
        run_once(
            channel=args.channel,
            motor_ids=args.motor_ids,
            tolerance_rad=args.tolerance_rad,
            max_start_error_rad=args.max_start_error_rad,
            kp=args.kp,
            kd=args.kd,
            pulses=args.pulses,
            interval_sec=args.interval_sec,
        )
    )


if __name__ == "__main__":
    main()
