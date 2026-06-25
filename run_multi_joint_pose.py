"""Run a finite synchronized AK70-10 multi-joint pose move."""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass

import can

from calibration import load_motor_calibration
from mit_packet import AK70_10_LIMIT, analyze_feedback_candidate, pack_mit_command


DEFAULT_CHANNEL = "can0"
DEFAULT_INTERFACE = "socketcan"
DEFAULT_KP = 8.0
DEFAULT_KD = 0.4
DEFAULT_MOVE_SEC = 2.0
DEFAULT_HOLD_SEC = 2.0
DEFAULT_RATE_HZ = 50.0
DEFAULT_FEEDBACK_PRINT_HZ = 5.0
DEFAULT_MAX_FOLLOWING_ERROR_DEG = 60.0
HOME_MAX_AVERAGE_SPEED_DEG_S = 60.0
DRAIN_DURATION_SEC = 0.05
DRAIN_POLL_TIMEOUT_SEC = 0.005
PREFLIGHT_FEEDBACK_WINDOW_SEC = 1.0
RECV_POLL_TIMEOUT_SEC = 0.002
MIT_ENTER_SLEEP_SEC = 0.02
MIT_ENTER_BETWEEN_SEC = 0.02
RELEASE_SETTLE_SEC = 0.03

MAX_MOTORS = 10
MAX_SAFETY_TARGET_DEG = 120.0
MAX_SAFETY_DELTA_DEG = 120.0
MAX_CONSECUTIVE_FEEDBACK_MISSES = 10
MAX_CONSECUTIVE_LARGE_ERROR_CYCLES = 10

EXPECTED_ZERO_TORQUE_PACKET = bytes([0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x00])
MIT_ENTER_PACKET = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])


@dataclass(frozen=True)
class MotorCalibration:
    raw_zero_pos_rad: float
    direction_sign: int


@dataclass(frozen=True)
class TargetSpec:
    motor_id: int
    target_joint_rad: float
    target_joint_deg: float
    source_unit: str
    source_angle: float


@dataclass(frozen=True)
class GainSpec:
    motor_id: int
    kp: float
    kd: float


@dataclass
class MotorPlan:
    motor_id: int
    target_joint_rad: float
    target_joint_deg: float
    raw_zero_pos_rad: float
    direction_sign: int
    kp: float
    kd: float
    current_joint_rad: float = 0.0
    current_joint_deg: float = 0.0
    delta_joint_rad: float = 0.0
    delta_joint_deg: float = 0.0
    final_actual_joint_deg: float | None = None
    feedback_received_count: int = 0
    feedback_missed_count: int = 0
    consecutive_feedback_misses: int = 0
    maximum_consecutive_feedback_misses: int = 0
    consecutive_large_error_cycles: int = 0
    maximum_consecutive_large_error_cycles: int = 0
    maximum_abs_following_error_deg: float = 0.0


@dataclass
class PoseStats:
    synchronized_cycles_completed: int = 0
    position_frames_sent: int = 0
    feedback_frames_received: int = 0
    feedback_frames_missed: int = 0
    timing_overruns: int = 0


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


def parse_target(value: str, unit: str) -> TargetSpec:
    parts = value.split(",")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("target must use MOTOR_ID,ANGLE format")

    motor_text, angle_text = (part.strip() for part in parts)
    try:
        motor_id = parse_motor_id(motor_text)
        angle = float(angle_text)
    except (argparse.ArgumentTypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError(f"invalid target: {value!r}") from exc

    if unit == "deg":
        target_joint_deg = angle
        target_joint_rad = math.radians(angle)
    elif unit == "rad":
        target_joint_rad = angle
        target_joint_deg = math.degrees(angle)
    else:
        raise argparse.ArgumentTypeError(f"invalid target unit: {unit!r}")

    return TargetSpec(
        motor_id=motor_id,
        target_joint_rad=target_joint_rad,
        target_joint_deg=target_joint_deg,
        source_unit=unit,
        source_angle=angle,
    )


def parse_targets(values: list[str] | None, unit: str) -> list[TargetSpec]:
    if values is None:
        return []
    return [parse_target(value, unit) for value in values]


def parse_gain(value: str) -> GainSpec:
    parts = value.split(",")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("gain must use MOTOR_ID,KP,KD format")

    motor_text, kp_text, kd_text = (part.strip() for part in parts)
    try:
        motor_id = parse_motor_id(motor_text)
        kp = float(kp_text)
        kd = float(kd_text)
    except (argparse.ArgumentTypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError(f"invalid gain override: {value!r}") from exc

    return GainSpec(motor_id=motor_id, kp=kp, kd=kd)


def parse_gains(values: list[str] | None) -> list[GainSpec]:
    if values is None:
        return []
    return [parse_gain(value) for value in values]


def validate_ak70_motor_id(motor_id: int) -> bool:
    return 0x001 <= motor_id <= 0x00A


def validate_gain_values(kp: float, kd: float, label: str, errors: list[str]) -> None:
    if not math.isfinite(kp):
        errors.append(f"Invalid {label} kp: must be finite")
    if not math.isfinite(kd):
        errors.append(f"Invalid {label} kd: must be finite")
    if not 0.0 < kp <= 10.0:
        errors.append(f"Invalid {label} kp: must be > 0.0 and <= 10.0")
    if not 0.0 <= kd <= 2.0:
        errors.append(f"Invalid {label} kd: must be >= 0.0 and <= 2.0")


def validate_inputs(
    targets: list[TargetSpec],
    gains: list[GainSpec],
    common_kp: float,
    common_kd: float,
    move_sec: float,
    hold_sec: float,
    rate_hz: float,
    feedback_print_hz: float,
    max_following_error_deg: float,
    home_to_zero: bool,
) -> bool:
    errors: list[str] = []

    if not 1 <= len(targets) <= MAX_MOTORS:
        errors.append(f"Invalid target count: must be >= 1 and <= {MAX_MOTORS}")

    seen_targets: set[int] = set()
    for target in targets:
        if not validate_ak70_motor_id(target.motor_id):
            errors.append("Error: motor-id must be in AK70 range 0x001~0x00A")
        if target.motor_id in seen_targets:
            errors.append(f"Invalid target: duplicate motor ID {format_motor_id(target.motor_id)}")
        seen_targets.add(target.motor_id)
        if not math.isfinite(target.target_joint_rad) or not math.isfinite(target.target_joint_deg):
            errors.append(f"Invalid target for {format_motor_id(target.motor_id)}: angle must be finite")
        if home_to_zero:
            if target.source_unit == "deg" and abs(target.source_angle) > 1e-9:
                errors.append("Error: --home-to-zero requires every target to be exactly 0 degrees.")
            if target.source_unit == "rad" and abs(target.source_angle) > 1e-12:
                errors.append("Error: --home-to-zero requires every target to be exactly 0 degrees.")
        if abs(target.target_joint_deg) > MAX_SAFETY_TARGET_DEG:
            errors.append(
                f"Invalid target for {format_motor_id(target.motor_id)}: "
                f"absolute target must be <= {MAX_SAFETY_TARGET_DEG:.1f} deg"
            )

    validate_gain_values(common_kp, common_kd, "common", errors)

    seen_gains: set[int] = set()
    target_ids = {target.motor_id for target in targets}
    for gain in gains:
        if not validate_ak70_motor_id(gain.motor_id):
            errors.append("Error: motor-id must be in AK70 range 0x001~0x00A")
        if gain.motor_id not in target_ids:
            errors.append(f"Invalid gain override: {format_motor_id(gain.motor_id)} is not targeted")
        if gain.motor_id in seen_gains:
            errors.append(f"Invalid gain override: duplicate motor ID {format_motor_id(gain.motor_id)}")
        seen_gains.add(gain.motor_id)
        validate_gain_values(gain.kp, gain.kd, format_motor_id(gain.motor_id), errors)

    float_values = {
        "move_sec": move_sec,
        "hold_sec": hold_sec,
        "rate_hz": rate_hz,
        "feedback_print_hz": feedback_print_hz,
        "max_following_error_deg": max_following_error_deg,
    }
    for name, value in float_values.items():
        if not math.isfinite(value):
            errors.append(f"Invalid {name}: must be finite")

    if not 0.2 <= move_sec <= 15.0:
        errors.append("Invalid move-sec: must be >= 0.2 and <= 15.0")
    if not 0.0 <= hold_sec <= 30.0:
        errors.append("Invalid hold-sec: must be >= 0.0 and <= 30.0")
    if not 10.0 <= rate_hz <= 100.0:
        errors.append("Invalid rate-hz: must be >= 10.0 and <= 100.0")
    if not 0.5 <= feedback_print_hz <= rate_hz:
        errors.append("Invalid feedback-print-hz: must be >= 0.5 and <= rate-hz")
    if not 10.0 <= max_following_error_deg <= 120.0:
        errors.append("Invalid max-following-error-deg: must be >= 10.0 and <= 120.0")

    for error in errors:
        print(error)
    return not errors


def print_start_warning(
    channel: str,
    plans: list[MotorPlan],
    common_kp: float,
    common_kd: float,
    gain_overrides: dict[int, tuple[float, float]],
    move_sec: float,
    hold_sec: float,
    rate_hz: float,
    feedback_print_hz: float,
    max_following_error_deg: float,
    enter_mit: bool,
    home_to_zero: bool,
) -> None:
    print("SAFETY WARNING")
    print("- AK70 synchronized finite multi-joint pose move")
    print("- selected motors only")
    print("- periodic position and velocity commands")
    print("- finite move and hold duration")
    print("- no 0xFE")
    print("- no calibration write")
    print("- Ctrl+C releases all selected motors")
    print(f"channel: {channel}")
    print(f"selected motor count: {len(plans)}")
    print("motor IDs and targets:")
    for plan in plans:
        print(f"  {format_motor_id(plan.motor_id)} -> {plan.target_joint_deg:+.2f} deg")
    print(f"common kp: {common_kp}")
    print(f"common kd: {common_kd}")
    print("per-motor gain overrides:")
    if gain_overrides:
        for motor_id in sorted(gain_overrides):
            kp, kd = gain_overrides[motor_id]
            print(f"  {format_motor_id(motor_id)}: kp={kp}, kd={kd}")
    else:
        print("  none")
    print(f"move_sec: {move_sec:.3f}")
    print(f"hold_sec: {hold_sec:.3f}")
    print(f"rate_hz: {rate_hz:.1f}")
    print(f"feedback_print_hz: {feedback_print_hz:.1f}")
    print(f"max_following_error_deg: {max_following_error_deg:.1f}")
    print(f"enter_mit: {enter_mit}")
    print(f"home_to_zero: {home_to_zero}")


def confirm_yes() -> bool:
    return input("Type YES to start synchronized multi-joint pose move: ").strip() == "YES"


def validate_zero_torque_packet(packet: bytes) -> bool:
    if packet == EXPECTED_ZERO_TORQUE_PACKET:
        return True

    print(
        "Refusing to transmit: zero-torque packet mismatch "
        f"actual={format_packet(packet)} expected={format_packet(EXPECTED_ZERO_TORQUE_PACKET)}"
    )
    return False


def raw_to_joint_position(raw_pos_rad: float, raw_zero_pos_rad: float, direction_sign: int) -> float:
    return direction_sign * (raw_pos_rad - raw_zero_pos_rad)


def joint_to_raw_position(joint_pos_rad: float, raw_zero_pos_rad: float, direction_sign: int) -> float:
    return raw_zero_pos_rad + direction_sign * joint_pos_rad


def joint_to_raw_velocity(joint_vel_rad_s: float, direction_sign: int) -> float:
    return direction_sign * joint_vel_rad_s


def command_in_range(raw_pos_rad: float, raw_vel_rad_s: float) -> bool:
    return (
        AK70_10_LIMIT.p_min <= raw_pos_rad <= AK70_10_LIMIT.p_max
        and AK70_10_LIMIT.v_min <= raw_vel_rad_s <= AK70_10_LIMIT.v_max
    )


def load_calibrations(motor_ids: list[int]) -> dict[int, MotorCalibration] | None:
    try:
        calibration = load_motor_calibration()
    except Exception as exc:
        print(f"Error: failed to load motor_calibration.yaml: {exc}")
        return None

    calibrations: dict[int, MotorCalibration] = {}
    for motor_id in motor_ids:
        motor_key = format_motor_id(motor_id)
        try:
            motor_calibration = calibration["motors"][motor_key]
        except KeyError:
            print(f"Error: calibration not found for motor {motor_key}.")
            print("No position commands were sent.")
            return None

        try:
            raw_zero_pos_rad = float(motor_calibration["raw_zero_pos_rad"])
            direction_sign = int(motor_calibration["direction_sign"])
        except (KeyError, TypeError, ValueError) as exc:
            print(f"Error: invalid calibration for motor {motor_key}: {exc}")
            print("No position commands were sent.")
            return None

        if not math.isfinite(raw_zero_pos_rad):
            print(f"Error: raw_zero_pos_rad for motor {motor_key} must be finite.")
            print("No position commands were sent.")
            return None
        if direction_sign not in (-1, 1):
            print(f"Error: direction_sign for motor {motor_key} must be 1 or -1.")
            print("No position commands were sent.")
            return None

        calibrations[motor_id] = MotorCalibration(
            raw_zero_pos_rad=raw_zero_pos_rad,
            direction_sign=direction_sign,
        )

    return calibrations


def build_motor_plans(
    targets: list[TargetSpec],
    gains: list[GainSpec],
    common_kp: float,
    common_kd: float,
    calibrations: dict[int, MotorCalibration],
) -> tuple[list[MotorPlan], dict[int, tuple[float, float]]]:
    gain_overrides = {gain.motor_id: (gain.kp, gain.kd) for gain in gains}
    plans: list[MotorPlan] = []
    for target in sorted(targets, key=lambda item: item.motor_id):
        calibration = calibrations[target.motor_id]
        kp, kd = gain_overrides.get(target.motor_id, (common_kp, common_kd))
        plans.append(
            MotorPlan(
                motor_id=target.motor_id,
                target_joint_rad=target.target_joint_rad,
                target_joint_deg=target.target_joint_deg,
                raw_zero_pos_rad=calibration.raw_zero_pos_rad,
                direction_sign=calibration.direction_sign,
                kp=kp,
                kd=kd,
            )
        )
    return plans, gain_overrides


def drain_rx_queue(bus: can.BusABC, duration_sec: float = DRAIN_DURATION_SEC) -> int:
    deadline = time.monotonic() + max(0.0, duration_sec)
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


def collect_feedback_window(
    bus: can.BusABC,
    motor_ids: list[int],
    sent_packet_by_motor_id: dict[int, bytes],
    timeout_sec: float,
) -> dict[int, dict[str, float]]:
    selected = set(motor_ids)
    feedback_by_motor_id: dict[int, dict[str, float]] = {}
    deadline = time.monotonic() + max(0.0, timeout_sec)

    while time.monotonic() < deadline and len(feedback_by_motor_id) < len(selected):
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            break

        msg = bus.recv(timeout=min(RECV_POLL_TIMEOUT_SEC, remaining))
        if msg is None:
            continue
        if msg.arbitration_id not in selected:
            continue
        if len(msg.data) != 8:
            continue

        motor_id = msg.arbitration_id
        raw_bytes = bytes(msg.data)
        if raw_bytes == sent_packet_by_motor_id.get(motor_id):
            continue
        if raw_bytes == MIT_ENTER_PACKET:
            continue
        if raw_bytes[0] != (motor_id & 0xFF):
            continue

        candidate = analyze_feedback_candidate(raw_bytes)
        if candidate is None:
            continue

        feedback_by_motor_id[motor_id] = {
            "raw_pos_rad": float(candidate["candidate_position_rad"]),
            "velocity_rad_s": float(candidate["candidate_velocity_rads"]),
            "effort": float(candidate["candidate_effort_from_torque_limit"]),
        }

    return feedback_by_motor_id


def send_zero_torque_all_best_effort(
    bus: can.BusABC | None,
    motor_ids: list[int],
    zero_torque_packet: bytes,
) -> None:
    if bus is None:
        return

    for motor_id in sorted(motor_ids):
        try:
            msg = can.Message(arbitration_id=motor_id, data=zero_torque_packet, is_extended_id=False)
            bus.send(msg)
        except Exception as exc:
            print(f"{format_motor_id(motor_id)} zero-torque send failed: {exc}")


def release_all_selected(
    bus: can.BusABC,
    motor_ids: list[int],
    zero_torque_packet: bytes,
) -> None:
    send_zero_torque_all_best_effort(bus, motor_ids, zero_torque_packet)
    print("Zero-torque sent to all selected motors.")
    time.sleep(RELEASE_SETTLE_SEC)
    feedback = collect_feedback_window(
        bus=bus,
        motor_ids=motor_ids,
        sent_packet_by_motor_id={motor_id: zero_torque_packet for motor_id in motor_ids},
        timeout_sec=RELEASE_SETTLE_SEC,
    )
    if len(feedback) == len(motor_ids):
        print("All selected joints released.")
    else:
        missing = [format_motor_id(motor_id) for motor_id in motor_ids if motor_id not in feedback]
        print(f"Release feedback not received from: {', '.join(missing)}")


def enter_mit_for_selected(bus: can.BusABC, motor_ids: list[int]) -> None:
    for motor_id in sorted(motor_ids):
        msg = can.Message(arbitration_id=motor_id, data=MIT_ENTER_PACKET, is_extended_id=False)
        bus.send(msg)
        time.sleep(MIT_ENTER_BETWEEN_SEC)
    time.sleep(MIT_ENTER_SLEEP_SEC)


def read_preflight_positions(
    bus: can.BusABC,
    plans: list[MotorPlan],
    zero_torque_packet: bytes,
) -> bool:
    motor_ids = [plan.motor_id for plan in plans]
    drain_rx_queue(bus)
    for motor_id in motor_ids:
        msg = can.Message(arbitration_id=motor_id, data=zero_torque_packet, is_extended_id=False)
        bus.send(msg)

    feedback = collect_feedback_window(
        bus=bus,
        motor_ids=motor_ids,
        sent_packet_by_motor_id={motor_id: zero_torque_packet for motor_id in motor_ids},
        timeout_sec=PREFLIGHT_FEEDBACK_WINDOW_SEC,
    )

    by_id = {plan.motor_id: plan for plan in plans}
    for motor_id in motor_ids:
        if motor_id not in feedback:
            print(f"Pre-flight failed: no valid feedback from motor {format_motor_id(motor_id)}.")
            print("No pose commands were sent.")
            return False

        plan = by_id[motor_id]
        raw_pos_rad = feedback[motor_id]["raw_pos_rad"]
        plan.current_joint_rad = raw_to_joint_position(
            raw_pos_rad,
            plan.raw_zero_pos_rad,
            plan.direction_sign,
        )
        plan.current_joint_deg = math.degrees(plan.current_joint_rad)
        plan.delta_joint_rad = plan.target_joint_rad - plan.current_joint_rad
        plan.delta_joint_deg = plan.target_joint_deg - plan.current_joint_deg

    return True


def validate_pose_plan(plans: list[MotorPlan], home_to_zero: bool) -> bool:
    for plan in plans:
        if home_to_zero and abs(plan.target_joint_deg) > 1e-9:
            print("Error: --home-to-zero requires every target to be exactly 0 degrees.")
            print("No pose commands were sent.")
            return False
        if abs(plan.target_joint_deg) > MAX_SAFETY_TARGET_DEG:
            print("Pose blocked due to safety limit constraints.")
            print("No pose commands were sent.")
            return False
        if not home_to_zero and abs(plan.delta_joint_deg) > MAX_SAFETY_DELTA_DEG:
            print("Pose blocked due to safety limit constraints.")
            print("No pose commands were sent.")
            return False

        target_raw_pos_rad = plan.raw_zero_pos_rad if home_to_zero else joint_to_raw_position(
            plan.target_joint_rad,
            plan.raw_zero_pos_rad,
            plan.direction_sign,
        )
        if not command_in_range(target_raw_pos_rad, 0.0):
            print("Pose blocked due to safety limit constraints.")
            print("No pose commands were sent.")
            return False

    return True


def print_pose_plan(
    plans: list[MotorPlan],
    move_sec: float,
    hold_sec: float,
    rate_hz: float,
    move_cycles: int,
    hold_cycles: int,
    home_to_zero: bool,
    requested_move_sec: float,
    max_home_delta_deg: float,
) -> None:
    total_cycles = move_cycles + hold_cycles
    total_expected_frames = len(plans) * total_cycles

    print("[Multi-Joint Home-to-Zero Plan]" if home_to_zero else "[Multi-Joint Pose Plan]")
    print(f"Selected motors: {len(plans)}")
    print(f"move_sec: {move_sec:.3f}")
    if home_to_zero:
        print("Home mode: enabled")
        print(f"Requested move_sec: {requested_move_sec:.3f}")
        print(f"Effective move_sec: {move_sec:.3f}")
        print(f"Maximum home delta: {max_home_delta_deg:.2f} deg")
    print(f"hold_sec: {hold_sec:.3f}")
    print(f"rate_hz: {rate_hz:.1f}")
    print(f"move cycles: {move_cycles}")
    print(f"hold cycles: {hold_cycles}")
    print(f"total synchronized cycles: {total_cycles}")
    print(f"total expected position frames: {total_expected_frames}")
    print("")
    print("ID      Current      Target       Delta       Kp      Kd")
    for plan in plans:
        print(
            f"{format_motor_id(plan.motor_id):<7} "
            f"{plan.current_joint_deg:+8.2f} deg "
            f"{plan.target_joint_deg:+8.2f} deg "
            f"{plan.delta_joint_deg:+8.2f} deg "
            f"{plan.kp:<7.1f} {plan.kd:<7.1f}"
        )


def smoothstep_quintic(u: float) -> tuple[float, float]:
    s = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
    ds_du = 30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4
    return s, ds_du


def wait_until_send_time(next_send_time: float) -> None:
    remaining = next_send_time - time.monotonic()
    if remaining > 0.0:
        time.sleep(remaining)


def update_next_send_time(next_send_time: float, period_sec: float, stats: PoseStats) -> float:
    scheduled_next = next_send_time + period_sec
    now = time.monotonic()
    if now >= scheduled_next:
        stats.timing_overruns += 1
        return now + period_sec
    return scheduled_next


def compute_command_packets(
    plans: list[MotorPlan],
    command_joint_by_motor_id: dict[int, tuple[float, float]],
) -> dict[int, bytes] | None:
    packets: dict[int, bytes] = {}
    for plan in plans:
        command_joint_pos_rad, command_joint_vel_rad_s = command_joint_by_motor_id[plan.motor_id]
        command_raw_pos_rad = joint_to_raw_position(
            command_joint_pos_rad,
            plan.raw_zero_pos_rad,
            plan.direction_sign,
        )
        command_raw_vel_rad_s = joint_to_raw_velocity(command_joint_vel_rad_s, plan.direction_sign)

        if not command_in_range(command_raw_pos_rad, command_raw_vel_rad_s):
            print("Pose aborted: command exceeds AK70 position or velocity range.")
            print("Releasing all selected motors.")
            return None

        packets[plan.motor_id] = pack_mit_command(
            command_raw_pos_rad,
            command_raw_vel_rad_s,
            plan.kp,
            plan.kd,
            0.0,
        )
    return packets


def send_packets(bus: can.BusABC, packets: dict[int, bytes]) -> None:
    for motor_id in sorted(packets):
        msg = can.Message(arbitration_id=motor_id, data=packets[motor_id], is_extended_id=False)
        bus.send(msg)


def update_motor_feedback_stats(
    plans: list[MotorPlan],
    feedback_by_motor_id: dict[int, dict[str, float]],
    command_joint_by_motor_id: dict[int, tuple[float, float]],
    max_following_error_deg: float,
    stats: PoseStats,
) -> int:
    by_id = {plan.motor_id: plan for plan in plans}
    abort_code = 0

    for plan in plans:
        feedback = feedback_by_motor_id.get(plan.motor_id)
        if feedback is None:
            plan.feedback_missed_count += 1
            plan.consecutive_feedback_misses += 1
            plan.maximum_consecutive_feedback_misses = max(
                plan.maximum_consecutive_feedback_misses,
                plan.consecutive_feedback_misses,
            )
            stats.feedback_frames_missed += 1
            if plan.consecutive_feedback_misses >= MAX_CONSECUTIVE_FEEDBACK_MISSES:
                print(
                    f"Pose aborted: motor {format_motor_id(plan.motor_id)} "
                    "exceeded consecutive feedback miss limit."
                )
                print("Releasing all selected motors.")
                abort_code = 2
            continue

        actual_joint_rad = raw_to_joint_position(
            feedback["raw_pos_rad"],
            plan.raw_zero_pos_rad,
            plan.direction_sign,
        )
        actual_joint_deg = math.degrees(actual_joint_rad)
        command_joint_pos_rad, _command_joint_vel_rad_s = command_joint_by_motor_id[plan.motor_id]
        following_error_deg = math.degrees(command_joint_pos_rad - actual_joint_rad)

        plan.final_actual_joint_deg = actual_joint_deg
        plan.feedback_received_count += 1
        plan.consecutive_feedback_misses = 0
        plan.maximum_abs_following_error_deg = max(
            plan.maximum_abs_following_error_deg,
            abs(following_error_deg),
        )
        stats.feedback_frames_received += 1

        if abs(following_error_deg) > max_following_error_deg:
            plan.consecutive_large_error_cycles += 1
            plan.maximum_consecutive_large_error_cycles = max(
                plan.maximum_consecutive_large_error_cycles,
                plan.consecutive_large_error_cycles,
            )
        else:
            plan.consecutive_large_error_cycles = 0

        if plan.consecutive_large_error_cycles >= MAX_CONSECUTIVE_LARGE_ERROR_CYCLES:
            print(
                f"Pose aborted: motor {format_motor_id(plan.motor_id)} "
                "following error remained too large."
            )
            print("Releasing all selected motors.")
            abort_code = 2

    return abort_code


def print_cycle_status(
    title: str,
    plans: list[MotorPlan],
    command_joint_by_motor_id: dict[int, tuple[float, float]],
    feedback_by_motor_id: dict[int, dict[str, float]],
) -> None:
    print(title)
    print("")
    print("ID      Command      Actual       Error")
    for plan in plans:
        command_joint_pos_rad, _command_joint_vel_rad_s = command_joint_by_motor_id[plan.motor_id]
        command_deg = math.degrees(command_joint_pos_rad)
        feedback = feedback_by_motor_id.get(plan.motor_id)
        if feedback is None:
            print(f"{format_motor_id(plan.motor_id):<7} {command_deg:+8.2f} deg   unavailable   unavailable")
            continue

        actual_joint_rad = raw_to_joint_position(
            feedback["raw_pos_rad"],
            plan.raw_zero_pos_rad,
            plan.direction_sign,
        )
        actual_deg = math.degrees(actual_joint_rad)
        error_deg = command_deg - actual_deg
        print(
            f"{format_motor_id(plan.motor_id):<7} "
            f"{command_deg:+8.2f} deg "
            f"{actual_deg:+8.2f} deg "
            f"{error_deg:+8.2f} deg"
        )


def run_synchronized_cycle(
    bus: can.BusABC,
    plans: list[MotorPlan],
    command_joint_by_motor_id: dict[int, tuple[float, float]],
    max_following_error_deg: float,
    feedback_window_sec: float,
    stats: PoseStats,
) -> tuple[int, dict[int, dict[str, float]]]:
    packets = compute_command_packets(plans, command_joint_by_motor_id)
    if packets is None:
        return 2, {}

    send_packets(bus, packets)
    stats.position_frames_sent += len(packets)

    motor_ids = [plan.motor_id for plan in plans]
    feedback_by_motor_id = collect_feedback_window(
        bus=bus,
        motor_ids=motor_ids,
        sent_packet_by_motor_id=packets,
        timeout_sec=feedback_window_sec,
    )
    abort_code = update_motor_feedback_stats(
        plans=plans,
        feedback_by_motor_id=feedback_by_motor_id,
        command_joint_by_motor_id=command_joint_by_motor_id,
        max_following_error_deg=max_following_error_deg,
        stats=stats,
    )
    return abort_code, feedback_by_motor_id


def run_pose_move(
    bus: can.BusABC,
    plans: list[MotorPlan],
    move_sec: float,
    hold_sec: float,
    rate_hz: float,
    feedback_print_hz: float,
    max_following_error_deg: float,
    move_cycles: int,
    hold_cycles: int,
    stats: PoseStats,
) -> int:
    period_sec = 1.0 / rate_hz
    feedback_window_sec = min(0.008, period_sec * 0.4)
    print_interval_cycles = max(1, int(round(rate_hz / feedback_print_hz)))
    next_send_time = time.monotonic()

    for cycle_index in range(1, move_cycles + 1):
        u = cycle_index / move_cycles
        s, ds_du = smoothstep_quintic(u)
        command_joint_by_motor_id: dict[int, tuple[float, float]] = {}
        for plan in plans:
            command_joint_pos_rad = plan.current_joint_rad + plan.delta_joint_rad * s
            command_joint_vel_rad_s = plan.delta_joint_rad / move_sec * ds_du
            if cycle_index == move_cycles:
                command_joint_pos_rad = plan.target_joint_rad
                command_joint_vel_rad_s = 0.0
            command_joint_by_motor_id[plan.motor_id] = (
                command_joint_pos_rad,
                command_joint_vel_rad_s,
            )

        wait_until_send_time(next_send_time)
        result, feedback = run_synchronized_cycle(
            bus=bus,
            plans=plans,
            command_joint_by_motor_id=command_joint_by_motor_id,
            max_following_error_deg=max_following_error_deg,
            feedback_window_sec=feedback_window_sec,
            stats=stats,
        )
        stats.synchronized_cycles_completed += 1

        should_print = (
            cycle_index == 1
            or cycle_index == move_cycles
            or stats.synchronized_cycles_completed % print_interval_cycles == 0
        )
        if should_print:
            print_cycle_status(
                title=f"[Move {cycle_index}/{move_cycles}]",
                plans=plans,
                command_joint_by_motor_id=command_joint_by_motor_id,
                feedback_by_motor_id=feedback,
            )

        if result != 0:
            return result

        next_send_time = update_next_send_time(next_send_time, period_sec, stats)

    for hold_index in range(1, hold_cycles + 1):
        command_joint_by_motor_id = {
            plan.motor_id: (plan.target_joint_rad, 0.0)
            for plan in plans
        }

        wait_until_send_time(next_send_time)
        result, feedback = run_synchronized_cycle(
            bus=bus,
            plans=plans,
            command_joint_by_motor_id=command_joint_by_motor_id,
            max_following_error_deg=max_following_error_deg,
            feedback_window_sec=feedback_window_sec,
            stats=stats,
        )
        stats.synchronized_cycles_completed += 1

        should_print = (
            hold_index == 1
            or hold_index == hold_cycles
            or stats.synchronized_cycles_completed % print_interval_cycles == 0
        )
        if should_print:
            print_cycle_status(
                title=f"[Hold {hold_index}/{hold_cycles}]",
                plans=plans,
                command_joint_by_motor_id=command_joint_by_motor_id,
                feedback_by_motor_id=feedback,
            )

        if result != 0:
            return result

        next_send_time = update_next_send_time(next_send_time, period_sec, stats)

    return 0


def print_final_report(plans: list[MotorPlan], stats: PoseStats) -> None:
    print("[Multi-Joint Pose End State]")
    print("")
    print("ID      Target       Actual       Final Error   Max Follow Error")
    for plan in plans:
        if plan.final_actual_joint_deg is None:
            actual = "unavailable"
            final_error = "unavailable"
            print(
                f"{format_motor_id(plan.motor_id):<7} "
                f"{plan.target_joint_deg:+8.2f} deg   "
                f"{actual:<11} {final_error:<13} "
                f"{plan.maximum_abs_following_error_deg:8.2f} deg"
            )
            continue

        final_error_deg = plan.target_joint_deg - plan.final_actual_joint_deg
        print(
            f"{format_motor_id(plan.motor_id):<7} "
            f"{plan.target_joint_deg:+8.2f} deg "
            f"{plan.final_actual_joint_deg:+8.2f} deg "
            f"{final_error_deg:+8.2f} deg    "
            f"{plan.maximum_abs_following_error_deg:8.2f} deg"
        )

    print("")
    print(f"Selected motors: {len(plans)}")
    print(f"Synchronized cycles completed: {stats.synchronized_cycles_completed}")
    print(f"Position frames sent: {stats.position_frames_sent}")
    print(f"Feedback frames received: {stats.feedback_frames_received}")
    print(f"Feedback frames missed: {stats.feedback_frames_missed}")
    print(f"Timing overruns: {stats.timing_overruns}")
    print("")
    print("[Per-Motor Feedback Stats]")
    for plan in plans:
        print(
            f"{format_motor_id(plan.motor_id)} | "
            f"feedback_received: {plan.feedback_received_count} | "
            f"feedback_missed: {plan.feedback_missed_count} | "
            f"max_consecutive_misses: {plan.maximum_consecutive_feedback_misses} | "
            f"max_consecutive_large_error: {plan.maximum_consecutive_large_error_cycles}"
        )


def run_once(
    channel: str,
    targets: list[TargetSpec],
    gains: list[GainSpec],
    common_kp: float,
    common_kd: float,
    move_sec: float,
    hold_sec: float,
    rate_hz: float,
    feedback_print_hz: float,
    max_following_error_deg: float,
    enter_mit: bool,
    home_to_zero: bool,
) -> int:
    if not validate_inputs(
        targets=targets,
        gains=gains,
        common_kp=common_kp,
        common_kd=common_kd,
        move_sec=move_sec,
        hold_sec=hold_sec,
        rate_hz=rate_hz,
        feedback_print_hz=feedback_print_hz,
        max_following_error_deg=max_following_error_deg,
        home_to_zero=home_to_zero,
    ):
        return 1

    motor_ids = sorted(target.motor_id for target in targets)
    calibrations = load_calibrations(motor_ids)
    if calibrations is None:
        return 2

    plans, gain_overrides = build_motor_plans(
        targets=targets,
        gains=gains,
        common_kp=common_kp,
        common_kd=common_kd,
        calibrations=calibrations,
    )

    print_start_warning(
        channel=channel,
        plans=plans,
        common_kp=common_kp,
        common_kd=common_kd,
        gain_overrides=gain_overrides,
        move_sec=move_sec,
        hold_sec=hold_sec,
        rate_hz=rate_hz,
        feedback_print_hz=feedback_print_hz,
        max_following_error_deg=max_following_error_deg,
        enter_mit=enter_mit,
        home_to_zero=home_to_zero,
    )

    zero_torque_packet = pack_mit_command(0.0, 0.0, 0.0, 0.0, 0.0)
    if not validate_zero_torque_packet(zero_torque_packet):
        return 2

    for plan in plans:
        target_raw_pos_rad = joint_to_raw_position(
            plan.target_joint_rad,
            plan.raw_zero_pos_rad,
            plan.direction_sign,
        )
        if not command_in_range(target_raw_pos_rad, 0.0):
            print("Pose blocked due to safety limit constraints.")
            print("No position commands were sent.")
            return 2

    if not confirm_yes():
        print("Cancelled.")
        print("No CAN bus was opened.")
        print("No position commands were sent.")
        return 0

    requested_move_sec = move_sec
    effective_move_sec = move_sec
    max_home_delta_deg = 0.0
    bus = None
    released = False
    stats = PoseStats()
    try:
        bus = can.Bus(interface=DEFAULT_INTERFACE, channel=channel)
        if enter_mit:
            enter_mit_for_selected(bus, motor_ids)

        if not read_preflight_positions(bus, plans, zero_torque_packet):
            return 2

        if not validate_pose_plan(plans, home_to_zero=home_to_zero):
            return 2

        if home_to_zero:
            max_home_delta_deg = max(abs(plan.delta_joint_deg) for plan in plans)
            minimum_home_move_sec = max_home_delta_deg / HOME_MAX_AVERAGE_SPEED_DEG_S
            effective_move_sec = max(requested_move_sec, minimum_home_move_sec)

        move_cycles = max(1, int(round(effective_move_sec * rate_hz)))
        hold_cycles = int(round(hold_sec * rate_hz))

        print_pose_plan(
            plans=plans,
            move_sec=effective_move_sec,
            hold_sec=hold_sec,
            rate_hz=rate_hz,
            move_cycles=move_cycles,
            hold_cycles=hold_cycles,
            home_to_zero=home_to_zero,
            requested_move_sec=requested_move_sec,
            max_home_delta_deg=max_home_delta_deg,
        )

        result = run_pose_move(
            bus=bus,
            plans=plans,
            move_sec=effective_move_sec,
            hold_sec=hold_sec,
            rate_hz=rate_hz,
            feedback_print_hz=feedback_print_hz,
            max_following_error_deg=max_following_error_deg,
            move_cycles=move_cycles,
            hold_cycles=hold_cycles,
            stats=stats,
        )
        if result != 0:
            return result

        print("Multi-joint home-to-zero completed." if home_to_zero else "Multi-joint pose completed.")
        release_all_selected(bus, motor_ids, zero_torque_packet)
        released = True
        print_final_report(plans, stats)
        return 0
    except KeyboardInterrupt:
        print("Multi-joint pose interrupted by user.")
        print("Releasing all selected motors.")
        return 1
    except can.CanError:
        print("CAN error during multi-joint pose.")
        print("Releasing all selected motors.")
        return 1
    except Exception as exc:
        print(f"Unexpected error during multi-joint pose: {exc}")
        print("Releasing all selected motors.")
        return 1
    finally:
        if bus is not None:
            if not released:
                send_zero_torque_all_best_effort(bus, motor_ids, zero_torque_packet)
                time.sleep(RELEASE_SETTLE_SEC)
            bus.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run a synchronized finite AK70-10 multi-joint pose move."
    )
    parser.add_argument("--channel", default=DEFAULT_CHANNEL)

    target_group = parser.add_mutually_exclusive_group(required=True)
    target_group.add_argument("--target-deg", action="append", default=None)
    target_group.add_argument("--target-rad", action="append", default=None)

    parser.add_argument("--kp", type=float, default=DEFAULT_KP)
    parser.add_argument("--kd", type=float, default=DEFAULT_KD)
    parser.add_argument("--gain", action="append", default=None)
    parser.add_argument("--move-sec", type=float, default=DEFAULT_MOVE_SEC)
    parser.add_argument("--hold-sec", type=float, default=DEFAULT_HOLD_SEC)
    parser.add_argument("--rate-hz", type=float, default=DEFAULT_RATE_HZ)
    parser.add_argument("--feedback-print-hz", type=float, default=DEFAULT_FEEDBACK_PRINT_HZ)
    parser.add_argument("--max-following-error-deg", type=float, default=DEFAULT_MAX_FOLLOWING_ERROR_DEG)
    parser.add_argument("--enter-mit", action="store_true")
    parser.add_argument(
        "--home-to-zero",
        action="store_true",
        help=(
            "Allow selected calibrated motors to move to "
            "software joint zero without the normal "
            "120-degree delta limit."
        ),
    )
    args = parser.parse_args()

    target_unit = "deg" if args.target_deg is not None else "rad"
    target_values = args.target_deg if args.target_deg is not None else args.target_rad
    targets = parse_targets(target_values, target_unit)
    gains = parse_gains(args.gain)

    raise SystemExit(
        run_once(
            channel=args.channel,
            targets=targets,
            gains=gains,
            common_kp=args.kp,
            common_kd=args.kd,
            move_sec=args.move_sec,
            hold_sec=args.hold_sec,
            rate_hz=args.rate_hz,
            feedback_print_hz=args.feedback_print_hz,
            max_following_error_deg=args.max_following_error_deg,
            enter_mit=args.enter_mit,
            home_to_zero=args.home_to_zero,
        )
    )


if __name__ == "__main__":
    main()
