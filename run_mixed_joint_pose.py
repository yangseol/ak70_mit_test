#!/usr/bin/env python3
"""Run a finite synchronized mixed AK70/AK45 pose move."""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from typing import Any

import can

from ak45_calibration import (
    calibration_exists as ak45_calibration_exists,
    joint_to_raw_position as ak45_joint_to_raw_position,
    joint_to_raw_velocity as ak45_joint_to_raw_velocity,
    load_ak45_calibration,
)
from calibration import load_motor_calibration
from homing_state import HomingMachine, HomingState
from mixed_mit_packet import MitCommand, pack_checked_commands, pack_mit_command, unpack_mit_feedback
from motor_profiles import MotorProfile, format_motor_id, get_motor_profile, is_ak45, normalize_motor_id


DEFAULT_CHANNEL = "can0"
DEFAULT_INTERFACE = "socketcan"
DEFAULT_KP = 8.0
DEFAULT_KD = 0.4
DEFAULT_MOVE_SEC = 2.0
DEFAULT_HOLD_SEC = 2.0
DEFAULT_RATE_HZ = 50.0
DEFAULT_FEEDBACK_PRINT_HZ = 5.0
DEFAULT_MAX_FOLLOWING_ERROR_DEG = 60.0
MAX_MOTORS = 13
LOWER_BODY_MOTOR_COUNT = 12
MAX_CONSECUTIVE_FEEDBACK_MISSES = 10
MAX_CONSECUTIVE_LARGE_ERROR_CYCLES = 10
PREFLIGHT_FEEDBACK_WINDOW_SEC = 1.0
RECV_POLL_TIMEOUT_SEC = 0.002
RELEASE_SETTLE_SEC = 0.03
HOME_MAX_AVERAGE_SPEED_DEG_S = 60.0
MIT_ENTER_PACKET = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
MIT_ENTER_BETWEEN_SEC = 0.02
MIT_ENTER_SLEEP_SEC = 0.02


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


@dataclass(frozen=True)
class CalibrationSpec:
    raw_zero_pos_rad: float
    direction_sign: int


@dataclass
class MotorPlan:
    motor_id: int
    profile: MotorProfile
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
    consecutive_feedback_misses: int = 0
    consecutive_large_error_cycles: int = 0
    feedback_received_count: int = 0
    feedback_missed_count: int = 0
    maximum_abs_following_error_deg: float = 0.0


@dataclass
class PoseStats:
    synchronized_cycles_completed: int = 0
    position_frames_sent: int = 0
    feedback_frames_received: int = 0
    feedback_frames_missed: int = 0
    timing_overruns: int = 0


def parse_motor_id(value: str) -> int:
    try:
        return normalize_motor_id(value)
    except Exception as exc:
        raise argparse.ArgumentTypeError(f"invalid motor id: {value!r}") from exc


def parse_target(value: str, unit: str) -> TargetSpec:
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("target must use MOTOR_ID,ANGLE format")
    try:
        motor_id = parse_motor_id(parts[0])
        angle = float(parts[1])
    except (ValueError, argparse.ArgumentTypeError) as exc:
        raise argparse.ArgumentTypeError(f"invalid target: {value!r}") from exc
    if not math.isfinite(angle):
        raise argparse.ArgumentTypeError("target angle must be finite")
    if unit == "deg":
        target_deg = angle
        target_rad = math.radians(angle)
    else:
        target_rad = angle
        target_deg = math.degrees(angle)
    return TargetSpec(motor_id, target_rad, target_deg, unit, angle)


def parse_targets(values: list[str] | None, unit: str) -> list[TargetSpec]:
    return [parse_target(value, unit) for value in values or []]


def parse_gain(value: str) -> GainSpec:
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("gain must use MOTOR_ID,KP,KD format")
    try:
        motor_id = parse_motor_id(parts[0])
        kp = float(parts[1])
        kd = float(parts[2])
    except (ValueError, argparse.ArgumentTypeError) as exc:
        raise argparse.ArgumentTypeError(f"invalid gain override: {value!r}") from exc
    return GainSpec(motor_id, kp, kd)


def parse_gains(values: list[str] | None) -> list[GainSpec]:
    return [parse_gain(value) for value in values or []]


def validate_gain(profile: MotorProfile, kp: float, kd: float, label: str, errors: list[str]) -> None:
    if not math.isfinite(kp) or not profile.kp_min <= kp <= profile.kp_max:
        errors.append(f"{label} Kp outside {profile.kp_min}..{profile.kp_max}")
    if not math.isfinite(kd) or not profile.kd_min <= kd <= profile.kd_max:
        errors.append(f"{label} Kd outside {profile.kd_min}..{profile.kd_max}")


def validate_static_inputs(
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
    ak45_power_verified: bool,
    dry_run: bool,
) -> list[str]:
    errors: list[str] = []
    if not 1 <= len(targets) <= MAX_MOTORS:
        errors.append(f"target count must be 1..{MAX_MOTORS}")
    seen: set[int] = set()
    has_ak45 = False
    for target in targets:
        try:
            profile = get_motor_profile(target.motor_id)
        except ValueError as exc:
            errors.append(str(exc))
            continue
        if target.motor_id in seen:
            errors.append(f"duplicate target ID {format_motor_id(target.motor_id)}")
        seen.add(target.motor_id)
        has_ak45 = has_ak45 or profile.model == "AK45-36-KV80"
        if not math.isfinite(target.target_joint_rad):
            errors.append(f"{format_motor_id(target.motor_id)} target must be finite")
        if abs(target.target_joint_deg) > profile.bench_target_limit_deg:
            errors.append(f"{format_motor_id(target.motor_id)} target exceeds bench +/-{profile.bench_target_limit_deg:.1f} deg")
        if home_to_zero:
            if target.source_unit == "deg" and abs(target.source_angle) > 1e-9:
                errors.append("--home-to-zero requires every target to be exactly 0 deg")
            if target.source_unit == "rad" and abs(target.source_angle) > 1e-12:
                errors.append("--home-to-zero requires every target to be exactly 0 deg")

    if has_ak45 and not ak45_power_verified and not dry_run:
        errors.append("AK45-36 requires --ak45-power-verified for actual movement")
    if has_ak45:
        print("AK45-36 전원 요구사항: 정격 24V, 허용 16~28V, 48V 연결 금지.")
        print("AK70과 AK45는 같은 CAN bus를 쓸 수 있지만 전원 레일은 분리하고 통신 기준 GND는 공유해야 합니다.")

    target_ids = {target.motor_id for target in targets}
    seen_gains: set[int] = set()
    for gain in gains:
        if gain.motor_id in seen_gains:
            errors.append(f"duplicate gain ID {format_motor_id(gain.motor_id)}")
        seen_gains.add(gain.motor_id)
        if gain.motor_id not in target_ids:
            errors.append(f"gain override for untargeted ID {format_motor_id(gain.motor_id)}")
        try:
            validate_gain(get_motor_profile(gain.motor_id), gain.kp, gain.kd, format_motor_id(gain.motor_id), errors)
        except ValueError as exc:
            errors.append(str(exc))
    for target in targets:
        try:
            validate_gain(get_motor_profile(target.motor_id), common_kp, common_kd, "common", errors)
        except ValueError as exc:
            errors.append(str(exc))
            break

    finite_values = {
        "move-sec": move_sec,
        "hold-sec": hold_sec,
        "rate-hz": rate_hz,
        "feedback-print-hz": feedback_print_hz,
        "max-following-error-deg": max_following_error_deg,
    }
    for label, value in finite_values.items():
        if not math.isfinite(value):
            errors.append(f"{label} must be finite")
    if not 0.2 <= move_sec <= 15.0:
        errors.append("move-sec must be 0.2..15.0")
    if not 0.0 <= hold_sec <= 30.0:
        errors.append("hold-sec must be 0.0..30.0")
    if not 10.0 <= rate_hz <= 100.0:
        errors.append("rate-hz must be 10.0..100.0")
    if not 0.5 <= feedback_print_hz <= rate_hz:
        errors.append("feedback-print-hz must be 0.5..rate-hz")
    if not 10.0 <= max_following_error_deg <= 120.0:
        errors.append("max-following-error-deg must be 10.0..120.0")
    return errors


def load_calibrations(motor_ids: list[int]) -> dict[int, CalibrationSpec]:
    ak70_data = load_motor_calibration()
    ak45_data = load_ak45_calibration()
    result: dict[int, CalibrationSpec] = {}
    for motor_id in motor_ids:
        key = format_motor_id(motor_id)
        if is_ak45(motor_id):
            if not ak45_calibration_exists(motor_id, ak45_data):
                raise ValueError(f"AK45 calibration missing for {key}")
            entry = ak45_data["motors"][key]
        else:
            try:
                entry = ak70_data["motors"][key]
            except KeyError as exc:
                raise ValueError(f"AK70 calibration missing for {key}") from exc
        raw_zero = float(entry["raw_zero_pos_rad"])
        direction_sign = int(entry.get("direction_sign", 1))
        if not math.isfinite(raw_zero) or direction_sign not in (-1, 1):
            raise ValueError(f"invalid calibration for {key}")
        result[motor_id] = CalibrationSpec(raw_zero, direction_sign)
    return result


def raw_to_joint_position(raw_pos_rad: float, raw_zero_pos_rad: float, direction_sign: int) -> float:
    return direction_sign * (raw_pos_rad - raw_zero_pos_rad)


def joint_to_raw_position(motor_id: int, joint_pos_rad: float, raw_zero_pos_rad: float, direction_sign: int) -> float:
    if is_ak45(motor_id):
        return ak45_joint_to_raw_position(joint_pos_rad, raw_zero_pos_rad, direction_sign)
    return raw_zero_pos_rad + direction_sign * joint_pos_rad


def joint_to_raw_velocity(motor_id: int, joint_vel_rad_s: float, direction_sign: int) -> float:
    if is_ak45(motor_id):
        return ak45_joint_to_raw_velocity(joint_vel_rad_s, direction_sign)
    return direction_sign * joint_vel_rad_s


def build_motor_plans(
    targets: list[TargetSpec],
    gains: list[GainSpec],
    common_kp: float,
    common_kd: float,
    calibrations: dict[int, CalibrationSpec],
) -> list[MotorPlan]:
    gain_by_id = {gain.motor_id: (gain.kp, gain.kd) for gain in gains}
    plans: list[MotorPlan] = []
    for target in sorted(targets, key=lambda item: item.motor_id):
        calibration = calibrations[target.motor_id]
        kp, kd = gain_by_id.get(target.motor_id, (common_kp, common_kd))
        plans.append(
            MotorPlan(
                motor_id=target.motor_id,
                profile=get_motor_profile(target.motor_id),
                target_joint_rad=target.target_joint_rad,
                target_joint_deg=target.target_joint_deg,
                raw_zero_pos_rad=calibration.raw_zero_pos_rad,
                direction_sign=calibration.direction_sign,
                kp=kp,
                kd=kd,
            )
        )
    return plans


def validate_homing_for_actual(plans: list[MotorPlan], session_homed_ids: set[int]) -> None:
    for plan in plans:
        if plan.profile.model == "AK45-36-KV80" and plan.motor_id not in session_homed_ids:
            raise ValueError(f"{format_motor_id(plan.motor_id)} AK45 is UNHOMED in this controller session")


def validate_pose_plan(plans: list[MotorPlan], home_to_zero: bool) -> list[str]:
    errors: list[str] = []
    commands: list[MitCommand] = []
    for plan in plans:
        if abs(plan.target_joint_deg) > plan.profile.bench_target_limit_deg:
            errors.append(f"{format_motor_id(plan.motor_id)} target exceeds bench limit")
        if not home_to_zero and abs(plan.delta_joint_deg) > plan.profile.bench_delta_limit_deg:
            errors.append(f"{format_motor_id(plan.motor_id)} delta exceeds bench limit")
        target_raw = joint_to_raw_position(plan.motor_id, plan.target_joint_rad, plan.raw_zero_pos_rad, plan.direction_sign)
        commands.append(MitCommand(plan.motor_id, target_raw, 0.0, plan.kp, plan.kd, 0.0))
    try:
        pack_checked_commands(commands)
    except ValueError as exc:
        errors.append(str(exc))
    return errors


def smoothstep_quintic(u: float) -> tuple[float, float]:
    s = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
    ds_du = 30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4
    return s, ds_du


def compute_command_packets(plans: list[MotorPlan], command_joint_by_motor_id: dict[int, tuple[float, float]]) -> dict[int, bytes]:
    commands: list[MitCommand] = []
    for plan in plans:
        joint_pos, joint_vel = command_joint_by_motor_id[plan.motor_id]
        raw_pos = joint_to_raw_position(plan.motor_id, joint_pos, plan.raw_zero_pos_rad, plan.direction_sign)
        raw_vel = joint_to_raw_velocity(plan.motor_id, joint_vel, plan.direction_sign)
        commands.append(MitCommand(plan.motor_id, raw_pos, raw_vel, plan.kp, plan.kd, 0.0))
    return pack_checked_commands(commands)


def send_packets(bus: can.BusABC, packets: dict[int, bytes]) -> None:
    for motor_id in sorted(packets):
        bus.send(can.Message(arbitration_id=motor_id, data=packets[motor_id], is_extended_id=False))


def enter_mit_for_selected(bus: can.BusABC, motor_ids: list[int]) -> None:
    for motor_id in sorted(motor_ids):
        bus.send(can.Message(arbitration_id=motor_id, data=MIT_ENTER_PACKET, is_extended_id=False))
        time.sleep(MIT_ENTER_BETWEEN_SEC)
    time.sleep(MIT_ENTER_SLEEP_SEC)


def collect_feedback_window(bus: can.BusABC, motor_ids: list[int], sent_packet_by_motor_id: dict[int, bytes], timeout_sec: float) -> dict[int, dict[str, float]]:
    selected = set(motor_ids)
    feedback: dict[int, dict[str, float]] = {}
    deadline = time.monotonic() + max(0.0, timeout_sec)
    while time.monotonic() < deadline and len(feedback) < len(selected):
        msg = bus.recv(timeout=min(RECV_POLL_TIMEOUT_SEC, max(0.0, deadline - time.monotonic())))
        if msg is None or msg.arbitration_id not in selected or len(msg.data) != 8:
            continue
        motor_id = msg.arbitration_id
        data = bytes(msg.data)
        if data == sent_packet_by_motor_id.get(motor_id) or data == MIT_ENTER_PACKET:
            continue
        try:
            decoded = unpack_mit_feedback(motor_id, data)
        except ValueError:
            continue
        feedback[motor_id] = {"raw_pos_rad": decoded.position, "velocity_rad_s": decoded.velocity, "effort": decoded.torque}
    return feedback


def read_preflight_positions(bus: can.BusABC, plans: list[MotorPlan]) -> bool:
    motor_ids = [plan.motor_id for plan in plans]
    packets = {motor_id: pack_mit_command(motor_id, 0.0, 0.0, 0.0, 0.0, 0.0) for motor_id in motor_ids}
    send_packets(bus, packets)
    feedback = collect_feedback_window(bus, motor_ids, packets, PREFLIGHT_FEEDBACK_WINDOW_SEC)
    for plan in plans:
        item = feedback.get(plan.motor_id)
        if item is None:
            print(f"Preflight failed: no feedback from {format_motor_id(plan.motor_id)}")
            return False
        raw_pos = item["raw_pos_rad"]
        plan.current_joint_rad = raw_to_joint_position(raw_pos, plan.raw_zero_pos_rad, plan.direction_sign)
        plan.current_joint_deg = math.degrees(plan.current_joint_rad)
        plan.delta_joint_rad = plan.target_joint_rad - plan.current_joint_rad
        plan.delta_joint_deg = plan.target_joint_deg - plan.current_joint_deg
    return True


def update_feedback_stats(
    plans: list[MotorPlan],
    feedback_by_motor_id: dict[int, dict[str, float]],
    command_joint_by_motor_id: dict[int, tuple[float, float]],
    max_following_error_deg: float,
    stats: PoseStats,
) -> int:
    result = 0
    for plan in plans:
        feedback = feedback_by_motor_id.get(plan.motor_id)
        if feedback is None:
            plan.consecutive_feedback_misses += 1
            plan.feedback_missed_count += 1
            stats.feedback_frames_missed += 1
            if plan.consecutive_feedback_misses >= MAX_CONSECUTIVE_FEEDBACK_MISSES:
                result = 2
            continue
        actual = raw_to_joint_position(feedback["raw_pos_rad"], plan.raw_zero_pos_rad, plan.direction_sign)
        command_pos, _ = command_joint_by_motor_id[plan.motor_id]
        error_deg = math.degrees(command_pos - actual)
        plan.final_actual_joint_deg = math.degrees(actual)
        plan.feedback_received_count += 1
        plan.consecutive_feedback_misses = 0
        stats.feedback_frames_received += 1
        plan.maximum_abs_following_error_deg = max(plan.maximum_abs_following_error_deg, abs(error_deg))
        if abs(error_deg) > max_following_error_deg:
            plan.consecutive_large_error_cycles += 1
            if plan.consecutive_large_error_cycles >= MAX_CONSECUTIVE_LARGE_ERROR_CYCLES:
                result = 2
        else:
            plan.consecutive_large_error_cycles = 0
    return result


def run_pose_move(bus: can.BusABC, plans: list[MotorPlan], move_sec: float, hold_sec: float, rate_hz: float, max_following_error_deg: float, stats: PoseStats) -> int:
    period_sec = 1.0 / rate_hz
    feedback_window_sec = min(0.008, period_sec * 0.4)
    move_cycles = max(1, int(round(move_sec * rate_hz)))
    hold_cycles = int(round(hold_sec * rate_hz))
    next_time = time.monotonic()
    for cycle_index in range(1, move_cycles + hold_cycles + 1):
        if cycle_index <= move_cycles:
            u = cycle_index / move_cycles
            s, ds_du = smoothstep_quintic(u)
            command_joint = {
                plan.motor_id: (
                    plan.target_joint_rad if cycle_index == move_cycles else plan.current_joint_rad + plan.delta_joint_rad * s,
                    0.0 if cycle_index == move_cycles else plan.delta_joint_rad / move_sec * ds_du,
                )
                for plan in plans
            }
        else:
            command_joint = {plan.motor_id: (plan.target_joint_rad, 0.0) for plan in plans}
        packets = compute_command_packets(plans, command_joint)
        remaining = next_time - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
        send_packets(bus, packets)
        stats.position_frames_sent += len(packets)
        feedback = collect_feedback_window(bus, [p.motor_id for p in plans], packets, feedback_window_sec)
        result = update_feedback_stats(plans, feedback, command_joint, max_following_error_deg, stats)
        stats.synchronized_cycles_completed += 1
        scheduled = next_time + period_sec
        if time.monotonic() >= scheduled:
            stats.timing_overruns += 1
            next_time = time.monotonic() + period_sec
        else:
            next_time = scheduled
        if result != 0:
            return result
    return 0


def send_zero_torque_all_best_effort(bus: can.BusABC | None, motor_ids: list[int]) -> None:
    if bus is None:
        return
    packets = {motor_id: pack_mit_command(motor_id, 0.0, 0.0, 0.0, 0.0, 0.0) for motor_id in motor_ids}
    try:
        send_packets(bus, packets)
    except Exception as exc:
        print(f"zero-torque best effort failed: {exc}")


def print_plan(plans: list[MotorPlan], dry_run: bool) -> None:
    title = "[Mixed Pose Dry-Run Plan]" if dry_run else "[Mixed Pose Plan]"
    print(title)
    print(f"selected motors: {len(plans)}")
    print("ID      Model          Current      Target       Kp      Kd")
    for plan in plans:
        print(
            f"{format_motor_id(plan.motor_id):<7} {plan.profile.model:<14} "
            f"{plan.current_joint_deg:+8.2f} deg {plan.target_joint_deg:+8.2f} deg "
            f"{plan.kp:<7.2f} {plan.kd:<7.2f}"
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
    ak45_power_verified: bool,
    dry_run: bool,
    session_homed_ids: set[int] | None = None,
) -> int:
    errors = validate_static_inputs(
        targets,
        gains,
        common_kp,
        common_kd,
        move_sec,
        hold_sec,
        rate_hz,
        feedback_print_hz,
        max_following_error_deg,
        home_to_zero,
        ak45_power_verified,
        dry_run,
    )
    if errors:
        for error in errors:
            print(f"Error: {error}")
        return 1
    motor_ids = sorted(target.motor_id for target in targets)
    try:
        calibrations = load_calibrations(motor_ids)
        plans = build_motor_plans(targets, gains, common_kp, common_kd, calibrations)
    except Exception as exc:
        print(f"Error: {exc}")
        return 2

    if dry_run:
        for plan in plans:
            plan.current_joint_rad = 0.0
            plan.current_joint_deg = 0.0
            plan.delta_joint_rad = plan.target_joint_rad
            plan.delta_joint_deg = plan.target_joint_deg
        plan_errors = validate_pose_plan(plans, home_to_zero)
        if plan_errors:
            for error in plan_errors:
                print(f"Error: {error}")
            return 2
        print_plan(plans, dry_run=True)
        print("dry-run: no CAN bus opened and no motor command sent")
        return 0

    try:
        validate_homing_for_actual(plans, session_homed_ids or set())
    except ValueError as exc:
        print(f"Error: {exc}")
        print("No position commands were sent.")
        return 2

    bus = None
    stats = PoseStats()
    try:
        bus = can.Bus(interface=DEFAULT_INTERFACE, channel=channel)
        if enter_mit:
            enter_mit_for_selected(bus, motor_ids)
        if not read_preflight_positions(bus, plans):
            return 2
        plan_errors = validate_pose_plan(plans, home_to_zero)
        if plan_errors:
            for error in plan_errors:
                print(f"Error: {error}")
            return 2
        if home_to_zero:
            max_home_delta_deg = max(abs(plan.delta_joint_deg) for plan in plans)
            move_sec = max(move_sec, max_home_delta_deg / HOME_MAX_AVERAGE_SPEED_DEG_S)
        print_plan(plans, dry_run=False)
        return run_pose_move(bus, plans, move_sec, hold_sec, rate_hz, max_following_error_deg, stats)
    except KeyboardInterrupt:
        print("Interrupted. Releasing all selected motors.")
        return 1
    except can.CanError as exc:
        print(f"CAN error: {exc}. Releasing all selected motors.")
        return 1
    finally:
        if bus is not None:
            send_zero_torque_all_best_effort(bus, motor_ids)
            time.sleep(RELEASE_SETTLE_SEC)
            bus.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a synchronized finite mixed AK70/AK45 pose move.")
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
    parser.add_argument("--home-to-zero", action="store_true")
    parser.add_argument("--ak45-power-verified", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--session-homed", action="append", type=parse_motor_id, default=None, help=argparse.SUPPRESS)
    args = parser.parse_args()
    target_unit = "deg" if args.target_deg is not None else "rad"
    targets = parse_targets(args.target_deg if args.target_deg is not None else args.target_rad, target_unit)
    raise SystemExit(
        run_once(
            channel=args.channel,
            targets=targets,
            gains=parse_gains(args.gain),
            common_kp=args.kp,
            common_kd=args.kd,
            move_sec=args.move_sec,
            hold_sec=args.hold_sec,
            rate_hz=args.rate_hz,
            feedback_print_hz=args.feedback_print_hz,
            max_following_error_deg=args.max_following_error_deg,
            enter_mit=args.enter_mit,
            home_to_zero=args.home_to_zero,
            ak45_power_verified=args.ak45_power_verified,
            dry_run=args.dry_run,
            session_homed_ids=set(args.session_homed or []),
        )
    )


if __name__ == "__main__":
    main()
