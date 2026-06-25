"""Run a finite AK70-10 single-joint waypoint trajectory."""

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
DEFAULT_KP = 5.0
DEFAULT_KD = 0.3
DEFAULT_FINAL_HOLD_SEC = 2.0
DEFAULT_RATE_HZ = 50.0
DEFAULT_FEEDBACK_PRINT_HZ = 5.0
DEFAULT_MAX_FOLLOWING_ERROR_DEG = 45.0
DRAIN_DURATION_SEC = 0.05
DRAIN_POLL_TIMEOUT_SEC = 0.005
INITIAL_FEEDBACK_TIMEOUT_SEC = 1.0
RECV_POLL_TIMEOUT_SEC = 0.05
MIT_ENTER_SLEEP_SEC = 0.02
RELEASE_SETTLE_SEC = 0.03

MAX_SAFETY_TARGET_DEG = 120.0
MAX_SAFETY_SEGMENT_DELTA_DEG = 120.0
MAX_CONSECUTIVE_FEEDBACK_MISSES = 10
MAX_CONSECUTIVE_LARGE_ERROR_CYCLES = 10
MAX_WAYPOINTS = 20
MAX_TOTAL_TRAJECTORY_SEC = 60.0

EXPECTED_ZERO_TORQUE_PACKET = bytes([0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x00])
MIT_ENTER_PACKET = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])


@dataclass(frozen=True)
class Waypoint:
    target_joint_rad: float
    target_joint_deg: float
    duration_sec: float


@dataclass(frozen=True)
class SegmentPlan:
    index: int
    start_joint_rad: float
    start_joint_deg: float
    target_joint_rad: float
    target_joint_deg: float
    duration_sec: float
    cycles: int

    @property
    def delta_joint_rad(self) -> float:
        return self.target_joint_rad - self.start_joint_rad

    @property
    def delta_joint_deg(self) -> float:
        return self.target_joint_deg - self.start_joint_deg


@dataclass(frozen=True)
class TrajectoryPlan:
    current_joint_rad: float
    current_joint_deg: float
    raw_zero_pos_rad: float
    direction_sign: int
    segments: list[SegmentPlan]
    final_hold_cycles: int

    @property
    def total_trajectory_cycles(self) -> int:
        return sum(segment.cycles for segment in self.segments) + self.final_hold_cycles

    @property
    def final_target_joint_rad(self) -> float:
        return self.segments[-1].target_joint_rad

    @property
    def final_target_joint_deg(self) -> float:
        return self.segments[-1].target_joint_deg


@dataclass
class TrajectoryStats:
    trajectory_commands_sent: int = 0
    feedback_received_count: int = 0
    feedback_missed_count: int = 0
    consecutive_feedback_misses: int = 0
    maximum_consecutive_feedback_misses: int = 0
    consecutive_large_error_cycles: int = 0
    maximum_consecutive_large_error_cycles: int = 0
    maximum_abs_following_error_deg: float = 0.0
    timing_overruns: int = 0
    last_actual_joint_deg: float | None = None


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


def parse_waypoint(value: str, unit: str) -> Waypoint:
    parts = value.split(",", 1)
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("waypoint must use ANGLE,DURATION_SEC format")

    try:
        angle = float(parts[0].strip())
        duration_sec = float(parts[1].strip())
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid waypoint: {value!r}") from exc

    if unit == "deg":
        target_joint_deg = angle
        target_joint_rad = math.radians(angle)
    elif unit == "rad":
        target_joint_rad = angle
        target_joint_deg = math.degrees(angle)
    else:
        raise argparse.ArgumentTypeError(f"invalid waypoint unit: {unit!r}")

    return Waypoint(
        target_joint_rad=target_joint_rad,
        target_joint_deg=target_joint_deg,
        duration_sec=duration_sec,
    )


def parse_waypoints(values: list[str] | None, unit: str) -> list[Waypoint]:
    if values is None:
        return []
    return [parse_waypoint(value, unit) for value in values]


def format_waypoints(waypoints: list[Waypoint]) -> str:
    return ", ".join(
        f"{waypoint.target_joint_deg:+.2f} deg/{waypoint.duration_sec:.3f} sec"
        for waypoint in waypoints
    )


def validate_arguments(
    motor_id: int,
    waypoints: list[Waypoint],
    kp: float,
    kd: float,
    final_hold_sec: float,
    rate_hz: float,
    feedback_print_hz: float,
    max_following_error_deg: float,
) -> bool:
    errors: list[str] = []

    if not (0x001 <= motor_id <= 0x00A):
        errors.append("Error: motor-id must be in AK70 range 0x001~0x00A")

    if not 1 <= len(waypoints) <= MAX_WAYPOINTS:
        errors.append(f"Invalid waypoint count: must be >= 1 and <= {MAX_WAYPOINTS}")

    float_values = {
        "kp": kp,
        "kd": kd,
        "final_hold_sec": final_hold_sec,
        "rate_hz": rate_hz,
        "feedback_print_hz": feedback_print_hz,
        "max_following_error_deg": max_following_error_deg,
    }
    for waypoint_index, waypoint in enumerate(waypoints, start=1):
        float_values[f"waypoint_{waypoint_index}_target_rad"] = waypoint.target_joint_rad
        float_values[f"waypoint_{waypoint_index}_target_deg"] = waypoint.target_joint_deg
        float_values[f"waypoint_{waypoint_index}_duration_sec"] = waypoint.duration_sec

    for name, value in float_values.items():
        if not math.isfinite(value):
            errors.append(f"Invalid {name}: must be finite")

    if not 0.0 < kp <= 10.0:
        errors.append("Invalid kp: must be > 0.0 and <= 10.0")
    if not 0.0 <= kd <= 2.0:
        errors.append("Invalid kd: must be >= 0.0 and <= 2.0")
    if not 0.0 <= final_hold_sec <= 30.0:
        errors.append("Invalid final-hold-sec: must be >= 0.0 and <= 30.0")
    if not 10.0 <= rate_hz <= 200.0:
        errors.append("Invalid rate-hz: must be >= 10.0 and <= 200.0")
    if not 0.5 <= feedback_print_hz <= rate_hz:
        errors.append("Invalid feedback-print-hz: must be >= 0.5 and <= rate-hz")
    if not 5.0 <= max_following_error_deg <= 120.0:
        errors.append("Invalid max-following-error-deg: must be >= 5.0 and <= 120.0")

    total_duration_sec = 0.0
    for waypoint_index, waypoint in enumerate(waypoints, start=1):
        if not 0.1 <= waypoint.duration_sec <= 10.0:
            errors.append(
                f"Invalid waypoint {waypoint_index} duration: must be >= 0.1 and <= 10.0"
            )
        if abs(waypoint.target_joint_deg) > MAX_SAFETY_TARGET_DEG:
            errors.append(
                f"Invalid waypoint {waypoint_index} target: absolute target must be <= "
                f"{MAX_SAFETY_TARGET_DEG:.1f} deg"
            )
        total_duration_sec += waypoint.duration_sec

    if total_duration_sec > MAX_TOTAL_TRAJECTORY_SEC:
        errors.append(
            f"Invalid trajectory duration: segment total must be <= {MAX_TOTAL_TRAJECTORY_SEC:.1f} sec"
        )

    for error in errors:
        print(error)
    return not errors


def print_safety_warning(
    channel: str,
    motor_id: int,
    waypoints: list[Waypoint],
    kp: float,
    kd: float,
    final_hold_sec: float,
    rate_hz: float,
    feedback_print_hz: float,
    max_following_error_deg: float,
    enter_mit: bool,
) -> None:
    print("SAFETY WARNING")
    print("- AK70 finite single-joint trajectory")
    print("- one motor only")
    print("- periodic position and velocity commands")
    print("- finite waypoint sequence")
    print("- no 0xFE")
    print("- no calibration write")
    print("- Ctrl+C releases torque")
    print(f"channel: {channel}")
    print(f"motor_id: {format_motor_id(motor_id)}")
    print(f"waypoints: {format_waypoints(waypoints)}")
    print(f"kp: {kp}")
    print(f"kd: {kd}")
    print(f"final_hold_sec: {final_hold_sec:.3f}")
    print(f"rate_hz: {rate_hz:.1f}")
    print(f"feedback_print_hz: {feedback_print_hz:.1f}")
    print(f"max_following_error_deg: {max_following_error_deg:.1f}")
    print(f"enter_mit: {enter_mit}")


def confirm_yes() -> bool:
    return input("Type YES to continue: ").strip() == "YES"


def confirm_trajectory() -> bool:
    return input("Type TRAJECTORY to execute the finite waypoint sequence: ").strip() == "TRAJECTORY"


def validate_zero_torque_packet(packet: bytes) -> bool:
    if packet == EXPECTED_ZERO_TORQUE_PACKET:
        return True

    print(
        "Refusing to transmit: zero-torque packet mismatch "
        f"actual={format_packet(packet)} expected={format_packet(EXPECTED_ZERO_TORQUE_PACKET)}"
    )
    return False


def get_motor_calibration(motor_id: int, calibration: dict) -> tuple[float, int] | None:
    motor_key = format_motor_id(motor_id)
    try:
        motor_calibration = calibration["motors"][motor_key]
    except KeyError:
        print(f"Error: calibration not found for motor {motor_key}")
        return None

    try:
        raw_zero_pos_rad = float(motor_calibration["raw_zero_pos_rad"])
        direction_sign = int(motor_calibration["direction_sign"])
    except (KeyError, TypeError, ValueError) as exc:
        print(f"Error: invalid calibration for motor {motor_key}: {exc}")
        return None

    if not math.isfinite(raw_zero_pos_rad):
        print("Error: raw_zero_pos_rad must be finite")
        return None

    if direction_sign not in (-1, 1):
        print("Error: direction_sign must be 1 or -1")
        return None

    return raw_zero_pos_rad, direction_sign


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


def receive_valid_feedback(
    bus: can.BusABC,
    motor_id: int,
    echo_packet: bytes,
    timeout_sec: float,
) -> dict[str, float] | None:
    deadline = time.monotonic() + max(0.0, timeout_sec)

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
        if raw_bytes == echo_packet or raw_bytes == MIT_ENTER_PACKET:
            continue

        if raw_bytes[0] != (motor_id & 0xFF):
            continue

        candidate = analyze_feedback_candidate(raw_bytes)
        if candidate is None:
            continue

        return {
            "raw_pos_rad": float(candidate["candidate_position_rad"]),
            "velocity_rad_s": float(candidate["candidate_velocity_rads"]),
            "effort": float(candidate["candidate_effort_from_torque_limit"]),
        }

    return None


def read_current_joint_once(
    bus: can.BusABC,
    motor_id: int,
    raw_zero_pos_rad: float,
    direction_sign: int,
    zero_torque_packet: bytes,
) -> dict[str, float] | None:
    drain_rx_queue(bus)

    msg = can.Message(arbitration_id=motor_id, data=zero_torque_packet, is_extended_id=False)
    bus.send(msg)

    feedback = receive_valid_feedback(
        bus=bus,
        motor_id=motor_id,
        echo_packet=zero_torque_packet,
        timeout_sec=INITIAL_FEEDBACK_TIMEOUT_SEC,
    )
    if feedback is None:
        return None

    raw_pos_rad = feedback["raw_pos_rad"]
    joint_rad = raw_to_joint_position(raw_pos_rad, raw_zero_pos_rad, direction_sign)
    return {
        "raw_pos_rad": raw_pos_rad,
        "joint_rad": joint_rad,
        "joint_deg": math.degrees(joint_rad),
    }


def send_zero_torque_best_effort(bus: can.BusABC | None, motor_id: int, zero_torque_packet: bytes) -> bool:
    if bus is None:
        return False

    try:
        msg = can.Message(arbitration_id=motor_id, data=zero_torque_packet, is_extended_id=False)
        bus.send(msg)
        return True
    except Exception as exc:
        print(f"Best-effort zero-torque send failed: {exc}")
        return False


def release_joint(bus: can.BusABC, motor_id: int, zero_torque_packet: bytes) -> None:
    if send_zero_torque_best_effort(bus, motor_id, zero_torque_packet):
        print("Zero-torque sent.")
    time.sleep(RELEASE_SETTLE_SEC)

    feedback = receive_valid_feedback(
        bus=bus,
        motor_id=motor_id,
        echo_packet=zero_torque_packet,
        timeout_sec=RELEASE_SETTLE_SEC,
    )
    if feedback is None:
        print("Release feedback not received.")
    else:
        print("Joint released.")


def wait_until_send_time(next_send_time: float) -> None:
    remaining = next_send_time - time.monotonic()
    if remaining > 0.0:
        time.sleep(remaining)


def update_next_send_time(next_send_time: float, period_sec: float, stats: TrajectoryStats) -> float:
    scheduled_next = next_send_time + period_sec
    now = time.monotonic()

    if now >= scheduled_next:
        stats.timing_overruns += 1
        return now + period_sec

    return scheduled_next


def smoothstep_quintic(u: float) -> tuple[float, float]:
    s = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
    ds_du = 30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4
    return s, ds_du


def build_trajectory_plan(
    current_state: dict[str, float],
    waypoints: list[Waypoint],
    raw_zero_pos_rad: float,
    direction_sign: int,
    final_hold_sec: float,
    rate_hz: float,
) -> TrajectoryPlan | None:
    segments: list[SegmentPlan] = []
    segment_start_rad = current_state["joint_rad"]
    segment_start_deg = current_state["joint_deg"]

    for index, waypoint in enumerate(waypoints, start=1):
        segment_delta_deg = waypoint.target_joint_deg - segment_start_deg
        target_raw_pos_rad = joint_to_raw_position(
            waypoint.target_joint_rad,
            raw_zero_pos_rad,
            direction_sign,
        )

        if abs(waypoint.target_joint_deg) > MAX_SAFETY_TARGET_DEG:
            print("Trajectory blocked due to safety limit constraints.")
            print("No trajectory commands were sent.")
            return None

        if abs(segment_delta_deg) > MAX_SAFETY_SEGMENT_DELTA_DEG:
            print("Trajectory blocked due to safety limit constraints.")
            print("No trajectory commands were sent.")
            return None

        if not command_in_range(target_raw_pos_rad, 0.0):
            print("Trajectory blocked due to safety limit constraints.")
            print("No trajectory commands were sent.")
            return None

        segments.append(
            SegmentPlan(
                index=index,
                start_joint_rad=segment_start_rad,
                start_joint_deg=segment_start_deg,
                target_joint_rad=waypoint.target_joint_rad,
                target_joint_deg=waypoint.target_joint_deg,
                duration_sec=waypoint.duration_sec,
                cycles=max(1, int(round(waypoint.duration_sec * rate_hz))),
            )
        )
        segment_start_rad = waypoint.target_joint_rad
        segment_start_deg = waypoint.target_joint_deg

    return TrajectoryPlan(
        current_joint_rad=current_state["joint_rad"],
        current_joint_deg=current_state["joint_deg"],
        raw_zero_pos_rad=raw_zero_pos_rad,
        direction_sign=direction_sign,
        segments=segments,
        final_hold_cycles=int(round(final_hold_sec * rate_hz)),
    )


def print_trajectory_plan(
    motor_id: int,
    plan: TrajectoryPlan,
    kp: float,
    kd: float,
    rate_hz: float,
    final_hold_sec: float,
) -> None:
    print("[Joint Trajectory Plan]")
    print(f"ID: {format_motor_id(motor_id)}")
    print(f"Current joint: {plan.current_joint_deg:+.2f} deg")
    print(f"direction_sign: {plan.direction_sign}")
    print(f"kp: {kp}")
    print(f"kd: {kd}")
    print(f"rate_hz: {rate_hz:.1f}")
    print(f"final_hold_sec: {final_hold_sec:.1f}")
    print("")

    for segment in plan.segments:
        print(f"Segment {segment.index}:")
        print(f"  start: {segment.start_joint_deg:+.2f} deg")
        print(f"  target: {segment.target_joint_deg:+.2f} deg")
        print(f"  delta: {segment.delta_joint_deg:+.2f} deg")
        print(f"  duration: {segment.duration_sec:.3f} sec")
        print(f"  cycles: {segment.cycles}")
        print("")

    print(f"Final hold cycles: {plan.final_hold_cycles}")
    print(f"Total trajectory cycles: {plan.total_trajectory_cycles}")


def handle_cycle_feedback(
    bus: can.BusABC,
    motor_id: int,
    command_packet: bytes,
    feedback_timeout_sec: float,
    raw_zero_pos_rad: float,
    direction_sign: int,
    command_joint_pos_rad: float,
    max_following_error_deg: float,
    stats: TrajectoryStats,
) -> dict[str, float] | None:
    feedback = receive_valid_feedback(
        bus=bus,
        motor_id=motor_id,
        echo_packet=command_packet,
        timeout_sec=feedback_timeout_sec,
    )
    if feedback is None:
        stats.feedback_missed_count += 1
        stats.consecutive_feedback_misses += 1
        stats.maximum_consecutive_feedback_misses = max(
            stats.maximum_consecutive_feedback_misses,
            stats.consecutive_feedback_misses,
        )
        return None

    actual_joint_rad = raw_to_joint_position(
        feedback["raw_pos_rad"],
        raw_zero_pos_rad,
        direction_sign,
    )
    actual_joint_deg = math.degrees(actual_joint_rad)
    following_error_deg = math.degrees(command_joint_pos_rad - actual_joint_rad)

    stats.feedback_received_count += 1
    stats.consecutive_feedback_misses = 0
    stats.last_actual_joint_deg = actual_joint_deg
    stats.maximum_abs_following_error_deg = max(
        stats.maximum_abs_following_error_deg,
        abs(following_error_deg),
    )

    if abs(following_error_deg) > max_following_error_deg:
        stats.consecutive_large_error_cycles += 1
        stats.maximum_consecutive_large_error_cycles = max(
            stats.maximum_consecutive_large_error_cycles,
            stats.consecutive_large_error_cycles,
        )
    else:
        stats.consecutive_large_error_cycles = 0

    return {
        "actual_joint_rad": actual_joint_rad,
        "actual_joint_deg": actual_joint_deg,
        "following_error_deg": following_error_deg,
    }


def check_abort_conditions(stats: TrajectoryStats) -> bool:
    if stats.consecutive_feedback_misses >= MAX_CONSECUTIVE_FEEDBACK_MISSES:
        print("Trajectory aborted: too many consecutive feedback misses.")
        print("Releasing torque.")
        return False

    if stats.consecutive_large_error_cycles >= MAX_CONSECUTIVE_LARGE_ERROR_CYCLES:
        print("Trajectory aborted: following error remained too large.")
        print("Releasing torque.")
        return False

    return True


def print_segment_feedback(
    segment_number: int,
    segment_count: int,
    cycle_index: int,
    segment_cycles: int,
    command_joint_pos_rad: float,
    command_joint_vel_rad_s: float,
    feedback_state: dict[str, float] | None,
    stats: TrajectoryStats,
) -> None:
    print(f"[Segment {segment_number}/{segment_count} | {cycle_index}/{segment_cycles}]")
    print(f"command position: {math.degrees(command_joint_pos_rad):+.2f} deg")
    print(f"command velocity: {math.degrees(command_joint_vel_rad_s):+.2f} deg/s")
    if feedback_state is None:
        print(
            f"feedback miss: "
            f"{stats.consecutive_feedback_misses}/{MAX_CONSECUTIVE_FEEDBACK_MISSES}"
        )
        return

    print(f"actual position: {feedback_state['actual_joint_deg']:+.2f} deg")
    print(f"following error: {feedback_state['following_error_deg']:+.2f} deg")


def print_final_hold_feedback(
    hold_index: int,
    hold_cycles: int,
    target_joint_deg: float,
    feedback_state: dict[str, float] | None,
    stats: TrajectoryStats,
) -> None:
    print(f"[Final Hold {hold_index}/{hold_cycles}]")
    print(f"target: {target_joint_deg:+.2f} deg")
    if feedback_state is None:
        print(
            f"feedback miss: "
            f"{stats.consecutive_feedback_misses}/{MAX_CONSECUTIVE_FEEDBACK_MISSES}"
        )
        return

    print(f"actual: {feedback_state['actual_joint_deg']:+.2f} deg")
    print(f"error: {feedback_state['following_error_deg']:+.2f} deg")


def send_trajectory_command(
    bus: can.BusABC,
    motor_id: int,
    raw_zero_pos_rad: float,
    direction_sign: int,
    command_joint_pos_rad: float,
    command_joint_vel_rad_s: float,
    kp: float,
    kd: float,
) -> bytes | None:
    command_raw_pos_rad = joint_to_raw_position(
        command_joint_pos_rad,
        raw_zero_pos_rad,
        direction_sign,
    )
    command_raw_vel_rad_s = joint_to_raw_velocity(command_joint_vel_rad_s, direction_sign)

    if not command_in_range(command_raw_pos_rad, command_raw_vel_rad_s):
        print("Trajectory aborted: command exceeds AK70 position or velocity range.")
        print("Releasing torque.")
        return None

    command_packet = pack_mit_command(command_raw_pos_rad, command_raw_vel_rad_s, kp, kd, 0.0)
    msg = can.Message(arbitration_id=motor_id, data=command_packet, is_extended_id=False)
    bus.send(msg)
    return command_packet


def run_segment(
    bus: can.BusABC,
    motor_id: int,
    plan: TrajectoryPlan,
    segment: SegmentPlan,
    segment_count: int,
    kp: float,
    kd: float,
    rate_hz: float,
    feedback_print_hz: float,
    max_following_error_deg: float,
    stats: TrajectoryStats,
    next_send_time: float,
) -> tuple[int, float]:
    period_sec = 1.0 / rate_hz
    feedback_timeout_sec = min(0.005, period_sec * 0.25)
    print_interval_cycles = max(1, int(round(rate_hz / feedback_print_hz)))

    for cycle_index in range(1, segment.cycles + 1):
        u = cycle_index / segment.cycles
        s, ds_du = smoothstep_quintic(u)
        command_joint_pos_rad = segment.start_joint_rad + segment.delta_joint_rad * s
        command_joint_vel_rad_s = segment.delta_joint_rad / segment.duration_sec * ds_du

        if cycle_index == segment.cycles:
            command_joint_pos_rad = segment.target_joint_rad
            command_joint_vel_rad_s = 0.0

        wait_until_send_time(next_send_time)
        command_packet = send_trajectory_command(
            bus=bus,
            motor_id=motor_id,
            raw_zero_pos_rad=plan.raw_zero_pos_rad,
            direction_sign=plan.direction_sign,
            command_joint_pos_rad=command_joint_pos_rad,
            command_joint_vel_rad_s=command_joint_vel_rad_s,
            kp=kp,
            kd=kd,
        )
        if command_packet is None:
            return 2, next_send_time

        stats.trajectory_commands_sent += 1
        feedback_state = handle_cycle_feedback(
            bus=bus,
            motor_id=motor_id,
            command_packet=command_packet,
            feedback_timeout_sec=feedback_timeout_sec,
            raw_zero_pos_rad=plan.raw_zero_pos_rad,
            direction_sign=plan.direction_sign,
            command_joint_pos_rad=command_joint_pos_rad,
            max_following_error_deg=max_following_error_deg,
            stats=stats,
        )

        should_print = (
            cycle_index == 1
            or cycle_index == segment.cycles
            or stats.trajectory_commands_sent % print_interval_cycles == 0
        )
        if should_print:
            print_segment_feedback(
                segment_number=segment.index,
                segment_count=segment_count,
                cycle_index=cycle_index,
                segment_cycles=segment.cycles,
                command_joint_pos_rad=command_joint_pos_rad,
                command_joint_vel_rad_s=command_joint_vel_rad_s,
                feedback_state=feedback_state,
                stats=stats,
            )

        if not check_abort_conditions(stats):
            return 2, next_send_time

        next_send_time = update_next_send_time(next_send_time, period_sec, stats)

    return 0, next_send_time


def run_final_hold(
    bus: can.BusABC,
    motor_id: int,
    plan: TrajectoryPlan,
    kp: float,
    kd: float,
    rate_hz: float,
    feedback_print_hz: float,
    max_following_error_deg: float,
    stats: TrajectoryStats,
    next_send_time: float,
) -> tuple[int, float]:
    if plan.final_hold_cycles <= 0:
        return 0, next_send_time

    period_sec = 1.0 / rate_hz
    feedback_timeout_sec = min(0.005, period_sec * 0.25)
    print_interval_cycles = max(1, int(round(rate_hz / feedback_print_hz)))
    target_joint_rad = plan.final_target_joint_rad
    target_joint_deg = plan.final_target_joint_deg

    for hold_index in range(1, plan.final_hold_cycles + 1):
        wait_until_send_time(next_send_time)
        command_packet = send_trajectory_command(
            bus=bus,
            motor_id=motor_id,
            raw_zero_pos_rad=plan.raw_zero_pos_rad,
            direction_sign=plan.direction_sign,
            command_joint_pos_rad=target_joint_rad,
            command_joint_vel_rad_s=0.0,
            kp=kp,
            kd=kd,
        )
        if command_packet is None:
            return 2, next_send_time

        stats.trajectory_commands_sent += 1
        feedback_state = handle_cycle_feedback(
            bus=bus,
            motor_id=motor_id,
            command_packet=command_packet,
            feedback_timeout_sec=feedback_timeout_sec,
            raw_zero_pos_rad=plan.raw_zero_pos_rad,
            direction_sign=plan.direction_sign,
            command_joint_pos_rad=target_joint_rad,
            max_following_error_deg=max_following_error_deg,
            stats=stats,
        )

        should_print = (
            hold_index == 1
            or hold_index == plan.final_hold_cycles
            or stats.trajectory_commands_sent % print_interval_cycles == 0
        )
        if should_print:
            print_final_hold_feedback(
                hold_index=hold_index,
                hold_cycles=plan.final_hold_cycles,
                target_joint_deg=target_joint_deg,
                feedback_state=feedback_state,
                stats=stats,
            )

        if not check_abort_conditions(stats):
            return 2, next_send_time

        next_send_time = update_next_send_time(next_send_time, period_sec, stats)

    return 0, next_send_time


def run_trajectory(
    bus: can.BusABC,
    motor_id: int,
    plan: TrajectoryPlan,
    kp: float,
    kd: float,
    rate_hz: float,
    feedback_print_hz: float,
    max_following_error_deg: float,
    stats: TrajectoryStats,
) -> int:
    next_send_time = time.monotonic()
    segment_count = len(plan.segments)

    for segment in plan.segments:
        result, next_send_time = run_segment(
            bus=bus,
            motor_id=motor_id,
            plan=plan,
            segment=segment,
            segment_count=segment_count,
            kp=kp,
            kd=kd,
            rate_hz=rate_hz,
            feedback_print_hz=feedback_print_hz,
            max_following_error_deg=max_following_error_deg,
            stats=stats,
            next_send_time=next_send_time,
        )
        if result != 0:
            return result

    result, _next_send_time = run_final_hold(
        bus=bus,
        motor_id=motor_id,
        plan=plan,
        kp=kp,
        kd=kd,
        rate_hz=rate_hz,
        feedback_print_hz=feedback_print_hz,
        max_following_error_deg=max_following_error_deg,
        stats=stats,
        next_send_time=next_send_time,
    )
    return result


def print_final_report(motor_id: int, plan: TrajectoryPlan, stats: TrajectoryStats) -> None:
    print("[Trajectory End State]")
    print(f"ID: {format_motor_id(motor_id)}")
    print(f"Final target: {plan.final_target_joint_deg:+.2f} deg")
    if stats.last_actual_joint_deg is None:
        print("Final actual position: unavailable")
        print("Final error: unavailable")
    else:
        final_error_deg = plan.final_target_joint_deg - stats.last_actual_joint_deg
        print(f"Final actual position: {stats.last_actual_joint_deg:+.2f} deg")
        print(f"Final error: {final_error_deg:+.2f} deg")
    print(f"Trajectory commands sent: {stats.trajectory_commands_sent}")
    print(f"Feedback received: {stats.feedback_received_count}")
    print(f"Feedback missed: {stats.feedback_missed_count}")
    print(f"Maximum consecutive misses: {stats.maximum_consecutive_feedback_misses}")
    print(f"Maximum absolute following error: {stats.maximum_abs_following_error_deg:.2f} deg")
    print(f"Maximum consecutive large-error cycles: {stats.maximum_consecutive_large_error_cycles}")
    print(f"Timing overruns: {stats.timing_overruns}")


def run_once(
    channel: str,
    motor_id: int,
    waypoints: list[Waypoint],
    kp: float,
    kd: float,
    final_hold_sec: float,
    rate_hz: float,
    feedback_print_hz: float,
    max_following_error_deg: float,
    enter_mit: bool,
) -> int:
    print_safety_warning(
        channel=channel,
        motor_id=motor_id,
        waypoints=waypoints,
        kp=kp,
        kd=kd,
        final_hold_sec=final_hold_sec,
        rate_hz=rate_hz,
        feedback_print_hz=feedback_print_hz,
        max_following_error_deg=max_following_error_deg,
        enter_mit=enter_mit,
    )

    if not validate_arguments(
        motor_id=motor_id,
        waypoints=waypoints,
        kp=kp,
        kd=kd,
        final_hold_sec=final_hold_sec,
        rate_hz=rate_hz,
        feedback_print_hz=feedback_print_hz,
        max_following_error_deg=max_following_error_deg,
    ):
        return 1

    try:
        calibration = load_motor_calibration()
    except Exception as exc:
        print(f"Error: failed to load motor_calibration.yaml: {exc}")
        return 2

    motor_calibration = get_motor_calibration(motor_id, calibration)
    if motor_calibration is None:
        return 2
    raw_zero_pos_rad, direction_sign = motor_calibration

    zero_torque_packet = pack_mit_command(0.0, 0.0, 0.0, 0.0, 0.0)
    if not validate_zero_torque_packet(zero_torque_packet):
        return 2

    for waypoint in waypoints:
        raw_pos_rad = joint_to_raw_position(
            waypoint.target_joint_rad,
            raw_zero_pos_rad,
            direction_sign,
        )
        if not command_in_range(raw_pos_rad, 0.0):
            print("Trajectory blocked due to safety limit constraints.")
            print("No trajectory commands were sent.")
            return 2

    if not confirm_yes():
        print("Cancelled.")
        print("No CAN bus was opened.")
        print("No trajectory commands were sent.")
        return 0

    bus = None
    torque_released = False
    stats = TrajectoryStats()
    try:
        bus = can.Bus(interface=DEFAULT_INTERFACE, channel=channel)
        current_state = read_current_joint_once(
            bus=bus,
            motor_id=motor_id,
            raw_zero_pos_rad=raw_zero_pos_rad,
            direction_sign=direction_sign,
            zero_torque_packet=zero_torque_packet,
        )
        if current_state is None:
            print(f"Error: no valid feedback received from {format_motor_id(motor_id)}.")
            print("No trajectory commands were sent.")
            return 2

        plan = build_trajectory_plan(
            current_state=current_state,
            waypoints=waypoints,
            raw_zero_pos_rad=raw_zero_pos_rad,
            direction_sign=direction_sign,
            final_hold_sec=final_hold_sec,
            rate_hz=rate_hz,
        )
        if plan is None:
            return 2

        print_trajectory_plan(
            motor_id=motor_id,
            plan=plan,
            kp=kp,
            kd=kd,
            rate_hz=rate_hz,
            final_hold_sec=final_hold_sec,
        )

        if not confirm_trajectory():
            print("Trajectory cancelled.")
            print("No trajectory commands were sent.")
            return 0

        if enter_mit:
            enter_msg = can.Message(arbitration_id=motor_id, data=MIT_ENTER_PACKET, is_extended_id=False)
            bus.send(enter_msg)
            time.sleep(MIT_ENTER_SLEEP_SEC)

        result = run_trajectory(
            bus=bus,
            motor_id=motor_id,
            plan=plan,
            kp=kp,
            kd=kd,
            rate_hz=rate_hz,
            feedback_print_hz=feedback_print_hz,
            max_following_error_deg=max_following_error_deg,
            stats=stats,
        )
        if result != 0:
            return result

        print("Trajectory completed.")
        release_joint(bus, motor_id, zero_torque_packet)
        torque_released = True
        print_final_report(motor_id, plan, stats)
        return 0
    except KeyboardInterrupt:
        print("Trajectory interrupted by user.")
        print("Releasing torque.")
        return 1
    except can.CanError:
        print("CAN error during trajectory.")
        print("Releasing torque.")
        return 1
    except Exception as exc:
        print(f"Unexpected error during trajectory: {exc}")
        print("Releasing torque.")
        return 1
    finally:
        if bus is not None:
            if not torque_released:
                send_zero_torque_best_effort(bus, motor_id, zero_torque_packet)
                time.sleep(RELEASE_SETTLE_SEC)
            bus.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run a finite AK70-10 single-joint waypoint trajectory."
    )
    parser.add_argument("--channel", default=DEFAULT_CHANNEL)
    parser.add_argument("--motor-id", type=parse_motor_id, required=True)

    waypoint_group = parser.add_mutually_exclusive_group(required=True)
    waypoint_group.add_argument("--waypoint-deg", action="append", default=None)
    waypoint_group.add_argument("--waypoint-rad", action="append", default=None)

    parser.add_argument("--kp", type=float, default=DEFAULT_KP)
    parser.add_argument("--kd", type=float, default=DEFAULT_KD)
    parser.add_argument("--final-hold-sec", type=float, default=DEFAULT_FINAL_HOLD_SEC)
    parser.add_argument("--rate-hz", type=float, default=DEFAULT_RATE_HZ)
    parser.add_argument("--feedback-print-hz", type=float, default=DEFAULT_FEEDBACK_PRINT_HZ)
    parser.add_argument("--max-following-error-deg", type=float, default=DEFAULT_MAX_FOLLOWING_ERROR_DEG)
    parser.add_argument("--enter-mit", action="store_true")
    args = parser.parse_args()

    waypoint_unit = "deg" if args.waypoint_deg is not None else "rad"
    waypoint_values = args.waypoint_deg if args.waypoint_deg is not None else args.waypoint_rad
    waypoints = parse_waypoints(waypoint_values, waypoint_unit)

    raise SystemExit(
        run_once(
            channel=args.channel,
            motor_id=args.motor_id,
            waypoints=waypoints,
            kp=args.kp,
            kd=args.kd,
            final_hold_sec=args.final_hold_sec,
            rate_hz=args.rate_hz,
            feedback_print_hz=args.feedback_print_hz,
            max_following_error_deg=args.max_following_error_deg,
            enter_mit=args.enter_mit,
        )
    )


if __name__ == "__main__":
    main()
