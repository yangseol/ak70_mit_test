"""Finite-duration AK70-10 joint ramp and position hold helper."""

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
DEFAULT_KD = 0.1
DEFAULT_MOVE_SEC = 1.0
DEFAULT_HOLD_SEC = 3.0
DEFAULT_RATE_HZ = 50.0
DEFAULT_FEEDBACK_PRINT_HZ = 5.0
DRAIN_DURATION_SEC = 0.05
DRAIN_POLL_TIMEOUT_SEC = 0.005
INITIAL_FEEDBACK_TIMEOUT_SEC = 1.0
INITIAL_RECV_POLL_TIMEOUT_SEC = 0.05
MIT_ENTER_SLEEP_SEC = 0.02
RELEASE_SETTLE_SEC = 0.03

MAX_SAFETY_TARGET_DEG = 120.0
MAX_SAFETY_DELTA_DEG = 120.0
MAX_CONSECUTIVE_FEEDBACK_MISSES = 10

EXPECTED_ZERO_TORQUE_PACKET = bytes([0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x00])
MIT_ENTER_PACKET = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])


@dataclass
class HoldStats:
    position_commands_sent: int = 0
    feedback_received_count: int = 0
    feedback_missed_count: int = 0
    consecutive_feedback_misses: int = 0
    maximum_consecutive_feedback_misses: int = 0
    timing_overruns: int = 0
    last_joint_deg: float | None = None


@dataclass(frozen=True)
class HoldPlan:
    current_joint_rad: float
    current_joint_deg: float
    target_joint_rad: float
    target_joint_deg: float
    delta_joint_rad: float
    delta_joint_deg: float
    raw_zero_pos_rad: float
    target_raw_pos_rad: float
    direction_sign: int
    ramp_cycles: int
    hold_cycles: int

    @property
    def total_command_cycles(self) -> int:
        return self.ramp_cycles + self.hold_cycles


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


def normalize_target(target_deg: float | None, target_rad: float | None) -> tuple[float, float]:
    if target_deg is not None:
        target_joint_deg = float(target_deg)
        target_joint_rad = math.radians(target_joint_deg)
        return target_joint_rad, target_joint_deg

    if target_rad is None:
        raise ValueError("target angle is required")

    target_joint_rad = float(target_rad)
    target_joint_deg = math.degrees(target_joint_rad)
    return target_joint_rad, target_joint_deg


def validate_arguments(
    motor_id: int,
    target_joint_rad: float,
    target_joint_deg: float,
    kp: float,
    kd: float,
    move_sec: float,
    hold_sec: float,
    rate_hz: float,
    feedback_print_hz: float,
) -> bool:
    errors: list[str] = []

    if not (0x001 <= motor_id <= 0x00A):
        errors.append("Error: motor-id must be in AK70 range 0x001~0x00A")

    float_values = {
        "target_rad": target_joint_rad,
        "target_deg": target_joint_deg,
        "kp": kp,
        "kd": kd,
        "move_sec": move_sec,
        "hold_sec": hold_sec,
        "rate_hz": rate_hz,
        "feedback_print_hz": feedback_print_hz,
    }
    for name, value in float_values.items():
        if not math.isfinite(value):
            errors.append(f"Invalid {name}: must be finite")

    if not 0.0 < kp <= 10.0:
        errors.append("Invalid kp: must be > 0.0 and <= 10.0")
    if not 0.0 <= kd <= 2.0:
        errors.append("Invalid kd: must be >= 0.0 and <= 2.0")
    if not 0.1 <= move_sec <= 10.0:
        errors.append("Invalid move-sec: must be >= 0.1 and <= 10.0")
    if not 0.1 <= hold_sec <= 30.0:
        errors.append("Invalid hold-sec: must be >= 0.1 and <= 30.0")
    if not 10.0 <= rate_hz <= 200.0:
        errors.append("Invalid rate-hz: must be >= 10.0 and <= 200.0")
    if not 0.5 <= feedback_print_hz <= rate_hz:
        errors.append("Invalid feedback-print-hz: must be >= 0.5 and <= rate-hz")

    for error in errors:
        print(error)
    return not errors


def print_safety_warning(
    channel: str,
    motor_id: int,
    target_joint_deg: float,
    target_joint_rad: float,
    kp: float,
    kd: float,
    move_sec: float,
    hold_sec: float,
    rate_hz: float,
    feedback_print_hz: float,
    enter_mit: bool,
) -> None:
    print("SAFETY WARNING")
    print("- AK70 finite-duration joint position hold")
    print("- one motor only")
    print("- periodic position commands")
    print("- finite move and hold duration")
    print("- no 0xFE")
    print("- no calibration write")
    print("- Ctrl+C releases torque")
    print(f"channel: {channel}")
    print(f"motor_id: {format_motor_id(motor_id)}")
    print(f"target_deg: {target_joint_deg:+.2f}")
    print(f"target_rad: {target_joint_rad:+.6f}")
    print(f"kp: {kp}")
    print(f"kd: {kd}")
    print(f"move_sec: {move_sec:.3f}")
    print(f"hold_sec: {hold_sec:.3f}")
    print(f"rate_hz: {rate_hz:.1f}")
    print(f"feedback_print_hz: {feedback_print_hz:.1f}")
    print(f"enter_mit: {enter_mit}")


def confirm_yes() -> bool:
    return input("Type YES to continue: ").strip() == "YES"


def confirm_hold() -> bool:
    return input("Type HOLD to start finite joint position hold: ").strip() == "HOLD"


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


def joint_to_raw_position(joint_rad: float, raw_zero_pos_rad: float, direction_sign: int) -> float:
    return raw_zero_pos_rad + direction_sign * joint_rad


def raw_position_in_range(raw_pos_rad: float) -> bool:
    return AK70_10_LIMIT.p_min <= raw_pos_rad <= AK70_10_LIMIT.p_max


def open_can_bus(channel: str) -> can.BusABC:
    return can.Bus(interface=DEFAULT_INTERFACE, channel=channel)


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

        msg = bus.recv(timeout=min(INITIAL_RECV_POLL_TIMEOUT_SEC, remaining))
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


def wait_until_send_time(next_send_time: float) -> None:
    remaining = next_send_time - time.monotonic()
    if remaining > 0.0:
        time.sleep(remaining)


def update_next_send_time(next_send_time: float, period_sec: float, stats: HoldStats) -> float:
    scheduled_next = next_send_time + period_sec
    now = time.monotonic()

    if now >= scheduled_next:
        stats.timing_overruns += 1
        return now + period_sec

    return scheduled_next


def build_hold_plan(
    current_state: dict[str, float],
    target_joint_rad: float,
    target_joint_deg: float,
    raw_zero_pos_rad: float,
    direction_sign: int,
    move_sec: float,
    hold_sec: float,
    rate_hz: float,
) -> HoldPlan | None:
    current_joint_rad = current_state["joint_rad"]
    current_joint_deg = current_state["joint_deg"]
    delta_joint_rad = target_joint_rad - current_joint_rad
    delta_joint_deg = target_joint_deg - current_joint_deg
    target_raw_pos_rad = joint_to_raw_position(target_joint_rad, raw_zero_pos_rad, direction_sign)

    if abs(target_joint_deg) > MAX_SAFETY_TARGET_DEG:
        print("Movement blocked due to safety limit constraints.")
        print("No position commands were sent.")
        return None

    if abs(delta_joint_deg) > MAX_SAFETY_DELTA_DEG:
        print("Movement blocked due to safety limit constraints.")
        print("No position commands were sent.")
        return None

    if not raw_position_in_range(target_raw_pos_rad):
        print("Movement blocked: target raw position is outside AK70 command range.")
        print("No position commands were sent.")
        return None

    return HoldPlan(
        current_joint_rad=current_joint_rad,
        current_joint_deg=current_joint_deg,
        target_joint_rad=target_joint_rad,
        target_joint_deg=target_joint_deg,
        delta_joint_rad=delta_joint_rad,
        delta_joint_deg=delta_joint_deg,
        raw_zero_pos_rad=raw_zero_pos_rad,
        target_raw_pos_rad=target_raw_pos_rad,
        direction_sign=direction_sign,
        ramp_cycles=max(1, int(round(move_sec * rate_hz))),
        hold_cycles=max(1, int(round(hold_sec * rate_hz))),
    )


def print_hold_plan(
    motor_id: int,
    plan: HoldPlan,
    kp: float,
    kd: float,
    move_sec: float,
    hold_sec: float,
    rate_hz: float,
    feedback_print_hz: float,
) -> None:
    print("[Joint Hold Plan]")
    print(f"ID: {format_motor_id(motor_id)}")
    print(f"Current joint: {plan.current_joint_deg:+.2f} deg")
    print(f"Target joint: {plan.target_joint_deg:+.2f} deg")
    print(f"Delta: {plan.delta_joint_deg:+.2f} deg")
    print(f"Raw zero: {plan.raw_zero_pos_rad:.6f} rad")
    print(f"Target raw: {plan.target_raw_pos_rad:.6f} rad")
    print(f"direction_sign: {plan.direction_sign}")
    print(f"kp: {kp}")
    print(f"kd: {kd}")
    print(f"move_sec: {move_sec:.3f}")
    print(f"hold_sec: {hold_sec:.3f}")
    print(f"rate_hz: {rate_hz:.1f}")
    print(f"feedback_print_hz: {feedback_print_hz:.1f}")
    print(f"ramp cycles: {plan.ramp_cycles}")
    print(f"hold cycles: {plan.hold_cycles}")
    print(f"total command cycles: {plan.total_command_cycles}")


def handle_cycle_feedback(
    bus: can.BusABC,
    motor_id: int,
    command_packet: bytes,
    feedback_timeout_sec: float,
    raw_zero_pos_rad: float,
    direction_sign: int,
    target_joint_deg: float,
    stats: HoldStats,
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

    raw_pos_rad = feedback["raw_pos_rad"]
    joint_rad = raw_to_joint_position(raw_pos_rad, raw_zero_pos_rad, direction_sign)
    joint_deg = math.degrees(joint_rad)
    stats.feedback_received_count += 1
    stats.consecutive_feedback_misses = 0
    stats.last_joint_deg = joint_deg
    return {
        "raw_pos_rad": raw_pos_rad,
        "joint_rad": joint_rad,
        "joint_deg": joint_deg,
        "error_deg": target_joint_deg - joint_deg,
    }


def print_move_feedback(
    cycle_index: int,
    ramp_cycles: int,
    feedback_state: dict[str, float] | None,
    ramp_target_deg: float,
    final_target_deg: float,
    stats: HoldStats,
) -> None:
    if feedback_state is None:
        print(
            f"[Move {cycle_index}/{ramp_cycles}] "
            f"feedback miss {stats.consecutive_feedback_misses}/{MAX_CONSECUTIVE_FEEDBACK_MISSES}"
        )
        return

    print(
        f"[Move {cycle_index}/{ramp_cycles}] "
        f"joint: {feedback_state['joint_deg']:+.2f} deg | "
        f"ramp target: {ramp_target_deg:+.2f} deg | "
        f"final target: {final_target_deg:+.2f} deg | "
        f"final error: {feedback_state['error_deg']:+.2f} deg"
    )


def print_hold_feedback(
    hold_index: int,
    hold_cycles: int,
    feedback_state: dict[str, float] | None,
    target_joint_deg: float,
    stats: HoldStats,
) -> None:
    if feedback_state is None:
        print(
            f"[Hold {hold_index}/{hold_cycles}] "
            f"feedback miss {stats.consecutive_feedback_misses}/{MAX_CONSECUTIVE_FEEDBACK_MISSES}"
        )
        return

    print(
        f"[Hold {hold_index}/{hold_cycles}] "
        f"joint: {feedback_state['joint_deg']:+.2f} deg | "
        f"target: {target_joint_deg:+.2f} deg | "
        f"error: {feedback_state['error_deg']:+.2f} deg"
    )


def check_feedback_miss_limit(stats: HoldStats) -> bool:
    if stats.consecutive_feedback_misses >= MAX_CONSECUTIVE_FEEDBACK_MISSES:
        print("Hold aborted: too many consecutive feedback misses.")
        print("Releasing torque.")
        return False
    return True


def run_ramp_phase(
    bus: can.BusABC,
    motor_id: int,
    plan: HoldPlan,
    kp: float,
    kd: float,
    rate_hz: float,
    feedback_print_hz: float,
    stats: HoldStats,
    next_send_time: float,
) -> tuple[int, float]:
    period_sec = 1.0 / rate_hz
    feedback_timeout_sec = min(0.005, period_sec * 0.25)
    print_interval_cycles = max(1, int(round(rate_hz / feedback_print_hz)))
    total_cycle_index = 0

    for cycle_index in range(1, plan.ramp_cycles + 1):
        alpha = cycle_index / plan.ramp_cycles
        ramp_joint_rad = plan.current_joint_rad + alpha * plan.delta_joint_rad
        ramp_joint_deg = math.degrees(ramp_joint_rad)
        ramp_raw_pos_rad = joint_to_raw_position(
            ramp_joint_rad,
            plan.raw_zero_pos_rad,
            plan.direction_sign,
        )

        if not raw_position_in_range(ramp_raw_pos_rad):
            print("Hold aborted: ramp raw target is outside AK70 command range.")
            print("Releasing torque.")
            return 2, next_send_time

        wait_until_send_time(next_send_time)
        command_packet = pack_mit_command(ramp_raw_pos_rad, 0.0, kp, kd, 0.0)
        msg = can.Message(arbitration_id=motor_id, data=command_packet, is_extended_id=False)
        bus.send(msg)
        stats.position_commands_sent += 1
        total_cycle_index = stats.position_commands_sent

        feedback_state = handle_cycle_feedback(
            bus=bus,
            motor_id=motor_id,
            command_packet=command_packet,
            feedback_timeout_sec=feedback_timeout_sec,
            raw_zero_pos_rad=plan.raw_zero_pos_rad,
            direction_sign=plan.direction_sign,
            target_joint_deg=plan.target_joint_deg,
            stats=stats,
        )

        should_print = total_cycle_index == 1 or total_cycle_index % print_interval_cycles == 0
        if cycle_index == plan.ramp_cycles:
            should_print = True
        if should_print:
            print_move_feedback(
                cycle_index=cycle_index,
                ramp_cycles=plan.ramp_cycles,
                feedback_state=feedback_state,
                ramp_target_deg=ramp_joint_deg,
                final_target_deg=plan.target_joint_deg,
                stats=stats,
            )

        if not check_feedback_miss_limit(stats):
            return 2, next_send_time

        next_send_time = update_next_send_time(next_send_time, period_sec, stats)

    return 0, next_send_time


def run_hold_phase(
    bus: can.BusABC,
    motor_id: int,
    plan: HoldPlan,
    kp: float,
    kd: float,
    rate_hz: float,
    feedback_print_hz: float,
    stats: HoldStats,
    next_send_time: float,
) -> tuple[int, float]:
    period_sec = 1.0 / rate_hz
    feedback_timeout_sec = min(0.005, period_sec * 0.25)
    print_interval_cycles = max(1, int(round(rate_hz / feedback_print_hz)))
    command_packet = pack_mit_command(plan.target_raw_pos_rad, 0.0, kp, kd, 0.0)
    msg = can.Message(arbitration_id=motor_id, data=command_packet, is_extended_id=False)

    for hold_index in range(1, plan.hold_cycles + 1):
        if not raw_position_in_range(plan.target_raw_pos_rad):
            print("Hold aborted: ramp raw target is outside AK70 command range.")
            print("Releasing torque.")
            return 2, next_send_time

        wait_until_send_time(next_send_time)
        bus.send(msg)
        stats.position_commands_sent += 1
        total_cycle_index = stats.position_commands_sent

        feedback_state = handle_cycle_feedback(
            bus=bus,
            motor_id=motor_id,
            command_packet=command_packet,
            feedback_timeout_sec=feedback_timeout_sec,
            raw_zero_pos_rad=plan.raw_zero_pos_rad,
            direction_sign=plan.direction_sign,
            target_joint_deg=plan.target_joint_deg,
            stats=stats,
        )

        should_print = total_cycle_index == 1 or total_cycle_index % print_interval_cycles == 0
        if hold_index == plan.hold_cycles:
            should_print = True
        if should_print:
            print_hold_feedback(
                hold_index=hold_index,
                hold_cycles=plan.hold_cycles,
                feedback_state=feedback_state,
                target_joint_deg=plan.target_joint_deg,
                stats=stats,
            )

        if not check_feedback_miss_limit(stats):
            return 2, next_send_time

        next_send_time = update_next_send_time(next_send_time, period_sec, stats)

    return 0, next_send_time


def print_final_report(motor_id: int, target_joint_deg: float, stats: HoldStats) -> None:
    print("[Hold End State]")
    print(f"ID: {format_motor_id(motor_id)}")
    print(f"Target: {target_joint_deg:+.2f} deg")
    if stats.last_joint_deg is None:
        print("Held position: unavailable")
        print("Hold error: unavailable")
    else:
        hold_error_deg = target_joint_deg - stats.last_joint_deg
        print(f"Held position: {stats.last_joint_deg:+.2f} deg")
        print(f"Hold error: {hold_error_deg:+.2f} deg")
    print(f"Position commands sent: {stats.position_commands_sent}")
    print(f"Feedback received: {stats.feedback_received_count}")
    print(f"Feedback missed: {stats.feedback_missed_count}")
    print(f"Maximum consecutive misses: {stats.maximum_consecutive_feedback_misses}")
    print(f"Timing overruns: {stats.timing_overruns}")


def release_joint(
    bus: can.BusABC,
    motor_id: int,
    zero_torque_packet: bytes,
) -> bool:
    sent = send_zero_torque_best_effort(bus, motor_id, zero_torque_packet)
    if sent:
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

    return sent


def run_once(
    channel: str,
    motor_id: int,
    target_joint_rad: float,
    target_joint_deg: float,
    kp: float,
    kd: float,
    move_sec: float,
    hold_sec: float,
    rate_hz: float,
    feedback_print_hz: float,
    enter_mit: bool,
) -> int:
    print_safety_warning(
        channel=channel,
        motor_id=motor_id,
        target_joint_deg=target_joint_deg,
        target_joint_rad=target_joint_rad,
        kp=kp,
        kd=kd,
        move_sec=move_sec,
        hold_sec=hold_sec,
        rate_hz=rate_hz,
        feedback_print_hz=feedback_print_hz,
        enter_mit=enter_mit,
    )

    if not validate_arguments(
        motor_id=motor_id,
        target_joint_rad=target_joint_rad,
        target_joint_deg=target_joint_deg,
        kp=kp,
        kd=kd,
        move_sec=move_sec,
        hold_sec=hold_sec,
        rate_hz=rate_hz,
        feedback_print_hz=feedback_print_hz,
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

    if not confirm_yes():
        print("Cancelled.")
        print("No CAN bus was opened.")
        print("No position commands were sent.")
        return 0

    bus = None
    torque_released = False
    stats = HoldStats()
    try:
        bus = open_can_bus(channel)
        current_state = read_current_joint_once(
            bus=bus,
            motor_id=motor_id,
            raw_zero_pos_rad=raw_zero_pos_rad,
            direction_sign=direction_sign,
            zero_torque_packet=zero_torque_packet,
        )
        if current_state is None:
            print(f"Error: no valid feedback received from {format_motor_id(motor_id)}.")
            print("No position commands were sent.")
            return 2

        plan = build_hold_plan(
            current_state=current_state,
            target_joint_rad=target_joint_rad,
            target_joint_deg=target_joint_deg,
            raw_zero_pos_rad=raw_zero_pos_rad,
            direction_sign=direction_sign,
            move_sec=move_sec,
            hold_sec=hold_sec,
            rate_hz=rate_hz,
        )
        if plan is None:
            return 2

        print_hold_plan(
            motor_id=motor_id,
            plan=plan,
            kp=kp,
            kd=kd,
            move_sec=move_sec,
            hold_sec=hold_sec,
            rate_hz=rate_hz,
            feedback_print_hz=feedback_print_hz,
        )

        if not confirm_hold():
            print("Hold cancelled.")
            print("No position commands were sent.")
            return 0

        if enter_mit:
            enter_msg = can.Message(arbitration_id=motor_id, data=MIT_ENTER_PACKET, is_extended_id=False)
            bus.send(enter_msg)
            time.sleep(MIT_ENTER_SLEEP_SEC)

        next_send_time = time.monotonic()
        ramp_result, next_send_time = run_ramp_phase(
            bus=bus,
            motor_id=motor_id,
            plan=plan,
            kp=kp,
            kd=kd,
            rate_hz=rate_hz,
            feedback_print_hz=feedback_print_hz,
            stats=stats,
            next_send_time=next_send_time,
        )
        if ramp_result != 0:
            return ramp_result

        hold_result, next_send_time = run_hold_phase(
            bus=bus,
            motor_id=motor_id,
            plan=plan,
            kp=kp,
            kd=kd,
            rate_hz=rate_hz,
            feedback_print_hz=feedback_print_hz,
            stats=stats,
            next_send_time=next_send_time,
        )
        if hold_result != 0:
            return hold_result

        print("Hold duration completed.")
        release_joint(bus, motor_id, zero_torque_packet)
        torque_released = True
        print_final_report(motor_id, plan.target_joint_deg, stats)
        return 0
    except KeyboardInterrupt:
        print("Hold interrupted by user.")
        print("Releasing torque.")
        return 1
    except can.CanError:
        print("CAN error during hold.")
        print("Releasing torque.")
        return 1
    except Exception as exc:
        print(f"Unexpected error during hold: {exc}")
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
        description="Ramp one AK70-10 joint to a target and hold it for a finite duration."
    )
    parser.add_argument("--channel", default=DEFAULT_CHANNEL)
    parser.add_argument("--motor-id", type=parse_motor_id, required=True)

    target_group = parser.add_mutually_exclusive_group(required=True)
    target_group.add_argument("--target-deg", type=float)
    target_group.add_argument("--target-rad", type=float)

    parser.add_argument("--kp", type=float, default=DEFAULT_KP)
    parser.add_argument("--kd", type=float, default=DEFAULT_KD)
    parser.add_argument("--move-sec", type=float, default=DEFAULT_MOVE_SEC)
    parser.add_argument("--hold-sec", type=float, default=DEFAULT_HOLD_SEC)
    parser.add_argument("--rate-hz", type=float, default=DEFAULT_RATE_HZ)
    parser.add_argument("--feedback-print-hz", type=float, default=DEFAULT_FEEDBACK_PRINT_HZ)
    parser.add_argument("--enter-mit", action="store_true")
    args = parser.parse_args()

    target_joint_rad, target_joint_deg = normalize_target(args.target_deg, args.target_rad)
    raise SystemExit(
        run_once(
            channel=args.channel,
            motor_id=args.motor_id,
            target_joint_rad=target_joint_rad,
            target_joint_deg=target_joint_deg,
            kp=args.kp,
            kd=args.kd,
            move_sec=args.move_sec,
            hold_sec=args.hold_sec,
            rate_hz=args.rate_hz,
            feedback_print_hz=args.feedback_print_hz,
            enter_mit=args.enter_mit,
        )
    )


if __name__ == "__main__":
    main()
