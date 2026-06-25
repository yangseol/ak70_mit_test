"""Move one AK70-10 joint once to a software-zero-relative target angle."""

from __future__ import annotations

import argparse
import math
import time

import can

from calibration import load_motor_calibration
from mit_packet import AK70_10_LIMIT, analyze_feedback_candidate, pack_mit_command


DEFAULT_CHANNEL = "can0"
DEFAULT_INTERFACE = "socketcan"
DEFAULT_KP = 1.5
DEFAULT_KD = 0.2
DEFAULT_PULSES = 5
DEFAULT_INTERVAL_SEC = 0.02
RECEIVE_DEADLINE_SEC = 1.0
RECV_POLL_TIMEOUT_SEC = 0.05
DRAIN_DURATION_SEC = 0.05
DRAIN_POLL_TIMEOUT_SEC = 0.005
FINAL_SETTLING_SEC = 0.10
MIT_ENTER_SLEEP_SEC = 0.02

MAX_SAFETY_TARGET_DEG = 120.0
MAX_SAFETY_DELTA_DEG = 120.0
TARGET_TOLERANCE_DEG = 1.0

EXPECTED_ZERO_TORQUE_PACKET = bytes([0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x00])
MIT_ENTER_PACKET = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])


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


def is_finite(value: float) -> bool:
    return math.isfinite(value)


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


def validate_parameters(
    motor_id: int,
    target_joint_rad: float,
    target_joint_deg: float,
    kp: float,
    kd: float,
    pulses: int,
    interval_sec: float,
) -> bool:
    errors: list[str] = []

    if not (0x001 <= motor_id <= 0x00A):
        errors.append("Error: motor-id must be in AK70 range 0x001~0x00A")

    float_values = {
        "target_rad": target_joint_rad,
        "target_deg": target_joint_deg,
        "kp": kp,
        "kd": kd,
        "interval_sec": interval_sec,
    }
    for name, value in float_values.items():
        if not is_finite(value):
            errors.append(f"Invalid {name}: must be finite")

    if not 0.0 < kp <= 5.0:
        errors.append("Invalid kp: must be > 0.0 and <= 5.0")
    if not 0.0 <= kd <= 2.0:
        errors.append("Invalid kd: must be >= 0.0 and <= 2.0")
    if not 1 <= pulses <= 20:
        errors.append("Invalid pulses: must be >= 1 and <= 20")
    if interval_sec < 0.005:
        errors.append("Invalid interval-sec: must be >= 0.005")

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
    pulses: int,
    interval_sec: float,
    enter_mit: bool,
) -> None:
    print("SAFETY WARNING")
    print("- AK70 joint-space limited position move")
    print("- one motor only")
    print("- finite pulses only")
    print("- no 0xFE")
    print("- no calibration write")
    print("- no continuous hold loop")
    print(f"channel: {channel}")
    print(f"motor_id: {format_motor_id(motor_id)}")
    print(f"target_deg: {target_joint_deg:+.2f}")
    print(f"target_rad: {target_joint_rad:+.6f}")
    print(f"kp: {kp}")
    print(f"kd: {kd}")
    print(f"pulses: {pulses}")
    print(f"interval_sec: {interval_sec:.3f}")
    print(f"enter_mit: {enter_mit}")


def confirm_yes() -> bool:
    confirmation = input("Type YES to continue: ")
    return confirmation == "YES"


def confirm_move() -> bool:
    confirmation = input("Type MOVE to execute limited position command pulses: ")
    return confirmation == "MOVE"


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
        print(f"Error: Calibration not found for {motor_key}")
        return None

    try:
        raw_zero_pos_rad = float(motor_calibration["raw_zero_pos_rad"])
        direction_sign = int(motor_calibration["direction_sign"])
    except (KeyError, TypeError, ValueError) as exc:
        print(f"Error: Invalid calibration for {motor_key}: {exc}")
        return None

    if direction_sign not in (-1, 1):
        print("Error: direction_sign must be 1 or -1")
        return None

    if not is_finite(raw_zero_pos_rad):
        print("Error: raw_zero_pos_rad must be finite")
        return None

    return raw_zero_pos_rad, direction_sign


def joint_from_raw(raw_pos_rad: float, raw_zero_pos_rad: float, direction_sign: int) -> float:
    return direction_sign * (raw_pos_rad - raw_zero_pos_rad)


def compute_target_raw(raw_zero_pos_rad: float, direction_sign: int, target_joint_rad: float) -> float:
    return raw_zero_pos_rad + direction_sign * target_joint_rad


def compute_final_error(target_joint_deg: float, after_joint_deg: float) -> float:
    return target_joint_deg - after_joint_deg


def raw_target_in_range(target_raw_pos_rad: float) -> bool:
    return AK70_10_LIMIT.p_min <= target_raw_pos_rad <= AK70_10_LIMIT.p_max


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
        if raw_bytes == echo_packet or raw_bytes == MIT_ENTER_PACKET:
            print(f"{format_motor_id(motor_id)} | [Rx SKIP] local echo packet")
            continue

        if raw_bytes[0] != (motor_id & 0xFF):
            continue

        return raw_bytes

    return None


def read_joint_once(
    bus: can.BusABC,
    motor_id: int,
    raw_zero_pos_rad: float,
    direction_sign: int,
    zero_torque_packet: bytes,
) -> dict[str, float] | None:
    drain_rx_queue(bus)

    msg = can.Message(arbitration_id=motor_id, data=zero_torque_packet, is_extended_id=False)
    bus.send(msg)

    raw_bytes = receive_feedback(bus, motor_id, zero_torque_packet)
    if raw_bytes is None:
        return None

    candidate = analyze_feedback_candidate(raw_bytes)
    if candidate is None:
        return None

    raw_pos_rad = float(candidate["candidate_position_rad"])
    joint_rad = joint_from_raw(raw_pos_rad, raw_zero_pos_rad, direction_sign)
    return {
        "raw_pos_rad": raw_pos_rad,
        "joint_rad": joint_rad,
        "joint_deg": math.degrees(joint_rad),
    }


def best_effort_zero_torque(bus: can.BusABC | None, motor_id: int, zero_torque_packet: bytes) -> None:
    if bus is None:
        return
    try:
        msg = can.Message(arbitration_id=motor_id, data=zero_torque_packet, is_extended_id=False)
        bus.send(msg)
    except Exception as exc:
        print(f"Best-effort zero-torque send failed: {exc}")


def print_move_plan(
    motor_id: int,
    current_joint_deg: float,
    target_joint_deg: float,
    delta_joint_deg: float,
    raw_zero_pos_rad: float,
    target_raw_pos_rad: float,
    direction_sign: int,
    kp: float,
    kd: float,
    pulses: int,
    interval_sec: float,
    enter_mit: bool,
) -> None:
    print("[Joint Move Plan]")
    print(f"ID: {format_motor_id(motor_id)}")
    print(f"Current joint: {current_joint_deg:+.2f} deg")
    print(f"Target joint: {target_joint_deg:+.2f} deg")
    print(f"Delta: {delta_joint_deg:+.2f} deg")
    print(f"Raw zero: {raw_zero_pos_rad:.6f} rad")
    print(f"Target raw: {target_raw_pos_rad:.6f} rad")
    print(f"direction_sign: {direction_sign}")
    print(f"kp: {kp}")
    print(f"kd: {kd}")
    print(f"pulses: {pulses}")
    print(f"interval_sec: {interval_sec:.3f}")
    print(f"enter_mit: {enter_mit}")


def send_position_pulses(
    bus: can.BusABC,
    motor_id: int,
    target_raw_pos_rad: float,
    kp: float,
    kd: float,
    pulses: int,
    interval_sec: float,
    enter_mit: bool,
) -> None:
    if enter_mit:
        enter_msg = can.Message(arbitration_id=motor_id, data=MIT_ENTER_PACKET, is_extended_id=False)
        bus.send(enter_msg)
        time.sleep(MIT_ENTER_SLEEP_SEC)

    position_packet = pack_mit_command(target_raw_pos_rad, 0.0, kp, kd, 0.0)
    position_msg = can.Message(arbitration_id=motor_id, data=position_packet, is_extended_id=False)

    for pulse_index in range(1, pulses + 1):
        bus.send(position_msg)
        print(
            f"[Pulse {pulse_index}/{pulses}] ID: {format_motor_id(motor_id)} | "
            f"Data: {format_packet(position_packet)}"
        )
        time.sleep(interval_sec)


def run_once(
    channel: str,
    motor_id: int,
    target_joint_rad: float,
    target_joint_deg: float,
    kp: float,
    kd: float,
    pulses: int,
    interval_sec: float,
    enter_mit: bool,
) -> int:
    print_safety_warning(
        channel=channel,
        motor_id=motor_id,
        target_joint_deg=target_joint_deg,
        target_joint_rad=target_joint_rad,
        kp=kp,
        kd=kd,
        pulses=pulses,
        interval_sec=interval_sec,
        enter_mit=enter_mit,
    )

    if not validate_parameters(motor_id, target_joint_rad, target_joint_deg, kp, kd, pulses, interval_sec):
        return 1

    try:
        calibration = load_motor_calibration()
    except Exception as exc:
        print(f"Error: Failed to load motor_calibration.yaml: {exc}")
        return 2

    motor_calibration = get_motor_calibration(motor_id, calibration)
    if motor_calibration is None:
        return 2
    raw_zero_pos_rad, direction_sign = motor_calibration

    zero_torque_packet = pack_mit_command(0.0, 0.0, 0.0, 0.0, 0.0)
    if not validate_zero_torque_packet(zero_torque_packet):
        return 2

    if not confirm_yes():
        print("Movement cancelled. No position commands were sent.")
        return 0

    bus = None
    position_pulses_sent = False
    try:
        bus = can.Bus(interface=DEFAULT_INTERFACE, channel=channel)
        current_state = read_joint_once(
            bus=bus,
            motor_id=motor_id,
            raw_zero_pos_rad=raw_zero_pos_rad,
            direction_sign=direction_sign,
            zero_torque_packet=zero_torque_packet,
        )
        if current_state is None:
            print("No feedback")
            print("No position commands were sent")
            return 2

        current_joint_deg = current_state["joint_deg"]
        delta_joint_deg = target_joint_deg - current_joint_deg

        if abs(target_joint_deg) > MAX_SAFETY_TARGET_DEG:
            print("Movement blocked due to safety limit constraints.")
            print("No position commands were sent.")
            return 2

        if abs(delta_joint_deg) > MAX_SAFETY_DELTA_DEG:
            print("Movement blocked due to safety limit constraints.")
            print("No position commands were sent.")
            return 2

        if abs(delta_joint_deg) <= TARGET_TOLERANCE_DEG:
            print("Already near target. No movement required.")
            return 0

        target_raw_pos_rad = compute_target_raw(raw_zero_pos_rad, direction_sign, target_joint_rad)
        if not raw_target_in_range(target_raw_pos_rad):
            print("Movement blocked: target raw position is outside AK70 command range.")
            print("No position commands were sent.")
            return 2

        print_move_plan(
            motor_id=motor_id,
            current_joint_deg=current_joint_deg,
            target_joint_deg=target_joint_deg,
            delta_joint_deg=delta_joint_deg,
            raw_zero_pos_rad=raw_zero_pos_rad,
            target_raw_pos_rad=target_raw_pos_rad,
            direction_sign=direction_sign,
            kp=kp,
            kd=kd,
            pulses=pulses,
            interval_sec=interval_sec,
            enter_mit=enter_mit,
        )

        if not confirm_move():
            print("Movement cancelled.")
            print("No position commands were sent.")
            return 0

        send_position_pulses(
            bus=bus,
            motor_id=motor_id,
            target_raw_pos_rad=target_raw_pos_rad,
            kp=kp,
            kd=kd,
            pulses=pulses,
            interval_sec=interval_sec,
            enter_mit=enter_mit,
        )
        position_pulses_sent = True

        time.sleep(FINAL_SETTLING_SEC)
        drain_rx_queue(bus)
        after_state = read_joint_once(
            bus=bus,
            motor_id=motor_id,
            raw_zero_pos_rad=raw_zero_pos_rad,
            direction_sign=direction_sign,
            zero_torque_packet=zero_torque_packet,
        )
        if after_state is None:
            print("Position pulses were sent, but final feedback was not received.")
            return 2

        after_joint_deg = after_state["joint_deg"]
        final_error_deg = compute_final_error(target_joint_deg, after_joint_deg)

        print("[Final Joint State]")
        print(f"ID: {format_motor_id(motor_id)}")
        print(f"Before: {current_joint_deg:+.2f} deg")
        print(f"Target: {target_joint_deg:+.2f} deg")
        print(f"After: {after_joint_deg:+.2f} deg")
        print(f"Final error: {final_error_deg:+.2f} deg")
        return 0
    except KeyboardInterrupt:
        print("Interrupted by user.")
        if not position_pulses_sent:
            print("No position commands were sent.")
        return 1
    except can.CanError as exc:
        print(f"CAN error: {exc}")
        if not position_pulses_sent:
            print("No position commands were sent.")
        return 1
    except ValueError as exc:
        print(f"Value error: {exc}")
        if not position_pulses_sent:
            print("No position commands were sent.")
        return 1
    except Exception as exc:
        print(f"Unexpected error: {exc}")
        if not position_pulses_sent:
            print("No position commands were sent.")
        return 1
    finally:
        if bus is not None:
            best_effort_zero_torque(bus, motor_id, zero_torque_packet)
            bus.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Move one AK70-10 motor once to a software-zero-relative joint angle."
    )
    parser.add_argument("--channel", default=DEFAULT_CHANNEL)
    parser.add_argument("--motor-id", type=parse_motor_id, required=True)

    target_group = parser.add_mutually_exclusive_group(required=True)
    target_group.add_argument("--target-deg", type=float)
    target_group.add_argument("--target-rad", type=float)

    parser.add_argument("--kp", type=float, default=DEFAULT_KP)
    parser.add_argument("--kd", type=float, default=DEFAULT_KD)
    parser.add_argument("--pulses", type=int, default=DEFAULT_PULSES)
    parser.add_argument("--interval-sec", type=float, default=DEFAULT_INTERVAL_SEC)
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
            pulses=args.pulses,
            interval_sec=args.interval_sec,
            enter_mit=args.enter_mit,
        )
    )


if __name__ == "__main__":
    main()
