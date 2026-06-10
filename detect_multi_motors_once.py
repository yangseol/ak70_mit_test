"""AK70 모터 ID 범위를 한 번 스캔하고 calibration 여부를 출력하는 감지 helper."""

from __future__ import annotations

import argparse
import math
import time

import can

from calibration import apply_software_offset, load_motor_calibration
from mit_packet import analyze_feedback_candidate, pack_mit_command


DEFAULT_CHANNEL = "can0"
DEFAULT_INTERFACE = "socketcan"
DEFAULT_MOTOR_IDS = tuple(range(0x001, 0x00B))
DEFAULT_TIMEOUT_SEC = 0.08
RECV_POLL_TIMEOUT_SEC = 0.005
DRAIN_DURATION_SEC = 0.05
DRAIN_POLL_TIMEOUT_SEC = 0.005
ENTER_SLEEP_SEC = 0.02
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


def parse_motor_ids(value: str) -> list[int]:
    motor_ids = [parse_motor_id(part) for part in value.split(",")]
    if not motor_ids:
        raise argparse.ArgumentTypeError("at least one motor id is required")
    return motor_ids


def format_motor_id(motor_id: int) -> str:
    return f"0x{motor_id:03X}"


def format_motor_ids(motor_ids: list[int] | tuple[int, ...]) -> str:
    return ", ".join(format_motor_id(motor_id) for motor_id in motor_ids)


def format_packet(packet: bytes) -> str:
    return packet.hex(" ").upper()


def print_safety_warning(channel: str, motor_ids: list[int], timeout_sec: float, enter_mit: bool) -> None:
    print("SAFETY WARNING")
    print("- detect only")
    print("- no 0xFE")
    print("- no position control")
    print("- no nudge")
    print("- no calibration write")
    print(f"channel: {channel}")
    print(f"motor_ids: {format_motor_ids(motor_ids)}")
    print(f"timeout_sec: {timeout_sec:.3f}")
    print(f"enter_mit: {enter_mit}")


def confirm_before_opening_bus(channel: str, motor_ids: list[int], timeout_sec: float, enter_mit: bool) -> bool:
    print_safety_warning(channel, motor_ids, timeout_sec, enter_mit)
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


def receive_feedback(
    bus: can.BusABC,
    motor_id: int,
    zero_torque_packet: bytes,
    timeout_sec: float,
) -> bytes | None:
    deadline = time.monotonic() + timeout_sec

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

        if raw_bytes == zero_torque_packet or raw_bytes == MIT_ENTER_PACKET:
            continue

        if raw_bytes[0] != (motor_id & 0xFF):
            continue

        return raw_bytes

    return None


def has_calibration(motor_id: int, calibration: dict) -> bool:
    return format_motor_id(motor_id) in calibration.get("motors", {})


def build_detected_result(motor_id: int, raw_bytes: bytes, calibration: dict) -> dict:
    result: dict = {
        "motor_id": motor_id,
        "calibrated": has_calibration(motor_id, calibration),
    }

    candidate = analyze_feedback_candidate(raw_bytes)
    if candidate is None:
        return result

    raw_pos = float(candidate["candidate_position_rad"])
    result["raw_pos_rad"] = raw_pos

    if result["calibrated"]:
        joint_rad = apply_software_offset(raw_pos, motor_id, calibration)
        result["joint_rad"] = joint_rad
        result["joint_deg"] = math.degrees(joint_rad)

    return result


def scan_motor(
    bus: can.BusABC,
    motor_id: int,
    zero_torque_packet: bytes,
    timeout_sec: float,
    enter_mit: bool,
    calibration: dict,
) -> dict | None:
    drain_rx_queue(bus)

    if enter_mit:
        enter_msg = can.Message(arbitration_id=motor_id, data=MIT_ENTER_PACKET, is_extended_id=False)
        bus.send(enter_msg)
        time.sleep(ENTER_SLEEP_SEC)

    msg = can.Message(arbitration_id=motor_id, data=zero_torque_packet, is_extended_id=False)
    bus.send(msg)

    raw_bytes = receive_feedback(bus, motor_id, zero_torque_packet, timeout_sec)
    if raw_bytes is None:
        return None

    return build_detected_result(motor_id, raw_bytes, calibration)


def print_results(motor_ids: list[int], detected: list[dict]) -> None:
    detected_by_id = {result["motor_id"]: result for result in detected}
    sorted_detected_ids = sorted(detected_by_id)
    missing_ids = [motor_id for motor_id in sorted(motor_ids) if motor_id not in detected_by_id]
    calibrated_count = sum(1 for result in detected if result["calibrated"])
    uncalibrated_count = len(detected) - calibrated_count

    print("[Detected Motors]")
    for motor_id in sorted_detected_ids:
        result = detected_by_id[motor_id]
        calibrated = "YES" if result["calibrated"] else "NO"
        raw_pos_text = (
            f"{result['raw_pos_rad']:.6f}"
            if "raw_pos_rad" in result
            else "N/A"
        )
        if "joint_rad" in result:
            joint_rad_text = f"{result['joint_rad']:.6f}"
            joint_deg_text = f"{result['joint_deg']:.2f}"
        else:
            joint_rad_text = "N/A"
            joint_deg_text = "N/A"

        print(
            f"ID: {format_motor_id(motor_id)} | "
            f"calibrated: {calibrated} | "
            f"raw_pos_rad: {raw_pos_text} | "
            f"joint_rad: {joint_rad_text} | "
            f"joint_deg: {joint_deg_text}"
        )

    print("[Missing Motors]")
    print(format_motor_ids(missing_ids) if missing_ids else "none")

    print("[Summary]")
    print(f"scan count: {len(motor_ids)}")
    print(f"detected count: {len(detected)}")
    print(f"calibrated detected count: {calibrated_count}")
    print(f"uncalibrated detected count: {uncalibrated_count}")


def run_once(channel: str, motor_ids: list[int], timeout_sec: float, enter_mit: bool, yes: bool) -> int:
    if timeout_sec <= 0.0:
        print("Invalid timeout-sec: must be > 0.0")
        return 2

    zero_torque_packet = pack_mit_command(0.0, 0.0, 0.0, 0.0, 0.0)
    if zero_torque_packet != EXPECTED_ZERO_TORQUE_PACKET:
        print(
            "Refusing to transmit: zero-torque packet mismatch "
            f"actual={format_packet(zero_torque_packet)} "
            f"expected={format_packet(EXPECTED_ZERO_TORQUE_PACKET)}"
        )
        return 2

    if yes:
        print_safety_warning(channel, motor_ids, timeout_sec, enter_mit)
    elif not confirm_before_opening_bus(channel, motor_ids, timeout_sec, enter_mit):
        print("Aborted. No CAN bus opened and no command transmitted.")
        return 0

    calibration = load_motor_calibration()
    detected: list[dict] = []

    bus = None
    try:
        bus = can.Bus(interface=DEFAULT_INTERFACE, channel=channel)
        for motor_id in motor_ids:
            result = scan_motor(
                bus=bus,
                motor_id=motor_id,
                zero_torque_packet=zero_torque_packet,
                timeout_sec=timeout_sec,
                enter_mit=enter_mit,
                calibration=calibration,
            )
            if result is not None:
                detected.append(result)
    except Exception as exc:
        print(f"CAN bus open/read failed: {exc}")
        return 2
    finally:
        if bus is not None:
            bus.shutdown()

    print_results(motor_ids, detected)
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Detect connected AK70-10 motors with one zero-torque probe per candidate ID."
    )
    parser.add_argument("--channel", default=DEFAULT_CHANNEL)
    parser.add_argument(
        "--motor-ids",
        type=parse_motor_ids,
        default=list(DEFAULT_MOTOR_IDS),
        help="Comma-separated motor IDs, for example: 0x001,0x002,0x00A",
    )
    parser.add_argument("--timeout-sec", type=float, default=DEFAULT_TIMEOUT_SEC)
    parser.add_argument("--enter-mit", action="store_true")
    parser.add_argument("--yes", action="store_true")
    args = parser.parse_args()

    raise SystemExit(
        run_once(
            channel=args.channel,
            motor_ids=args.motor_ids,
            timeout_sec=args.timeout_sec,
            enter_mit=args.enter_mit,
            yes=args.yes,
        )
    )


if __name__ == "__main__":
    main()
