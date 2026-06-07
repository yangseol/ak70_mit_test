from __future__ import annotations

import argparse
import time

import can

from calibration import apply_software_offset, load_motor_calibration
from mit_packet import analyze_feedback_candidate, pack_mit_command


DEFAULT_CHANNEL = "can0"
DEFAULT_INTERFACE = "socketcan"
DEFAULT_MOTOR_ID = 0x00A
DEFAULT_KP = 1.5
DEFAULT_KD = 0.1
DEFAULT_TORQUE = 0.0
DEFAULT_MAX_START_ERROR_RAD = 0.5
RECEIVE_DEADLINE_SEC = 1.0
RECV_POLL_TIMEOUT_SEC = 0.05
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


def motor_key(motor_id: int) -> str:
    return format_motor_id(motor_id)


def get_raw_zero_pos_rad(motor_id: int, calibration: dict) -> float:
    key = motor_key(motor_id)
    try:
        return float(calibration["motors"][key]["raw_zero_pos_rad"])
    except KeyError as exc:
        raise KeyError(f"No raw_zero_pos_rad calibration found for motor_id={key}") from exc


def validate_zero_torque_packet(packet: bytes) -> bool:
    if packet == EXPECTED_ZERO_TORQUE_PACKET:
        return True

    print(
        "Refusing to transmit: zero-torque packet mismatch "
        f"actual={format_packet(packet)} expected={format_packet(EXPECTED_ZERO_TORQUE_PACKET)}"
    )
    return False


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
            print("[Rx SKIP] local echo packet")
            continue

        if raw_bytes[0] != (motor_id & 0xFF):
            continue

        return raw_bytes

    return None


def read_current_joint_once(bus: can.BusABC, motor_id: int, calibration: dict) -> dict[str, float] | None:
    packet = pack_mit_command(0.0, 0.0, 0.0, 0.0, 0.0)
    if not validate_zero_torque_packet(packet):
        return None

    msg = can.Message(arbitration_id=motor_id, data=packet, is_extended_id=False)
    bus.send(msg)

    raw_bytes = receive_feedback(bus, motor_id, packet)
    if raw_bytes is None:
        print("No feedback received")
        return None

    candidate = analyze_feedback_candidate(raw_bytes)
    if candidate is None:
        print("No feedback received")
        return None

    raw_pos = float(candidate["candidate_position_rad"])
    return {
        "current_raw_pos_rad": raw_pos,
        "current_joint_rad": apply_software_offset(raw_pos, motor_id, calibration),
    }


def confirm_position_command(current_joint_rad: float, raw_zero_pos_rad: float, kp: float, kd: float, torque: float) -> bool:
    print(f"Current Joint Rad: {current_joint_rad:.6f}")
    print("Target Joint Rad: 0.000000")
    print(f"Target Raw Position Rad: {raw_zero_pos_rad:.6f}")
    print(f"Safe Control Gains: Kp={kp}, Kd={kd}, Torque_Feedforward={torque}")
    print("WARNING: This will send exactly ONE brief position command.")
    confirmation = input("Type YES to send position command: ")
    return confirmation == "YES"


def send_position_command_once(
    bus: can.BusABC,
    motor_id: int,
    raw_zero_pos_rad: float,
    kp: float,
    kd: float,
    torque: float,
) -> None:
    packet = pack_mit_command(
        p=raw_zero_pos_rad,
        v=0.0,
        kp=kp,
        kd=kd,
        t=torque,
    )
    msg = can.Message(arbitration_id=motor_id, data=packet, is_extended_id=False)
    bus.send(msg)
    print(f"[Tx Position Cmd Once] ID: {format_motor_id(motor_id)} | Data: {format_packet(packet)}")


def run_once(channel: str, motor_id: int, kp: float, kd: float, torque: float, max_start_error_rad: float) -> int:
    calibration = load_motor_calibration()
    raw_zero_pos_rad = get_raw_zero_pos_rad(motor_id, calibration)

    bus = None
    try:
        bus = can.Bus(interface=DEFAULT_INTERFACE, channel=channel)
        current = read_current_joint_once(bus, motor_id, calibration)
        if current is None:
            return 0

        current_joint_rad = current["current_joint_rad"]
        if abs(current_joint_rad) > max_start_error_rad:
            print("Refusing to move: current joint error is too large.")
            print(f"Current Joint Rad: {current_joint_rad:.6f}")
            print(f"Max Start Error Rad: {max_start_error_rad:.6f}")
            return 0

        if not confirm_position_command(current_joint_rad, raw_zero_pos_rad, kp, kd, torque):
            print("Aborted. No position command transmitted.")
            return 0

        send_position_command_once(bus, motor_id, raw_zero_pos_rad, kp, kd, torque)
        return 0
    finally:
        if bus is not None:
            bus.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Send one low-gain AK70-10 position command toward the software zero."
    )
    parser.add_argument("--channel", default=DEFAULT_CHANNEL)
    parser.add_argument("--motor-id", type=parse_motor_id, default=DEFAULT_MOTOR_ID)
    parser.add_argument("--kp", type=float, default=DEFAULT_KP)
    parser.add_argument("--kd", type=float, default=DEFAULT_KD)
    parser.add_argument("--torque", type=float, default=DEFAULT_TORQUE)
    parser.add_argument("--max-start-error-rad", type=float, default=DEFAULT_MAX_START_ERROR_RAD)
    args = parser.parse_args()

    raise SystemExit(
        run_once(
            channel=args.channel,
            motor_id=args.motor_id,
            kp=args.kp,
            kd=args.kd,
            torque=args.torque,
            max_start_error_rad=args.max_start_error_rad,
        )
    )


if __name__ == "__main__":
    main()
