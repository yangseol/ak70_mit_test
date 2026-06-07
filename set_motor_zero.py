from __future__ import annotations

import argparse
import time

import can


DEFAULT_CHANNEL = "can0"
DEFAULT_INTERFACE = "socketcan"
DEFAULT_MOTOR_ID = 0x00A

ENTER_MIT_MODE = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
SET_CURRENT_POSITION_ZERO = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])


# Physical safety checklist before typing YES:
# - Only one motor connected
# - Confirm motor ID is 0x00A
# - Remove load/linkage if possible
# - Keep hand near power cutoff
# - Align shaft to intended zero posture before typing YES


def format_data_bytes(data: bytes) -> str:
    return ", ".join(f"{byte:02X}" for byte in data)


def print_preflight(channel: str, motor_id: int) -> None:
    print(f"channel: {channel}")
    print(f"motor_id: 0x{motor_id:03X}")
    print(f"ENTER_MIT_MODE hex: {ENTER_MIT_MODE.hex(' ').upper()}")
    print(f"SET_CURRENT_POSITION_ZERO hex: {SET_CURRENT_POSITION_ZERO.hex(' ').upper()}")
    print("WARNING: This will set the CURRENT shaft position as the motor zero reference.")
    print("Ensure the shaft is physically aligned to the intended 0-degree posture.")
    print("This may change the motor's internal zero reference depending on firmware behavior.")


def run_once(channel: str = DEFAULT_CHANNEL, motor_id: int = DEFAULT_MOTOR_ID) -> int:
    print_preflight(channel, motor_id)
    confirmation = input("Type YES to transmit: ")
    if confirmation != "YES":
        print("Aborted. No CAN bus opened and no zero command transmitted.")
        return 0

    bus = None
    try:
        bus = can.Bus(interface=DEFAULT_INTERFACE, channel=channel)

        enter_msg = can.Message(arbitration_id=motor_id, data=ENTER_MIT_MODE, is_extended_id=False)
        bus.send(enter_msg)
        print(f"[Tx Enter MIT] ID: 0x{motor_id:03X} | Data: [{format_data_bytes(ENTER_MIT_MODE)}]")

        time.sleep(0.2)

        zero_msg = can.Message(arbitration_id=motor_id, data=SET_CURRENT_POSITION_ZERO, is_extended_id=False)
        bus.send(zero_msg)
        print(f"[Tx Set Zero]  ID: 0x{motor_id:03X} | Data: [{format_data_bytes(SET_CURRENT_POSITION_ZERO)}]")

        time.sleep(0.5)
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
    parser = argparse.ArgumentParser(description="Set the current AK70-10 shaft position as the MIT zero reference.")
    parser.add_argument("--channel", default=DEFAULT_CHANNEL, help="SocketCAN channel name.")
    parser.add_argument("--motor-id", type=_parse_motor_id, default=DEFAULT_MOTOR_ID, help="Motor arbitration ID.")
    args = parser.parse_args()

    raise SystemExit(run_once(channel=args.channel, motor_id=args.motor_id))


if __name__ == "__main__":
    main()
