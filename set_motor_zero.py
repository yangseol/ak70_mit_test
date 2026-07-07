from __future__ import annotations

import argparse


DEFAULT_CHANNEL = "can0"
DEFAULT_MOTOR_ID = 0x00A


def run_once(channel: str = DEFAULT_CHANNEL, motor_id: int = DEFAULT_MOTOR_ID) -> int:
    print(f"channel: {channel}")
    print(f"motor_id: 0x{motor_id:03X}")
    print("This hardware-origin tool is disabled.")
    print("Use software-zero calibration files and GUI helpers instead.")
    print("No CAN bus was opened and no motor command was sent.")
    return 2


def _parse_motor_id(value: str) -> int:
    return int(value, 0)


def main() -> None:
    parser = argparse.ArgumentParser(description="Disabled hardware-origin tool.")
    parser.add_argument("--channel", default=DEFAULT_CHANNEL, help="SocketCAN channel name.")
    parser.add_argument("--motor-id", type=_parse_motor_id, default=DEFAULT_MOTOR_ID, help="Motor arbitration ID.")
    args = parser.parse_args()
    raise SystemExit(run_once(channel=args.channel, motor_id=args.motor_id))


if __name__ == "__main__":
    main()
