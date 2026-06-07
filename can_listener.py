from __future__ import annotations

import argparse
import time

import can


# Debug-only raw CAN monitor. Do not use print-based monitoring in a high-speed
# control loop; this script is intentionally receive-only for bus observation.


def format_can_message(msg: can.Message) -> str:
    timestamp = f"{msg.timestamp:.6f}" if msg.timestamp is not None else "0.000000"
    arbitration_id = f"0x{msg.arbitration_id:03X}"
    data = ", ".join(f"{byte:02X}" for byte in msg.data)
    return f"[Rx] t={timestamp} | ID: {arbitration_id} | DLC: {msg.dlc} | Data: [{data}]"


class RawCanPrintListener(can.Listener):
    def __init__(self, filter_ids: set[int] | None = None) -> None:
        self.filter_ids = filter_ids
        self.rx_count = 0

    def on_message_received(self, msg: can.Message) -> None:
        if self.filter_ids is not None and msg.arbitration_id not in self.filter_ids:
            return

        self.rx_count += 1
        print(format_can_message(msg), flush=True)

    def on_error(self, exc: Exception) -> None:
        print(f"[RxError] {exc}", flush=True)


def cleanup_resources(notifier, bus) -> None:
    if notifier is not None:
        try:
            notifier.stop()
        except Exception as exc:
            print(f"[CleanupError] notifier.stop(): {exc}", flush=True)

    if bus is not None:
        try:
            bus.shutdown()
        except Exception as exc:
            print(f"[CleanupError] bus.shutdown(): {exc}", flush=True)


def run_listener(channel: str, interface: str = "socketcan", filter_ids: set[int] | None = None) -> None:
    bus = None
    notifier = None

    try:
        bus = can.Bus(interface=interface, channel=channel)
        listener = RawCanPrintListener(filter_ids=filter_ids)
        notifier = can.Notifier(bus, [listener])

        print(
            f"[RxStart] interface={interface} channel={channel} "
            f"filter_ids={_format_filter_ids(filter_ids)}",
            flush=True,
        )

        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("[RxStop] KeyboardInterrupt received; shutting down.", flush=True)
    except can.CanError as exc:
        print(f"[CanError] {exc}", flush=True)
    finally:
        cleanup_resources(notifier, bus)


def _parse_filter_id(value: str) -> int:
    return int(value, 0)


def _format_filter_ids(filter_ids: set[int] | None) -> str:
    if filter_ids is None:
        return "all"
    return ",".join(f"0x{filter_id:03X}" for filter_id in sorted(filter_ids))


def main() -> None:
    parser = argparse.ArgumentParser(description="Receive-only raw SocketCAN monitor.")
    parser.add_argument("--channel", default="can0", help="SocketCAN channel name, e.g. can0 or vcan0.")
    parser.add_argument("--interface", default="socketcan", help="python-can interface name.")
    parser.add_argument(
        "--filter-id",
        action="append",
        type=_parse_filter_id,
        default=None,
        help="Arbitration ID to print. May be repeated. Accepts decimal or 0x-prefixed hex.",
    )
    args = parser.parse_args()

    filter_ids = set(args.filter_id) if args.filter_id is not None else None
    run_listener(channel=args.channel, interface=args.interface, filter_ids=filter_ids)


if __name__ == "__main__":
    main()
