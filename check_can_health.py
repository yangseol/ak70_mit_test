#!/usr/bin/env python3
"""Read SocketCAN interface health counters without sending motor commands."""

from __future__ import annotations

import argparse
import re
import subprocess
import time
from dataclasses import dataclass


@dataclass
class CanHealth:
    interface_state: str = "UNKNOWN"
    can_state: str = "UNKNOWN"
    restarted: int = 0
    bus_errors: int = 0
    arbitration_lost: int = 0
    rx_packets: int = 0
    rx_dropped: int = 0
    rx_overrun: int = 0
    tx_packets: int = 0
    tx_dropped: int = 0
    txqueuelen: int = 0


def run_ip(channel: str) -> str:
    result = subprocess.run(
        ["ip", "-details", "-statistics", "link", "show", channel],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(result.stdout.strip())
    return result.stdout


def _first_int(pattern: str, text: str, default: int = 0) -> int:
    match = re.search(pattern, text, flags=re.IGNORECASE)
    return int(match.group(1)) if match else default


def parse_health(text: str) -> CanHealth:
    health = CanHealth()
    state = re.search(r"\bstate\s+(\S+)", text)
    if state:
        health.interface_state = state.group(1)
    can_state = re.search(r"\bcan\s+state\s+([A-Z-]+)", text)
    if can_state:
        health.can_state = can_state.group(1)
    if health.interface_state == "DOWN":
        health.can_state = "DOWN"
    health.restarted = _first_int(r"\bre-started\s+(\d+)", text)
    health.bus_errors = _first_int(r"\bbus-errors\s+(\d+)", text)
    health.arbitration_lost = _first_int(r"\barbitration-lost\s+(\d+)", text)
    health.txqueuelen = _first_int(r"\bqlen\s+(\d+)", text)

    lines = text.splitlines()
    for idx, line in enumerate(lines):
        if re.search(r"\bRX:\s+bytes", line) and idx + 1 < len(lines):
            values = [int(x) for x in re.findall(r"\d+", lines[idx + 1])]
            if len(values) >= 5:
                health.rx_packets = values[1]
                health.rx_dropped = values[3]
                health.rx_overrun = values[4]
        if re.search(r"\bTX:\s+bytes", line) and idx + 1 < len(lines):
            values = [int(x) for x in re.findall(r"\d+", lines[idx + 1])]
            if len(values) >= 4:
                health.tx_packets = values[1]
                health.tx_dropped = values[3]
    return health


def diff_health(before: CanHealth, after: CanHealth) -> dict[str, int]:
    fields = [
        "restarted",
        "bus_errors",
        "arbitration_lost",
        "rx_packets",
        "rx_dropped",
        "rx_overrun",
        "tx_packets",
        "tx_dropped",
    ]
    return {field: getattr(after, field) - getattr(before, field) for field in fields}


def print_health(health: CanHealth) -> None:
    print(f"interface state: {health.interface_state}")
    print(f"CAN state: {health.can_state}")
    for label in ("ERROR-ACTIVE", "ERROR-WARNING", "ERROR-PASSIVE", "BUS-OFF"):
        print(f"{label}: {'YES' if health.can_state == label else 'NO'}")
    print(f"re-started: {health.restarted}")
    print(f"bus-errors: {health.bus_errors}")
    print(f"arbitration-lost: {health.arbitration_lost}")
    print(f"RX packets: {health.rx_packets}")
    print(f"RX dropped: {health.rx_dropped}")
    print(f"RX overrun: {health.rx_overrun}")
    print(f"TX packets: {health.tx_packets}")
    print(f"TX dropped: {health.tx_dropped}")
    print(f"txqueuelen: {health.txqueuelen}")


def print_diagnosis(delta: dict[str, int], health: CanHealth) -> None:
    print("")
    print("diagnosis:")
    if delta.get("bus_errors", 0) > 0:
        print("- CAN error counter increased: wiring, termination, EMI, or GND issue is possible.")
    if delta.get("rx_dropped", 0) > 0:
        print("- RX dropped increased: socket or program receive handling issue is possible.")
    if delta.get("tx_dropped", 0) > 0:
        print("- TX dropped increased: pacing or queue pressure issue is possible.")
    if delta.get("restarted", 0) > 0 or health.can_state == "BUS-OFF":
        print("- BUS-OFF/restart observed: physical layer or ACK issue is possible.")
    print("- If only motors reboot, suspect supply voltage dip.")
    print("- If CAN counters are normal but timing overruns occur, suspect scheduler or CPU load.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Check SocketCAN health without sending CAN motor commands.")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--compare-sec", type=float, default=0.0, help="Take a second snapshot after this delay and print counter deltas.")
    args = parser.parse_args()
    before = parse_health(run_ip(args.channel))
    print("[start snapshot]")
    print_health(before)
    if args.compare_sec > 0.0:
        time.sleep(args.compare_sec)
        after = parse_health(run_ip(args.channel))
        print("")
        print("[end snapshot]")
        print_health(after)
        delta = diff_health(before, after)
        print("")
        print("[delta]")
        for key, value in delta.items():
            print(f"{key}: {value:+d}")
        print_diagnosis(delta, after)


if __name__ == "__main__":
    main()

