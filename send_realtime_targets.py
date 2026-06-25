#!/usr/bin/env python3
"""Send validated target commands to the realtime controller IPC socket."""

from __future__ import annotations

import argparse
import json
import time
import uuid
from pathlib import Path

from motor_profiles import normalize_motor_id
from realtime_ipc import DEFAULT_SOCKET_PATH, send_request


def parse_target_deg(value: str) -> dict:
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("target must use MOTOR_ID,DEG")
    return {"motor_id": f"0x{normalize_motor_id(parts[0]):03X}", "position_deg": float(parts[1])}


def main() -> None:
    parser = argparse.ArgumentParser(description="Send latest targets to the realtime mixed AK controller.")
    parser.add_argument("--socket-path", type=Path, default=DEFAULT_SOCKET_PATH)
    parser.add_argument("--target-deg", action="append", default=None)
    parser.add_argument("--arm", action="store_true")
    parser.add_argument("--disarm", action="store_true")
    parser.add_argument("--clear-targets", action="store_true")
    parser.add_argument("--status", action="store_true")
    parser.add_argument("--shutdown", action="store_true")
    parser.add_argument("--session-token")
    parser.add_argument("--session-epoch", type=int)
    parser.add_argument("--heartbeat", action="store_true")
    parser.add_argument("--heartbeat-seq", type=int, default=0)
    parser.add_argument("--release-id", action="append", default=None)
    parser.add_argument("--release-session", action="store_true")
    parser.add_argument("--estop", action="store_true")
    args = parser.parse_args()

    if args.arm:
        payload = {"command": "ARM", "request_id": uuid.uuid4().hex}
        if args.session_token:
            payload["session_token"] = args.session_token
        print(json.dumps(send_request(payload, args.socket_path), ensure_ascii=False))
    if args.status:
        print(json.dumps(send_request({"command": "STATUS", "request_id": uuid.uuid4().hex}, args.socket_path), ensure_ascii=False))
    if args.heartbeat:
        if not args.session_token:
            parser.error("--heartbeat requires --session-token")
        print(
            json.dumps(
                send_request(
                    {
                        "command": "HEARTBEAT",
                        "request_id": uuid.uuid4().hex,
                        "session_token": args.session_token,
                        "heartbeat_seq": args.heartbeat_seq,
                        "generated_monotonic": time.monotonic(),
                    },
                    args.socket_path,
                ),
                ensure_ascii=False,
            )
        )
    if args.clear_targets:
        print(json.dumps(send_request({"command": "CLEAR_TARGETS", "request_id": uuid.uuid4().hex}, args.socket_path), ensure_ascii=False))
    if args.target_deg:
        targets = [parse_target_deg(value) for value in args.target_deg]
        payload = {"command": "SET_TARGETS", "request_id": uuid.uuid4().hex, "targets": targets}
        if args.session_token:
            payload["session_token"] = args.session_token
            for target in targets:
                target["session_token"] = args.session_token
        if args.session_epoch is not None:
            payload["session_epoch"] = args.session_epoch
            for target in targets:
                target["session_epoch"] = args.session_epoch
        print(json.dumps(send_request(payload, args.socket_path), ensure_ascii=False))
    if args.release_id:
        if not args.session_token:
            parser.error("--release-id requires --session-token")
        print(
            json.dumps(
                send_request(
                    {
                        "command": "RELEASE_IDS",
                        "request_id": uuid.uuid4().hex,
                        "session_token": args.session_token,
                        "motor_ids": args.release_id,
                    },
                    args.socket_path,
                ),
                ensure_ascii=False,
            )
        )
    if args.release_session:
        if not args.session_token:
            parser.error("--release-session requires --session-token")
        print(
            json.dumps(
                send_request(
                    {"command": "RELEASE_SESSION", "request_id": uuid.uuid4().hex, "session_token": args.session_token},
                    args.socket_path,
                ),
                ensure_ascii=False,
            )
        )
    if args.estop:
        print(json.dumps(send_request({"command": "ESTOP", "request_id": uuid.uuid4().hex}, args.socket_path), ensure_ascii=False))
    if args.disarm:
        print(json.dumps(send_request({"command": "DISARM", "request_id": uuid.uuid4().hex}, args.socket_path), ensure_ascii=False))
    if args.shutdown:
        print(json.dumps(send_request({"command": "SHUTDOWN", "request_id": uuid.uuid4().hex}, args.socket_path), ensure_ascii=False))
    if not any(
        [
            args.arm,
            args.status,
            args.heartbeat,
            args.clear_targets,
            args.target_deg,
            args.release_id,
            args.release_session,
            args.estop,
            args.disarm,
            args.shutdown,
        ]
    ):
        parser.error("select at least one command")


if __name__ == "__main__":
    main()
