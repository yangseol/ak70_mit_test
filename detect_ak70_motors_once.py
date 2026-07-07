#!/usr/bin/env python3
"""
AK70-10 단독 CAN 감지 도구

- 기본 CAN 채널: can0
- 기본 검색 ID: 0x001 ~ 0x00A
- 각 ID에 MIT 진입 패킷과 Zero Torque 패킷을 보내고,
  동일 arbitration ID의 응답이 오는지 확인한다.

주의:
이 스크립트는 Zero Torque 명령을 전송하므로 모터의 힘이 풀릴 수 있다.
로봇 구조물을 반드시 지지하거나 고정한 뒤 실행한다.
실시간 controller 또는 다른 CAN 송신 프로그램과 동시에 실행하지 않는다.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Iterable

try:
    import can
except ImportError:
    print("[오류] python-can이 설치되어 있지 않습니다.")
    print("설치: python3 -m pip install python-can")
    raise SystemExit(1)


MIT_ENTER = bytes.fromhex("FF FF FF FF FF FF FF FC")
ZERO_TORQUE = bytes.fromhex("80 00 80 00 00 00 08 00")

DEFAULT_START_ID = 0x001
DEFAULT_END_ID = 0x00A


@dataclass
class DetectionResult:
    motor_id: int
    detected: bool
    response_count: int = 0
    last_data: bytes = b""
    last_timestamp: float | None = None


def parse_int(value: str) -> int:
    """0x00A 또는 10 형식 모두 허용."""
    try:
        return int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"잘못된 정수/ID 형식: {value}") from exc


def check_interface(channel: str) -> None:
    """can0가 존재하고 UP 상태인지 확인한다."""
    try:
        result = subprocess.run(
            ["ip", "-details", "link", "show", channel],
            check=False,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError:
        print("[경고] ip 명령을 찾지 못해 CAN 인터페이스 상태를 확인하지 못했습니다.")
        return

    if result.returncode != 0:
        print(f"[오류] CAN 인터페이스 {channel}를 찾을 수 없습니다.")
        print(result.stderr.strip() or result.stdout.strip())
        raise SystemExit(2)

    first_line = result.stdout.splitlines()[0] if result.stdout else ""
    if "UP" not in first_line:
        print(f"[오류] {channel}가 UP 상태가 아닙니다.")
        print("먼저 기존 프로젝트의 CAN 설정 명령으로 can0를 올린 뒤 다시 실행하세요.")
        raise SystemExit(2)


def open_bus(channel: str):
    """python-can 구버전/신버전을 모두 고려해 SocketCAN bus를 연다."""
    try:
        return can.Bus(interface="socketcan", channel=channel)
    except TypeError:
        return can.interface.Bus(bustype="socketcan", channel=channel)


def drain_bus(bus, duration: float = 0.03) -> None:
    """이전 프레임을 짧게 비운다."""
    end = time.monotonic() + duration
    while time.monotonic() < end:
        if bus.recv(timeout=0.001) is None:
            break


def send_frame(bus, motor_id: int, payload: bytes) -> None:
    msg = can.Message(
        arbitration_id=motor_id,
        data=payload,
        is_extended_id=False,
    )
    bus.send(msg, timeout=0.1)


def wait_for_matching_response(
    bus,
    motor_id: int,
    timeout: float,
) -> tuple[int, bytes, float | None]:
    """
    같은 arbitration ID 응답을 기다린다.
    한 번의 timeout 동안 여러 프레임이 오면 개수를 센다.
    """
    deadline = time.monotonic() + timeout
    count = 0
    last_data = b""
    last_timestamp: float | None = None

    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break

        msg = bus.recv(timeout=remaining)
        if msg is None:
            break

        if msg.is_error_frame:
            continue

        if msg.arbitration_id == motor_id:
            count += 1
            last_data = bytes(msg.data)
            last_timestamp = getattr(msg, "timestamp", None)

    return count, last_data, last_timestamp


def detect_one(
    bus,
    motor_id: int,
    timeout: float,
    retries: int,
    enter_mit: bool,
) -> DetectionResult:
    total_count = 0
    last_data = b""
    last_timestamp: float | None = None

    for _ in range(retries):
        drain_bus(bus)

        if enter_mit:
            send_frame(bus, motor_id, MIT_ENTER)
            time.sleep(0.01)

        # 검증된 Zero Torque 패킷. 응답 유도용이며 토크가 풀릴 수 있다.
        send_frame(bus, motor_id, ZERO_TORQUE)

        count, data, timestamp = wait_for_matching_response(
            bus=bus,
            motor_id=motor_id,
            timeout=timeout,
        )
        total_count += count

        if count:
            last_data = data
            last_timestamp = timestamp
            return DetectionResult(
                motor_id=motor_id,
                detected=True,
                response_count=total_count,
                last_data=last_data,
                last_timestamp=last_timestamp,
            )

        time.sleep(0.01)

    return DetectionResult(
        motor_id=motor_id,
        detected=False,
        response_count=total_count,
    )


def make_id_range(start_id: int, end_id: int) -> Iterable[int]:
    if start_id < 0 or end_id > 0x7FF:
        raise ValueError("표준 CAN ID 범위는 0x000~0x7FF입니다.")
    if start_id > end_id:
        raise ValueError("시작 ID가 끝 ID보다 클 수 없습니다.")
    return range(start_id, end_id + 1)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="AK70-10 ID 0x001~0x00A 단독 감지 도구"
    )
    parser.add_argument("--channel", default="can0", help="SocketCAN 채널 (기본: can0)")
    parser.add_argument("--start-id", type=parse_int, default=DEFAULT_START_ID)
    parser.add_argument("--end-id", type=parse_int, default=DEFAULT_END_ID)
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.08,
        help="ID별 응답 대기 시간 초 (기본: 0.08)",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=3,
        help="ID별 재시도 횟수 (기본: 3)",
    )
    parser.add_argument(
        "--skip-enter-mit",
        action="store_true",
        help="MIT 진입 패킷을 보내지 않고 Zero Torque만 전송",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="안전 확인 질문 없이 바로 실행",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()

    if not math.isfinite(args.timeout) or args.timeout <= 0:
        print("[오류] --timeout은 0보다 큰 유한한 값이어야 합니다.")
        return 2
    if args.retries <= 0:
        print("[오류] --retries는 1 이상이어야 합니다.")
        return 2

    try:
        motor_ids = list(make_id_range(args.start_id, args.end_id))
    except ValueError as exc:
        print(f"[오류] {exc}")
        return 2

    print("=" * 66)
    print("AK70-10 단독 감지")
    print(f"채널: {args.channel}")
    print(f"검색: 0x{motor_ids[0]:03X} ~ 0x{motor_ids[-1]:03X}")
    print("주의: MIT 진입 및 Zero Torque 명령을 전송합니다.")
    print("      모터의 힘이 풀릴 수 있으므로 구조물을 지지하세요.")
    print("      다른 controller/GUI/CAN 송신 프로그램은 모두 종료하세요.")
    print("=" * 66)

    if not args.yes:
        answer = input("안전 상태를 확인했다면 YES 입력: ").strip()
        if answer != "YES":
            print("취소했습니다.")
            return 1

    check_interface(args.channel)

    detected: list[DetectionResult] = []

    try:
        bus = open_bus(args.channel)
    except Exception as exc:
        print(f"[오류] {args.channel} 열기 실패: {exc}")
        return 3

    try:
        for motor_id in motor_ids:
            print(f"[검색] 0x{motor_id:03X} ... ", end="", flush=True)

            try:
                result = detect_one(
                    bus=bus,
                    motor_id=motor_id,
                    timeout=args.timeout,
                    retries=args.retries,
                    enter_mit=not args.skip_enter_mit,
                )
            except can.CanError as exc:
                print(f"CAN 오류: {exc}")
                continue

            if result.detected:
                data_hex = result.last_data.hex(" ").upper()
                print(
                    f"감지됨 | 응답 {result.response_count}회"
                    f" | DATA: {data_hex or '(없음)'}"
                )
                detected.append(result)
            else:
                print("응답 없음")
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass

    print()
    print("=" * 66)
    if detected:
        ids = ", ".join(f"0x{item.motor_id:03X}" for item in detected)
        print(f"[완료] 감지된 AK70: {ids}")
        print(f"[완료] 총 {len(detected)}개")
        return 0

    print("[완료] 감지된 AK70이 없습니다.")
    print("확인 항목:")
    print("1. can0가 UP인지 확인")
    print("2. CAN-H/CAN-L 배선과 GND 공통 연결 확인")
    print("3. 종단저항과 전원 확인")
    print("4. 다른 GUI/controller가 can0를 사용 중인지 확인")
    print("5. 모터 ID가 0x001~0x00A 범위인지 확인")
    return 4


if __name__ == "__main__":
    raise SystemExit(main())
