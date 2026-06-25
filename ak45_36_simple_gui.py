#!/usr/bin/env python3
"""
AK45-36 KV80 간단 회전 GUI
- Ubuntu SocketCAN can0
- MIT mode
- Motor CAN ID: 11 (0x00B)
- 버튼: 정방향 / 멈춤 / 역방향

외부 패키지 없이 Python 표준 라이브러리만 사용합니다.
"""

from __future__ import annotations

import queue
import socket
import struct
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, ttk


# =========================
# 사용자 설정
# =========================
CAN_INTERFACE = "can0"
MOTOR_ID = 11                 # 10진수 11 = 0x00B
TX_HZ = 50.0

DEFAULT_SPEED_RAD_S = 0.50    # AK45-36 출력축 기준 약 4.8 rpm
DEFAULT_KD = 3.0
FORWARD_SIGN = 1.0            # 방향이 반대면 -1.0으로 변경

# CubeMars AK45-36 MIT 전용 범위
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -6.0, 6.0
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX = -34.0, 34.0

ENTER_MOTOR_MODE = bytes.fromhex("FF FF FF FF FF FF FF FC")
EXIT_MOTOR_MODE = bytes.fromhex("FF FF FF FF FF FF FF FD")


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def float_to_uint(value: float, low: float, high: float, bits: int) -> int:
    value = clamp(value, low, high)
    span = high - low
    max_int = (1 << bits) - 1
    return int(round((value - low) * max_int / span))


def uint_to_float(value: int, low: float, high: float, bits: int) -> float:
    max_int = (1 << bits) - 1
    return value * (high - low) / max_int + low


def pack_mit_command(
    position: float,
    velocity: float,
    kp: float,
    kd: float,
    torque: float,
) -> bytes:
    """CubeMars MIT 8-byte command packet."""
    p_int = float_to_uint(position, P_MIN, P_MAX, 16)
    v_int = float_to_uint(velocity, V_MIN, V_MAX, 12)
    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    t_int = float_to_uint(torque, T_MIN, T_MAX, 12)

    return bytes(
        [
            (p_int >> 8) & 0xFF,
            p_int & 0xFF,
            (v_int >> 4) & 0xFF,
            ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F),
            kp_int & 0xFF,
            (kd_int >> 4) & 0xFF,
            ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F),
            t_int & 0xFF,
        ]
    )


@dataclass
class Feedback:
    motor_id: int
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    temperature_c: int
    error_code: int
    timestamp: float


def unpack_feedback(data: bytes) -> Feedback:
    if len(data) != 8:
        raise ValueError(f"피드백 길이가 8바이트가 아닙니다: {len(data)}")

    motor_id = data[0]
    p_int = (data[1] << 8) | data[2]
    v_int = (data[3] << 4) | (data[4] >> 4)
    t_int = ((data[4] & 0x0F) << 8) | data[5]

    return Feedback(
        motor_id=motor_id,
        position_rad=uint_to_float(p_int, P_MIN, P_MAX, 16),
        velocity_rad_s=uint_to_float(v_int, V_MIN, V_MAX, 12),
        torque_nm=uint_to_float(t_int, T_MIN, T_MAX, 12),
        temperature_c=data[6],
        error_code=data[7],
        timestamp=time.monotonic(),
    )


class AK45Controller:
    def __init__(self, interface: str, motor_id: int, event_queue: queue.Queue):
        self.interface = interface
        self.motor_id = motor_id
        self.events = event_queue

        self.sock: socket.socket | None = None
        self.running = False
        self.command_lock = threading.Lock()
        self.motion = "STOP"
        self.speed_abs = DEFAULT_SPEED_RAD_S
        self.kd = DEFAULT_KD

        self.tx_thread: threading.Thread | None = None
        self.rx_thread: threading.Thread | None = None

    def _send_frame(self, can_id: int, data: bytes) -> None:
        if self.sock is None:
            raise RuntimeError("CAN 소켓이 열려 있지 않습니다.")
        if len(data) > 8:
            raise ValueError("Classic CAN 데이터는 최대 8바이트입니다.")

        frame = struct.pack(
            "=IB3x8s",
            can_id,
            len(data),
            data.ljust(8, b"\x00"),
        )
        self.sock.send(frame)

    def start(self) -> None:
        if self.running:
            return

        try:
            socket.if_nametoindex(self.interface)
        except OSError as exc:
            raise RuntimeError(
                f"{self.interface} 인터페이스를 찾을 수 없습니다.\n"
                "먼저 SocketCAN을 설정하세요."
            ) from exc

        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.settimeout(0.10)
        self.sock.bind((self.interface,))

        # MIT Motor Mode 진입 명령을 확실하게 전달
        for _ in range(3):
            self._send_frame(self.motor_id, ENTER_MOTOR_MODE)
            time.sleep(0.02)

        self.running = True
        self.tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.tx_thread.start()
        self.rx_thread.start()
        self.events.put(("status", "CAN 연결됨 / MIT 모드 진입 명령 전송"))

    def set_parameters(self, speed_abs: float, kd: float) -> None:
        speed_abs = clamp(abs(speed_abs), 0.0, V_MAX)
        kd = clamp(kd, KD_MIN, KD_MAX)
        with self.command_lock:
            self.speed_abs = speed_abs
            self.kd = kd

    def set_motion(self, motion: str) -> None:
        if motion not in {"FORWARD", "REVERSE", "STOP"}:
            raise ValueError(f"잘못된 동작: {motion}")
        with self.command_lock:
            self.motion = motion
        self.events.put(("motion", motion))

    def _current_packet(self) -> bytes:
        with self.command_lock:
            motion = self.motion
            speed_abs = self.speed_abs
            kd = self.kd

        if motion == "FORWARD":
            velocity = FORWARD_SIGN * speed_abs
        elif motion == "REVERSE":
            velocity = -FORWARD_SIGN * speed_abs
        else:
            velocity = 0.0

        # MIT 속도 제어: KP=0, 목표 속도와 KD 사용, feed-forward torque=0
        return pack_mit_command(
            position=0.0,
            velocity=velocity,
            kp=0.0,
            kd=kd,
            torque=0.0,
        )

    def _tx_loop(self) -> None:
        period = 1.0 / TX_HZ
        next_time = time.monotonic()

        while self.running:
            try:
                self._send_frame(self.motor_id, self._current_packet())
            except Exception as exc:
                self.events.put(("error", f"CAN 송신 오류: {exc}"))
                self.running = False
                break

            next_time += period
            delay = next_time - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_time = time.monotonic()

    def _rx_loop(self) -> None:
        while self.running and self.sock is not None:
            try:
                raw = self.sock.recv(16)
            except socket.timeout:
                continue
            except Exception as exc:
                if self.running:
                    self.events.put(("error", f"CAN 수신 오류: {exc}"))
                break

            if len(raw) != 16:
                continue

            can_id, dlc, data = struct.unpack("=IB3x8s", raw)
            can_id &= 0x7FF
            data = data[:dlc]

            if can_id != self.motor_id or dlc != 8:
                continue

            try:
                feedback = unpack_feedback(data)
            except Exception as exc:
                self.events.put(("error", f"피드백 해석 오류: {exc}"))
                continue

            self.events.put(("feedback", feedback))

    def shutdown(self) -> None:
        if self.sock is None:
            return

        # 먼저 속도 0 명령으로 감속
        with self.command_lock:
            self.motion = "STOP"

        stop_packet = pack_mit_command(0.0, 0.0, 0.0, self.kd, 0.0)
        try:
            for _ in range(5):
                self._send_frame(self.motor_id, stop_packet)
                time.sleep(0.02)

            # 토크를 완전히 풀고 Motor Mode 종료
            zero_torque = pack_mit_command(0.0, 0.0, 0.0, 0.0, 0.0)
            self._send_frame(self.motor_id, zero_torque)
            time.sleep(0.02)
            self._send_frame(self.motor_id, EXIT_MOTOR_MODE)
        except Exception:
            pass

        self.running = False

        try:
            self.sock.close()
        except Exception:
            pass
        self.sock = None


class AK45GUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("CubeMars AK45-36 간단 회전 제어")
        self.root.geometry("700x470")
        self.root.minsize(650, 430)

        self.events: queue.Queue = queue.Queue()
        self.controller = AK45Controller(CAN_INTERFACE, MOTOR_ID, self.events)
        self.last_feedback_time: float | None = None

        self.speed_var = tk.DoubleVar(value=DEFAULT_SPEED_RAD_S)
        self.kd_var = tk.DoubleVar(value=DEFAULT_KD)
        self.motion_var = tk.StringVar(value="멈춤")
        self.status_var = tk.StringVar(value="연결 준비")
        self.feedback_var = tk.StringVar(value="아직 피드백 없음")

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.bind("<space>", lambda _event: self._stop())
        self.root.bind("<Right>", lambda _event: self._forward())
        self.root.bind("<Left>", lambda _event: self._reverse())

        try:
            self.controller.start()
        except Exception as exc:
            self.status_var.set(f"연결 실패: {exc}")
            messagebox.showerror(
                "CAN 연결 실패",
                f"{exc}\n\n"
                "터미널에서 아래 명령으로 can0을 먼저 올리세요:\n\n"
                "sudo ip link set can0 down\n"
                "sudo ip link set can0 type can bitrate 1000000 restart-ms 100\n"
                "sudo ip link set can0 up",
            )

        self.root.after(50, self._poll_events)
        self.root.after(250, self._update_feedback_age)

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, padding=18)
        outer.pack(fill="both", expand=True)

        title = ttk.Label(
            outer,
            text="AK45-36 KV80 · ID 11 (0x00B)",
            font=("TkDefaultFont", 17, "bold"),
        )
        title.pack(pady=(0, 12))

        info = ttk.Label(
            outer,
            text=f"SocketCAN: {CAN_INTERFACE} / 1 Mbps / MIT 명령 주기: {TX_HZ:.0f} Hz",
        )
        info.pack()

        settings = ttk.LabelFrame(outer, text="회전 설정", padding=12)
        settings.pack(fill="x", pady=14)

        ttk.Label(settings, text="속도 (rad/s)").grid(row=0, column=0, padx=8, pady=5)
        ttk.Spinbox(
            settings,
            from_=0.05,
            to=2.0,
            increment=0.05,
            textvariable=self.speed_var,
            width=10,
        ).grid(row=0, column=1, padx=8)

        ttk.Label(settings, text="KD").grid(row=0, column=2, padx=8)
        ttk.Spinbox(
            settings,
            from_=0.1,
            to=5.0,
            increment=0.1,
            textvariable=self.kd_var,
            width=10,
        ).grid(row=0, column=3, padx=8)

        ttk.Label(
            settings,
            text="기본 0.50 rad/s ≈ 4.8 rpm",
        ).grid(row=0, column=4, padx=8)

        buttons = ttk.Frame(outer)
        buttons.pack(fill="x", pady=10)

        reverse_btn = tk.Button(
            buttons,
            text="◀ 역방향 회전",
            font=("TkDefaultFont", 15, "bold"),
            height=3,
            command=self._reverse,
            bg="#4d7cff",
            fg="white",
            activebackground="#315dd2",
        )
        reverse_btn.pack(side="left", fill="x", expand=True, padx=6)

        stop_btn = tk.Button(
            buttons,
            text="■ 멈춤",
            font=("TkDefaultFont", 16, "bold"),
            height=3,
            command=self._stop,
            bg="#e84b4b",
            fg="white",
            activebackground="#bd3030",
        )
        stop_btn.pack(side="left", fill="x", expand=True, padx=6)

        forward_btn = tk.Button(
            buttons,
            text="정방향 회전 ▶",
            font=("TkDefaultFont", 15, "bold"),
            height=3,
            command=self._forward,
            bg="#38a169",
            fg="white",
            activebackground="#28794e",
        )
        forward_btn.pack(side="left", fill="x", expand=True, padx=6)

        state_box = ttk.LabelFrame(outer, text="상태", padding=12)
        state_box.pack(fill="both", expand=True, pady=(12, 0))

        ttk.Label(state_box, text="현재 명령:").grid(row=0, column=0, sticky="w", pady=4)
        ttk.Label(
            state_box,
            textvariable=self.motion_var,
            font=("TkDefaultFont", 13, "bold"),
        ).grid(row=0, column=1, sticky="w", padx=8)

        ttk.Label(state_box, text="통신 상태:").grid(row=1, column=0, sticky="w", pady=4)
        ttk.Label(state_box, textvariable=self.status_var).grid(
            row=1, column=1, sticky="w", padx=8
        )

        ttk.Label(state_box, text="모터 피드백:").grid(row=2, column=0, sticky="nw", pady=4)
        ttk.Label(
            state_box,
            textvariable=self.feedback_var,
            font=("TkFixedFont", 11),
            justify="left",
        ).grid(row=2, column=1, sticky="w", padx=8)

        ttk.Label(
            outer,
            text="키보드: ← 역방향 / Space 멈춤 / → 정방향",
        ).pack(pady=(10, 0))

    def _apply_settings(self) -> bool:
        try:
            speed = float(self.speed_var.get())
            kd = float(self.kd_var.get())
        except (ValueError, tk.TclError):
            messagebox.showerror("입력 오류", "속도와 KD 값을 숫자로 입력하세요.")
            return False

        if not (0.0 < speed <= 2.0):
            messagebox.showerror("입력 오류", "간단 시험에서는 속도를 0.05~2.0 rad/s로 설정하세요.")
            return False
        if not (0.0 < kd <= 5.0):
            messagebox.showerror("입력 오류", "KD는 0.1~5.0 범위로 설정하세요.")
            return False

        self.controller.set_parameters(speed, kd)
        return True

    def _forward(self) -> None:
        if not self.controller.running:
            messagebox.showerror("CAN 오류", "CAN이 연결되지 않았습니다.")
            return
        if self._apply_settings():
            self.controller.set_motion("FORWARD")

    def _reverse(self) -> None:
        if not self.controller.running:
            messagebox.showerror("CAN 오류", "CAN이 연결되지 않았습니다.")
            return
        if self._apply_settings():
            self.controller.set_motion("REVERSE")

    def _stop(self) -> None:
        if self.controller.running:
            self._apply_settings()
            self.controller.set_motion("STOP")

    def _poll_events(self) -> None:
        try:
            while True:
                kind, payload = self.events.get_nowait()

                if kind == "status":
                    self.status_var.set(str(payload))

                elif kind == "motion":
                    names = {
                        "FORWARD": "정방향 회전",
                        "REVERSE": "역방향 회전",
                        "STOP": "멈춤",
                    }
                    self.motion_var.set(names.get(str(payload), str(payload)))

                elif kind == "feedback":
                    fb: Feedback = payload
                    self.last_feedback_time = fb.timestamp
                    self.feedback_var.set(
                        f"ID       : {fb.motor_id}\n"
                        f"Position : {fb.position_rad:+.4f} rad\n"
                        f"Velocity : {fb.velocity_rad_s:+.4f} rad/s\n"
                        f"Torque   : {fb.torque_nm:+.3f} Nm\n"
                        f"Temp     : {fb.temperature_c} °C\n"
                        f"Error    : {fb.error_code}"
                    )
                    if fb.error_code == 0:
                        self.status_var.set("피드백 정상")
                    else:
                        self.status_var.set(f"모터 오류 코드: {fb.error_code}")

                elif kind == "error":
                    self.status_var.set(str(payload))

        except queue.Empty:
            pass

        self.root.after(50, self._poll_events)

    def _update_feedback_age(self) -> None:
        if self.controller.running and self.last_feedback_time is not None:
            age = time.monotonic() - self.last_feedback_time
            if age > 1.0:
                self.status_var.set(f"피드백 지연: {age:.1f}초")

        self.root.after(250, self._update_feedback_age)

    def _on_close(self) -> None:
        self.motion_var.set("종료 중")
        self.controller.shutdown()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    AK45GUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
