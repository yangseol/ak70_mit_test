#!/usr/bin/env python3
"""AK45-36 KV80 software-zero and session homing helper GUI."""

from __future__ import annotations

import argparse
import math
import queue
import sys
import threading
import time
from dataclasses import dataclass
from tkinter import messagebox, ttk
import tkinter as tk

import can

from ak45_calibration import (
    AK45_IDS,
    calibration_exists,
    get_ak45_entry,
    joint_position_from_calibration,
    load_ak45_calibration,
    record_power_cycle_verified,
    save_software_zero,
    set_direction_sign,
)
from homing_state import HomingMachine, HomingState
from mixed_mit_packet import pack_mit_command, unpack_mit_feedback
from motor_profiles import format_motor_id


DEFAULT_CHANNEL = "can0"
ZERO_TORQUE_REQUEST = pack_mit_command(0x00B, 0.0, 0.0, 0.0, 0.0, 0.0)


@dataclass
class MotorRow:
    motor_id: int
    raw_pos_rad: float | None = None
    joint_deg: float | None = None
    detected: bool = False


def read_one_position(channel: str, motor_id: int, timeout_sec: float = 0.8) -> float:
    bus = can.Bus(interface="socketcan", channel=channel)
    try:
        bus.send(can.Message(arbitration_id=motor_id, data=pack_mit_command(motor_id, 0.0, 0.0, 0.0, 0.0, 0.0), is_extended_id=False))
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            msg = bus.recv(timeout=0.02)
            if msg is None or msg.arbitration_id != motor_id or len(msg.data) != 8:
                continue
            try:
                feedback = unpack_mit_feedback(motor_id, bytes(msg.data))
            except ValueError:
                continue
            return feedback.position
    finally:
        bus.shutdown()
    raise TimeoutError(f"{format_motor_id(motor_id)} feedback timeout")


class AK45CalibrationHelper:
    def __init__(self, channel: str) -> None:
        self.channel = channel
        self.root = tk.Tk()
        self.root.title("AK45-36 KV80 Software Zero / Homing")
        self.root.geometry("980x600")
        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.rows = {motor_id: MotorRow(motor_id) for motor_id in AK45_IDS}
        self.homing = {motor_id: HomingMachine(motor_id) for motor_id in AK45_IDS}
        self.selected_id = tk.StringVar(value=format_motor_id(0x00B))
        self.direction_var = tk.IntVar(value=1)
        self.status_var = tk.StringVar(value="대기")
        self._build()
        self.refresh_table()
        self.root.after(100, self.process_events)

    def _build(self) -> None:
        top = ttk.Frame(self.root)
        top.pack(fill="x", padx=10, pady=8)
        ttk.Label(top, text="channel").pack(side="left")
        ttk.Label(top, text=self.channel).pack(side="left", padx=6)
        ttk.Label(top, text="motor").pack(side="left", padx=(20, 4))
        combo = ttk.Combobox(top, textvariable=self.selected_id, values=[format_motor_id(i) for i in AK45_IDS], width=10, state="readonly")
        combo.pack(side="left")
        ttk.Label(top, text="direction_sign").pack(side="left", padx=(20, 4))
        ttk.Radiobutton(top, text="+1", variable=self.direction_var, value=1, command=self.save_direction).pack(side="left")
        ttk.Radiobutton(top, text="-1", variable=self.direction_var, value=-1, command=self.save_direction).pack(side="left")

        buttons = ttk.Frame(self.root)
        buttons.pack(fill="x", padx=10, pady=4)
        for text, command in [
            ("전체 감지", self.detect_all),
            ("선택 위치 읽기", self.read_selected),
            ("현재 자세를 Software Zero로 저장", self.capture_zero),
            ("수평 자세 확인", self.confirm_home),
            ("Homing 해제", self.clear_home),
            ("전원 재인가 비교 기록", self.record_power_cycle),
            ("저장값 확인", self.refresh_table),
            ("닫기", self.root.destroy),
        ]:
            ttk.Button(buttons, text=text, command=command).pack(side="left", padx=3)

        self.tree = ttk.Treeview(
            self.root,
            columns=("id", "name", "model", "detected", "raw", "zero", "sign", "joint", "power", "homing"),
            show="headings",
            height=11,
        )
        for col, text, width in [
            ("id", "ID", 80),
            ("name", "name", 150),
            ("model", "model", 130),
            ("detected", "detected", 90),
            ("raw", "current raw rad", 130),
            ("zero", "software zero rad", 145),
            ("sign", "sign", 60),
            ("joint", "joint deg", 100),
            ("power", "power cycle", 100),
            ("homing", "HomingState", 140),
        ]:
            self.tree.heading(col, text=text)
            self.tree.column(col, width=width, anchor="center")
        self.tree.pack(fill="both", expand=True, padx=10, pady=8)
        self.tree.bind("<<TreeviewSelect>>", self.on_select)

        ttk.Label(
            self.root,
            text="AK45-36 전원: 정격 24V, 허용 16~28V, 48V 연결 금지. AK70과 전원 레일은 분리하고 CAN 기준 GND는 공유하십시오.",
            foreground="#a33",
        ).pack(anchor="w", padx=10, pady=4)
        ttk.Label(self.root, textvariable=self.status_var).pack(anchor="w", padx=10, pady=4)

    def selected_motor_id(self) -> int:
        return int(self.selected_id.get(), 16)

    def on_select(self, _event: tk.Event) -> None:
        selected = self.tree.selection()
        if selected:
            self.selected_id.set(selected[0])
            self.sync_direction_var()

    def sync_direction_var(self) -> None:
        try:
            entry = get_ak45_entry(self.selected_motor_id())
            self.direction_var.set(int(entry.get("direction_sign", 1)))
        except Exception:
            self.direction_var.set(1)

    def refresh_table(self) -> None:
        calibration = load_ak45_calibration()
        self.tree.delete(*self.tree.get_children())
        for motor_id in AK45_IDS:
            key = format_motor_id(motor_id)
            entry = calibration["motors"][key]
            row = self.rows[motor_id]
            raw = row.raw_pos_rad
            joint = None
            if raw is not None and calibration_exists(motor_id, calibration):
                try:
                    joint = math.degrees(joint_position_from_calibration(motor_id, raw, calibration))
                except Exception:
                    joint = None
            row.joint_deg = joint
            self.tree.insert(
                "",
                "end",
                iid=key,
                values=(
                    key,
                    entry.get("name", ""),
                    entry.get("model", ""),
                    "YES" if row.detected else "NO",
                    f"{raw:+.6f}" if raw is not None else "N/A",
                    entry.get("raw_zero_pos_rad") if entry.get("raw_zero_pos_rad") is not None else "N/A",
                    entry.get("direction_sign", 1),
                    f"{joint:+.3f}" if joint is not None else "N/A",
                    "YES" if entry.get("power_cycle_verified") else "NO",
                    self.homing[motor_id].state.value,
                ),
            )
        self.sync_direction_var()

    def run_background(self, func) -> None:
        threading.Thread(target=func, daemon=True).start()

    def detect_all(self) -> None:
        def work() -> None:
            for motor_id in AK45_IDS:
                try:
                    pos = read_one_position(self.channel, motor_id)
                    self.events.put(("position", (motor_id, pos)))
                except Exception as exc:
                    self.events.put(("log", f"{format_motor_id(motor_id)} 감지 실패: {exc}"))
        self.run_background(work)

    def read_selected(self) -> None:
        motor_id = self.selected_motor_id()
        self.run_background(lambda: self.events.put(("position", (motor_id, read_one_position(self.channel, motor_id)))))

    def capture_zero(self) -> None:
        motor_id = self.selected_motor_id()
        row = self.rows[motor_id]
        if row.raw_pos_rad is None:
            messagebox.showerror("위치 없음", "먼저 선택 위치를 읽으십시오.")
            return
        if not messagebox.askokcancel("Software Zero 저장", "현재 raw 위치를 AK45 software zero로 저장합니다. 모터는 움직이지 않습니다."):
            return
        save_software_zero(motor_id, row.raw_pos_rad)
        self.homing[motor_id].on_calibration_changed()
        self.status_var.set(f"{format_motor_id(motor_id)} software zero 저장 완료")
        self.refresh_table()

    def confirm_home(self) -> None:
        motor_id = self.selected_motor_id()
        row = self.rows[motor_id]
        if row.raw_pos_rad is None:
            messagebox.showerror("위치 없음", "먼저 선택 위치를 읽으십시오.")
            return
        if not messagebox.askokcancel(
            "수평 자세 확인",
            "발목이 실제 수평 기준면 또는 기계적 지그에 고정되어 있는지 확인하십시오.\n센서값만으로 수평 자세를 확인할 수 없습니다.",
        ):
            return
        calibration = load_ak45_calibration()
        state = self.homing[motor_id].confirm_horizontal_pose(row.raw_pos_rad, calibration)
        if state != HomingState.HOMED:
            messagebox.showerror("Homing 실패", "AK45 calibration이 없거나 저장값이 유효하지 않습니다.")
        self.refresh_table()

    def clear_home(self) -> None:
        self.homing[self.selected_motor_id()].clear_homing("user_clear")
        self.refresh_table()

    def record_power_cycle(self) -> None:
        motor_id = self.selected_motor_id()
        if not messagebox.askokcancel("전원 재인가 비교 기록", "현재 모터의 전원 재인가 비교가 수동으로 완료되었음을 기록합니다."):
            return
        record_power_cycle_verified(motor_id)
        self.refresh_table()

    def save_direction(self) -> None:
        try:
            set_direction_sign(self.selected_motor_id(), int(self.direction_var.get()))
            self.homing[self.selected_motor_id()].on_calibration_changed()
            self.refresh_table()
        except Exception as exc:
            messagebox.showerror("direction_sign 오류", str(exc))

    def process_events(self) -> None:
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "position":
                    motor_id, pos = payload
                    self.rows[motor_id].raw_pos_rad = float(pos)
                    self.rows[motor_id].detected = True
                    self.status_var.set(f"{format_motor_id(motor_id)} 위치 읽기 완료")
                    self.refresh_table()
                elif kind == "log":
                    self.status_var.set(str(payload))
        except queue.Empty:
            pass
        self.root.after(100, self.process_events)

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser(description="AK45 software-zero and session homing helper GUI.")
    parser.add_argument("--channel", default=DEFAULT_CHANNEL)
    args = parser.parse_args()
    AK45CalibrationHelper(args.channel).run()


if __name__ == "__main__":
    main()

