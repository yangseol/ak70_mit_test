"""AK70-10 통합 제어 센터 GUI."""

from __future__ import annotations

import datetime as dt
import json
import math
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from tkinter import BooleanVar, StringVar, filedialog, messagebox, simpledialog
from tkinter import ttk
import tkinter as tk
from typing import Callable, Any

import can
import yaml

from mit_packet import pack_mit_command
from realtime_ipc import (
    DEFAULT_LOCK_PATH,
    DEFAULT_SOCKET_PATH,
    ControllerAlreadyRunning,
    acquire_controller_lock,
    probe_datagram_socket,
    send_request,
)


PROJECT_DIR = Path(__file__).resolve().parent
CONFIG_DIR = Path.home() / ".config" / "ak70_control_center"
PRESET_PATH = CONFIG_DIR / "presets.json"
CALIBRATION_PATH = PROJECT_DIR / "motor_calibration.yaml"
AK45_CALIBRATION_PATH = PROJECT_DIR / "ak45_motor_calibration.yaml"
DEFAULT_CHANNEL = "can0"
AK70_IDS = [f"0x{i:03X}" for i in range(1, 11)]
AK45_IDS = [f"0x{i:03X}" for i in range(0x00B, 0x00E)]
SUPPORTED_IDS = AK70_IDS + AK45_IDS
EXPECTED_ZERO_TORQUE_PACKET = bytes([0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x00])
GUI_HEARTBEAT_INTERVAL_MS = 250
IPC_TIMEOUT_SEC = 0.25
IPC_RETRY_DELAY_SEC = 0.1
RELEASE_CONFIRM_TEXT = (
    "선택한 모터의 힘이 즉시 풀립니다.\n\n"
    "모터가 로봇에 장착된 상태라면 관절이나 구조물이\n"
    "중력에 의해 갑자기 움직이거나 떨어질 수 있습니다.\n\n"
    "로봇과 구조물을 손으로 지지하거나 고정했는지\n"
    "확인한 후 진행하십시오.\n\n"
    "정말 선택한 모터의 TORQUE를 해제하시겠습니까?"
)


SCRIPTS = {
    "detect_multi": "detect_multi_motors_once.py",
    "read_single": "read_single_joint_once.py",
    "read_multi": "read_multi_joints_once.py",
    "move_once": "move_joint_once.py",
    "hold": "hold_joint_position.py",
    "trajectory": "run_joint_trajectory.py",
    "multi_pose": "run_multi_joint_pose.py",
    "mixed_pose": "run_mixed_joint_pose.py",
    "calibration_helper": "calibration_helper_app.py",
    "ak45_calibration_helper": "ak45_calibration_helper_gui.py",
    "motor_manager": "ak70_motor_manager_gui.py",
    "realtime_controller": "run_realtime_controller.py",
    "realtime_targets": "send_realtime_targets.py",
    "can_health": "check_can_health.py",
}


DETECT_RE = re.compile(
    r"ID:\s*(0x[0-9A-Fa-f]+)\s*\|\s*calibrated:\s*(YES|NO)\s*\|\s*"
    r"raw_pos_rad:\s*([+-]?[0-9.]+|N/A)\s*\|\s*"
    r"joint_rad:\s*([+-]?[0-9.]+|N/A)\s*\|\s*"
    r"joint_deg:\s*([+-]?[0-9.]+|N/A)"
)
READ_MULTI_RE = re.compile(
    r"(0x[0-9A-Fa-f]+)\s*\|.*joint_deg:\s*([+-]?[0-9.]+)"
)
READ_SINGLE_RE = re.compile(r"joint_rad:\s*[+-]?[0-9.]+\s*\|\s*joint_deg:\s*([+-]?[0-9.]+)")
ACTUAL_TABLE_HEADER_RE = re.compile(r"\bID\b.*\bActual\b")
ACTUAL_TABLE_ROW_RE = re.compile(
    r"^\s*(0x[0-9A-Fa-f]+)\s+"
    r"[+-]?[0-9.]+\s*deg\s+"
    r"([+-]?[0-9.]+)\s*deg\b"
)
SINGLE_ACTUAL_RES = [
    re.compile(r"\bAfter:\s*([+-]?[0-9.]+)\s*deg\b"),
    re.compile(r"\bHeld position:\s*([+-]?[0-9.]+)\s*deg\b"),
    re.compile(r"\bFinal actual position:\s*([+-]?[0-9.]+)\s*deg\b"),
    re.compile(r"\bactual(?: position)?:\s*([+-]?[0-9.]+)\s*deg\b"),
    re.compile(r"\bjoint:\s*([+-]?[0-9.]+)\s*deg\b"),
]


@dataclass
class CanStatus:
    interface_state: str = "UNKNOWN"
    can_state: str = "UNKNOWN"
    bitrate: str = "N/A"
    tx_error: str = "N/A"
    rx_error: str = "N/A"
    restart_ms: str = "N/A"


@dataclass
class MotorInfo:
    motor_id: str
    detected: bool = False
    calibrated: bool = False
    joint_deg: float | None = None
    last_seen: str = ""


@dataclass
class AutoResponse:
    prompt: str
    response: str
    label: str
    sent: bool = False


@dataclass
class IpcRequest:
    payload: dict[str, Any]
    callback: Callable[[dict[str, Any]], None] | None = None
    retries: int = 0
    retry_delay_sec: float = IPC_RETRY_DELAY_SEC


class CloseState:
    RUNNING = "RUNNING"
    CLOSE_STATUS_PENDING = "CLOSE_STATUS_PENDING"
    CLOSE_CONFIRM_PENDING = "CLOSE_CONFIRM_PENDING"
    CLOSE_RELEASE_PENDING = "CLOSE_RELEASE_PENDING"
    CLOSE_PROCESS_WAIT = "CLOSE_PROCESS_WAIT"
    CLOSED = "CLOSED"


def close_command_for_ownership(process_owner: bool, session_owner: bool) -> str | None:
    if process_owner and session_owner:
        return "SHUTDOWN"
    if session_owner:
        return "RELEASE_SESSION"
    return None


def now_text() -> str:
    return dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def normalize_motor_id(value: str) -> str:
    text = value.strip()
    if not text:
        raise ValueError("모터 ID가 비어 있습니다.")
    try:
        if text.lower().startswith("0x"):
            motor_id = int(text, 16)
        else:
            motor_id = int(text, 10)
    except ValueError as exc:
        raise ValueError(f"잘못된 모터 ID: {value}") from exc
    if not 0x001 <= motor_id <= 0x00D:
        raise ValueError("지원 모터 ID는 0x001~0x00D 범위만 허용됩니다.")
    return f"0x{motor_id:03X}"


def model_for_motor_id(motor_id: str) -> str:
    value = int(normalize_motor_id(motor_id), 16)
    if 0x001 <= value <= 0x00A:
        return "AK70-10"
    if 0x00B <= value <= 0x00D:
        return "AK45-36 KV80"
    return "N/A"


def is_ak45_id(motor_id: str) -> bool:
    return normalize_motor_id(motor_id) in AK45_IDS


def parse_finite_float(text: str, label: str) -> float:
    try:
        value = float(text.strip())
    except ValueError as exc:
        raise ValueError(f"{label}: 숫자가 아닙니다.") from exc
    if not math.isfinite(value):
        raise ValueError(f"{label}: NaN/Infinity는 허용되지 않습니다.")
    return value


def script_path(name: str) -> Path:
    return PROJECT_DIR / SCRIPTS[name]


def script_exists(name: str) -> bool:
    return script_path(name).exists()


def command_text(command: list[str]) -> str:
    return " ".join(command)


class PresetManager:
    """다중 pose 프리셋을 사용자 config 경로에 저장한다."""

    def __init__(self) -> None:
        self.path = PRESET_PATH
        self.presets: dict[str, dict[str, Any]] = {}
        self.load()

    def load(self) -> None:
        CONFIG_DIR.mkdir(parents=True, exist_ok=True)
        if not self.path.exists():
            self.presets = {}
            return
        try:
            with self.path.open("r", encoding="utf-8") as f:
                data = json.load(f)
            self.presets = data if isinstance(data, dict) else {}
        except Exception:
            self.presets = {}

    def save(self) -> None:
        CONFIG_DIR.mkdir(parents=True, exist_ok=True)
        with self.path.open("w", encoding="utf-8") as f:
            json.dump(self.presets, f, ensure_ascii=False, indent=2)

    def names(self) -> list[str]:
        return sorted(self.presets)


class CanStatusReader:
    """ip 명령으로 can 인터페이스 상태를 읽는다."""

    def read(self, channel: str) -> tuple[CanStatus, str]:
        try:
            result = subprocess.run(
                ["ip", "-details", "link", "show", channel],
                cwd=PROJECT_DIR,
                shell=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=3.0,
                check=False,
            )
        except FileNotFoundError:
            return CanStatus(interface_state="NOT FOUND", can_state="NOT FOUND"), "ip 명령을 찾을 수 없습니다."
        except Exception as exc:
            return CanStatus(interface_state="ERROR", can_state="ERROR"), str(exc)

        output = result.stdout or ""
        if result.returncode != 0:
            return CanStatus(interface_state="NOT FOUND", can_state="NOT FOUND"), output.strip()

        status = CanStatus()
        state_match = re.search(r"\bstate\s+(\S+)", output)
        if state_match:
            status.interface_state = state_match.group(1)
        can_match = re.search(r"\bcan\s+state\s+([A-Z-]+)", output)
        if can_match:
            status.can_state = can_match.group(1)
        bitrate_match = re.search(r"\bbitrate\s+(\d+)", output)
        if bitrate_match:
            status.bitrate = bitrate_match.group(1)
        restart_match = re.search(r"\brestart-ms\s+(\d+)", output)
        if restart_match:
            status.restart_ms = restart_match.group(1)
        berr_match = re.search(r"berr-counter\s+tx\s+(\d+)\s+rx\s+(\d+)", output)
        if berr_match:
            status.tx_error = berr_match.group(1)
            status.rx_error = berr_match.group(2)
        if status.interface_state == "DOWN":
            status.can_state = "DOWN"
        return status, output.strip()


class ProcessManager:
    """subprocess 실행, 실시간 로그, CLI 게이트 자동 입력을 관리한다."""

    def __init__(
        self,
        app: "AK70ControlCenterApp",
        log_queue: queue.Queue[tuple[str, Any]],
    ) -> None:
        self.app = app
        self.log_queue = log_queue
        self.active_process: subprocess.Popen[str] | None = None
        self.active_task_name = ""
        self.active_motor_ids: list[str] = []
        self.lock = threading.Lock()

    def is_running(self) -> bool:
        with self.lock:
            return self.active_process is not None and self.active_process.poll() is None

    def start(
        self,
        task_name: str,
        command: list[str],
        motor_ids: list[str],
        auto_responses: list[AutoResponse] | None = None,
        on_output: Callable[[str], None] | None = None,
        on_finish: Callable[[int], None] | None = None,
    ) -> bool:
        with self.lock:
            if self.active_process is not None and self.active_process.poll() is None:
                messagebox.showerror("실행 중", "이미 실행 중인 모터 제어 작업이 있습니다.")
                return False
            try:
                process = subprocess.Popen(
                    command,
                    cwd=PROJECT_DIR,
                    shell=False,
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                )
            except Exception as exc:
                self.log_queue.put(("log", f"[오류] subprocess 시작 실패: {exc}\n"))
                messagebox.showerror("실행 실패", str(exc))
                return False

            self.active_process = process
            self.active_task_name = task_name
            self.active_motor_ids = motor_ids

        self.log_queue.put(("status", ("실행 중", task_name)))
        self.log_queue.put(("log", f"\n[{now_text()}] [작업 시작] {task_name}\n"))
        self.log_queue.put(("log", f"[실행 명령] {command_text(command)}\n"))
        if "--enter-mit" in command:
            self.log_queue.put(("log", "[MIT 모드] 실제 모터 이동 전 자동 진입 사용\n"))
        self.log_queue.put(("log", "[GUI 승인 여부] 승인됨\n"))
        if auto_responses:
            self.log_queue.put(("log", "[CLI 게이트 자동 처리] 대기 중\n"))

        thread = threading.Thread(
            target=self._reader_thread,
            args=(process, auto_responses or [], on_output, on_finish),
            daemon=True,
        )
        thread.start()
        return True

    def _reader_thread(
        self,
        process: subprocess.Popen[str],
        auto_responses: list[AutoResponse],
        on_output: Callable[[str], None] | None,
        on_finish: Callable[[int], None] | None,
    ) -> None:
        prompt_buffer = ""
        try:
            assert process.stdout is not None
            while True:
                ch = process.stdout.read(1)
                if ch == "":
                    break
                self.log_queue.put(("log", ch))
                if on_output is not None:
                    self.log_queue.put(("parse", (on_output, ch)))
                prompt_buffer = (prompt_buffer + ch)[-500:]
                for response in auto_responses:
                    if not response.sent and response.prompt in prompt_buffer:
                        self._send_auto_response(process, response)
        except Exception as exc:
            self.log_queue.put(("log", f"\n[stdout 읽기 오류] {exc}\n"))

        return_code = process.wait()
        self.log_queue.put(("log", f"\n[종료 코드] {return_code}\n"))
        self.log_queue.put(("log", f"[종료 시간] {now_text()}\n"))
        self.log_queue.put(("finished", (return_code, on_finish)))

    def _send_auto_response(self, process: subprocess.Popen[str], response: AutoResponse) -> None:
        try:
            if process.stdin is None:
                return
            process.stdin.write(response.response + "\n")
            process.stdin.flush()
            response.sent = True
            self.log_queue.put(("log", f"\n[CLI 게이트 자동 처리] {response.label}\n"))
        except Exception as exc:
            self.log_queue.put(("log", f"\n[CLI 게이트 자동 입력 실패] {exc}\n"))

    def clear_active(self, return_code: int) -> None:
        with self.lock:
            self.active_process = None
            self.active_task_name = ""
            self.active_motor_ids = []
        state = "정상 완료" if return_code == 0 else "오류"
        self.log_queue.put(("status", (state, "대기")))

    def emergency_stop(self) -> None:
        with self.lock:
            process = self.active_process
            motor_ids = list(self.active_motor_ids)
        if process is not None and process.poll() is None:
            self.log_queue.put(("log", "\n[비상 정지] SIGINT 전송\n"))
            try:
                process.send_signal(signal.SIGINT)
            except Exception as exc:
                self.log_queue.put(("log", f"[비상 정지] SIGINT 실패: {exc}\n"))
            self._wait_or_escalate(process)
        if motor_ids:
            self.release_motors_best_effort(motor_ids)
        self.log_queue.put(("status", ("사용자 중단", "대기")))

    def _wait_or_escalate(self, process: subprocess.Popen[str]) -> None:
        try:
            process.wait(timeout=3.0)
            return
        except subprocess.TimeoutExpired:
            self.log_queue.put(("log", "[비상 정지] terminate 시도\n"))
            try:
                process.terminate()
                process.wait(timeout=2.0)
                return
            except Exception:
                self.log_queue.put(("log", "[비상 정지] kill 시도\n"))
                try:
                    process.kill()
                except Exception as exc:
                    self.log_queue.put(("log", f"[비상 정지] kill 실패: {exc}\n"))

    def release_motors_best_effort(self, motor_ids: list[str]) -> None:
        self.log_queue.put(("log", "[CONTROLLER_UNREACHABLE] 직접 fallback 조건 검증 시작\n"))
        self.app.direct_zero_torque_fallback(motor_ids, source="ProcessManager")


class BasePage(ttk.Frame):
    def __init__(self, app: "AK70ControlCenterApp", parent: ttk.Frame) -> None:
        super().__init__(parent)
        self.app = app

    def on_show(self) -> None:
        pass


class HomePage(BasePage):
    def __init__(self, app: "AK70ControlCenterApp", parent: ttk.Frame) -> None:
        super().__init__(app, parent)
        ttk.Label(self, text="AK70 통합 제어 센터", font=("TkDefaultFont", 22, "bold")).pack(anchor="w", padx=20, pady=20)
        self.summary = ttk.Label(self, text="", justify="left")
        self.summary.pack(anchor="w", padx=20, pady=10)
        cards = ttk.Frame(self)
        cards.pack(fill="x", padx=20, pady=20)
        for idx, (title, page) in enumerate(
            [
                ("통신 연결", "통신 연결"),
                ("원점 관리", "원점 / 캘리브레이션"),
                ("단일 모터", "단일 모터 제어"),
                ("다중 모터", "다중 모터 제어"),
                ("로그 및 안전", "로그 / 안전"),
            ]
        ):
            button = ttk.Button(cards, text=title, command=lambda p=page: app.show_page(p))
            button.grid(row=idx // 3, column=idx % 3, padx=10, pady=10, ipadx=40, ipady=20, sticky="nsew")
        for col in range(3):
            cards.columnconfigure(col, weight=1)
        ttk.Label(self, text="홈 화면에서는 모터를 직접 움직이지 않습니다.").pack(anchor="w", padx=20, pady=10)

    def on_show(self) -> None:
        self.summary.configure(
            text=(
                f"CAN 상태: {self.app.can_status.can_state}\n"
                f"최근 감지된 모터: {len(self.app.detected_motors)}개\n"
                f"calibration 등록 모터 수: {len(self.app.calibration_ids)}개\n"
                f"현재 실행 중 작업: {self.app.current_task_var.get()}\n"
                f"최근 로그: 아래 로그 패널 또는 로그 / 안전 화면에서 확인"
            )
        )


class CommunicationPage(BasePage):
    def __init__(self, app: "AK70ControlCenterApp", parent: ttk.Frame) -> None:
        super().__init__(app, parent)
        ttk.Label(self, text="통신 연결", font=("TkDefaultFont", 16, "bold")).pack(anchor="w", padx=12, pady=8)
        controls = ttk.Frame(self)
        controls.pack(fill="x", padx=12, pady=4)
        ttk.Button(controls, text="CAN 상태 새로고침", command=app.refresh_can_status).pack(side="left", padx=4)
        ttk.Button(controls, text="전체 AK70 감지", command=self.detect_all).pack(side="left", padx=4)
        ttk.Button(controls, text="AK45 원점 설정 도구 열기", command=self.open_ak45_helper).pack(side="left", padx=4)
        ttk.Button(controls, text="CAN 상태 확인", command=self.run_can_health).pack(side="left", padx=4)
        ttk.Button(controls, text="선택 모터 감지", command=self.detect_selected).pack(side="left", padx=4)
        ttk.Button(controls, text="선택 모터 위치 읽기", command=self.read_selected).pack(side="left", padx=4)
        ttk.Button(controls, text="전체 위치 읽기", command=self.read_all_detected).pack(side="left", padx=4)
        ttk.Button(controls, text="전체 선택", command=self.select_all).pack(side="left", padx=4)
        ttk.Button(controls, text="전체 해제", command=self.clear_selection).pack(side="left", padx=4)
        ttk.Button(controls, text="감지 모터만 선택", command=self.select_detected).pack(side="left", padx=4)
        self.selected_summary_var = StringVar(value="선택된 모터: 없음")
        ttk.Label(self, textvariable=self.selected_summary_var).pack(anchor="w", padx=12, pady=4)
        self.can_text = tk.Text(self, height=8, wrap="word")
        self.can_text.pack(fill="x", padx=12, pady=6)
        self.tree = ttk.Treeview(
            self,
            columns=("selected", "id", "model", "detected", "calibrated", "homing", "joint_deg", "time"),
            show="headings",
            height=13,
        )
        for col, text, width in [
            ("selected", "선택", 60),
            ("id", "ID", 90),
            ("model", "Model", 130),
            ("detected", "연결 상태", 100),
            ("calibrated", "calibration 상태", 130),
            ("homing", "AK45 HomingState", 150),
            ("joint_deg", "현재 joint_deg", 140),
            ("time", "최근 확인 시간", 180),
        ]:
            self.tree.heading(col, text=text)
            self.tree.column(col, width=width, anchor="center")
        self.tree.pack(fill="both", expand=True, padx=12, pady=6)
        self.tree.bind("<Button-1>", self.on_tree_click)
        self.tree.bind("<Double-1>", self.on_tree_double_click)
        self.tree.bind("<space>", self.on_space)

    def on_show(self) -> None:
        self.refresh_table()
        self.update_can_text()

    def update_can_text(self) -> None:
        status = self.app.can_status
        text = (
            f"interface state: {status.interface_state}\n"
            f"CAN state: {status.can_state}\n"
            f"bitrate: {status.bitrate}\n"
            f"tx error counter: {status.tx_error}\n"
            f"rx error counter: {status.rx_error}\n"
            f"restart-ms: {status.restart_ms}\n"
        )
        self.can_text.delete("1.0", "end")
        self.can_text.insert("end", text)

    def refresh_table(self) -> None:
        for item in self.tree.get_children():
            self.tree.delete(item)
        for motor_id in SUPPORTED_IDS:
            info = self.app.motor_infos.get(motor_id, MotorInfo(motor_id=motor_id))
            self.tree.insert(
                "",
                "end",
                iid=motor_id,
                values=(
                    "☑" if motor_id in self.app.selected_communication_motor_ids else "☐",
                    motor_id,
                    model_for_motor_id(motor_id),
                    "연결" if info.detected else "미확인",
                    "등록" if info.calibrated else "없음",
                    self.app.ak45_homing_states.get(motor_id, "N/A") if is_ak45_id(motor_id) else "N/A",
                    f"{info.joint_deg:+.2f}" if info.joint_deg is not None else "N/A",
                    info.last_seen or "N/A",
                ),
            )
        self.update_selected_summary()

    def refresh_current_angles(self) -> None:
        if not self.tree.get_children():
            self.refresh_table()
            return
        for motor_id in SUPPORTED_IDS:
            if not self.tree.exists(motor_id):
                continue
            info = self.app.motor_infos.get(motor_id, MotorInfo(motor_id=motor_id))
            current = self.app.current_positions.get(motor_id)
            if current is not None:
                info.joint_deg = current
            self.tree.set(motor_id, "selected", "☑" if motor_id in self.app.selected_communication_motor_ids else "☐")
            self.tree.set(motor_id, "model", model_for_motor_id(motor_id))
            self.tree.set(motor_id, "detected", "연결" if info.detected else "미확인")
            self.tree.set(motor_id, "homing", self.app.ak45_homing_states.get(motor_id, "N/A") if is_ak45_id(motor_id) else "N/A")
            self.tree.set(motor_id, "joint_deg", f"{info.joint_deg:+.2f}" if info.joint_deg is not None else "N/A")
            self.tree.set(motor_id, "time", info.last_seen or "N/A")
        self.update_selected_summary()

    def update_selected_summary(self) -> None:
        ids = sorted(self.app.selected_communication_motor_ids)
        text = ", ".join(ids) if ids else "없음"
        self.selected_summary_var.set(f"선택된 모터: {text}")

    def toggle_motor_selection(self, motor_id: str) -> None:
        if motor_id in self.app.selected_communication_motor_ids:
            self.app.selected_communication_motor_ids.remove(motor_id)
        else:
            self.app.selected_communication_motor_ids.add(motor_id)
        self.refresh_table()

    def on_tree_click(self, event: tk.Event) -> None:
        if self.tree.identify_column(event.x) != "#1":
            return
        item = self.tree.identify_row(event.y)
        if item:
            self.toggle_motor_selection(item)

    def on_tree_double_click(self, event: tk.Event) -> None:
        item = self.tree.identify_row(event.y)
        if item:
            self.toggle_motor_selection(item)

    def on_space(self, _event: tk.Event) -> None:
        for item in self.tree.selection():
            self.toggle_motor_selection(item)

    def select_all(self) -> None:
        self.app.selected_communication_motor_ids = set(SUPPORTED_IDS)
        self.refresh_table()

    def clear_selection(self) -> None:
        self.app.selected_communication_motor_ids.clear()
        self.refresh_table()

    def select_detected(self) -> None:
        self.app.selected_communication_motor_ids = set(self.app.detected_motors)
        self.refresh_table()

    def selected_ids_or_warn(self) -> list[str]:
        ids = sorted(self.app.selected_communication_motor_ids)
        if not ids:
            messagebox.showwarning("선택 없음", "통신 표에서 모터를 하나 이상 선택하십시오.")
        return ids

    def detect_all(self) -> None:
        if not script_exists("detect_multi"):
            self.app.show_missing_script("detect_multi")
            return
        active = self.app.active_realtime_ids()
        ids = [motor_id for motor_id in AK70_IDS if motor_id not in active]
        if active:
            self.app.append_log(f"[감지 제외] controller active ID: {', '.join(sorted(active))}\n")
        if not ids:
            messagebox.showinfo("감지 생략", "모든 AK70 대상이 controller active 상태입니다. STATUS 값을 사용합니다.")
            return
        command = [
            sys.executable,
            str(script_path("detect_multi")),
            "--channel",
            self.app.channel_var.get(),
            "--motor-ids",
            ",".join(ids),
            "--yes",
        ]
        self.app.process_manager.start("전체 AK70 감지", command, [], on_output=self.app.parse_process_output)

    def open_ak45_helper(self) -> None:
        self.app.open_ak45_helper()

    def run_can_health(self) -> None:
        self.app.run_can_health()

    def detect_selected(self) -> None:
        if not script_exists("detect_multi"):
            self.app.show_missing_script("detect_multi")
            return
        ids = self.selected_ids_or_warn()
        if not ids:
            return
        active = self.app.active_realtime_ids()
        ids = [motor_id for motor_id in ids if motor_id not in active]
        if active:
            self.app.append_log(f"[감지 제외] controller active ID: {', '.join(sorted(active))}\n")
        if not ids:
            messagebox.showinfo("감지 생략", "선택한 모터가 controller active 상태입니다. STATUS 값을 사용합니다.")
            return
        command = [
            sys.executable,
            str(script_path("detect_multi")),
            "--channel",
            self.app.channel_var.get(),
            "--motor-ids",
            ",".join(ids),
            "--yes",
        ]
        self.app.process_manager.start("선택 ID 감지", command, ids, on_output=self.app.parse_process_output)

    def read_selected(self) -> None:
        ids = self.selected_ids_or_warn()
        if not ids:
            return
        self.app.run_read_positions(ids)

    def read_all_detected(self) -> None:
        ids = sorted(self.app.detected_motors) or sorted(self.app.calibration_ids)
        if not ids:
            messagebox.showwarning("대상 없음", "감지 모터 또는 calibration 등록 모터가 없습니다.")
            return
        self.app.run_read_positions(ids)


class CalibrationPage(BasePage):
    def __init__(self, app: "AK70ControlCenterApp", parent: ttk.Frame) -> None:
        super().__init__(app, parent)
        ttk.Label(self, text="원점 / 캘리브레이션", font=("TkDefaultFont", 16, "bold")).pack(anchor="w", padx=12, pady=8)
        ttk.Label(self, text="원점 저장은 기존 검증 앱에서만 실행됩니다. 통합 앱에서는 calibration 파일을 수정하지 않습니다.").pack(
            anchor="w", padx=12, pady=4
        )
        buttons = ttk.Frame(self)
        buttons.pack(fill="x", padx=12, pady=4)
        ttk.Button(buttons, text="calibration 새로고침", command=self.refresh).pack(side="left", padx=4)
        ttk.Button(buttons, text="원점 저장 도우미 열기", command=lambda: self.open_external("calibration_helper")).pack(side="left", padx=4)
        ttk.Button(buttons, text="AK45 원점 설정 도구 열기", command=lambda: self.open_external("ak45_calibration_helper")).pack(side="left", padx=4)
        ttk.Button(buttons, text="기존 모터 관리 앱 열기", command=lambda: self.open_external("motor_manager")).pack(side="left", padx=4)
        self.tree = ttk.Treeview(
            self,
            columns=("id", "name", "puint", "raw", "sign", "notes"),
            show="headings",
            height=18,
        )
        for col, text, width in [
            ("id", "motor ID", 90),
            ("name", "name", 140),
            ("puint", "raw_zero_p_uint", 130),
            ("raw", "raw_zero_pos_rad", 160),
            ("sign", "direction_sign", 120),
            ("notes", "notes", 420),
        ]:
            self.tree.heading(col, text=text)
            self.tree.column(col, width=width, anchor="w")
        self.tree.pack(fill="both", expand=True, padx=12, pady=8)

    def on_show(self) -> None:
        self.refresh()

    def refresh(self) -> None:
        self.app.load_calibration()
        for item in self.tree.get_children():
            self.tree.delete(item)
        for motor_id, entry in sorted(self.app.calibration_entries.items()):
            self.tree.insert(
                "",
                "end",
                values=(
                    motor_id,
                    entry.get("name", ""),
                    entry.get("raw_zero_p_uint", ""),
                    entry.get("raw_zero_pos_rad", ""),
                    entry.get("direction_sign", ""),
                    entry.get("notes", ""),
                ),
            )

    def open_external(self, script_key: str) -> None:
        if self.app.process_manager.is_running():
            messagebox.showerror("실행 중", "모터 제어가 실행 중일 때는 원점 앱을 열 수 없습니다.")
            return
        if not script_exists(script_key):
            self.app.show_missing_script(script_key)
            return
        if not messagebox.askokcancel(
            "기존 앱 열기",
            "원점 저장 기능이 포함된 기존 앱을 엽니다.\n현재 모터 제어가 실행 중이지 않은지 확인하십시오.",
        ):
            return
        command = [sys.executable, str(script_path(script_key))]
        if script_key == "calibration_helper":
            command += ["--channel", self.app.channel_var.get()]
        if script_key == "ak45_calibration_helper":
            command += ["--channel", self.app.channel_var.get()]
        self.app.process_manager.start(f"{SCRIPTS[script_key]} 실행", command, [])


class SingleMotorPage(BasePage):
    def __init__(self, app: "AK70ControlCenterApp", parent: ttk.Frame) -> None:
        super().__init__(app, parent)
        ttk.Label(self, text="단일 모터 제어", font=("TkDefaultFont", 16, "bold")).pack(anchor="w", padx=12, pady=8)
        ttk.Label(
            self,
            text="실제 모터가 움직입니다. 주변을 정리하고 모터를 안전하게 고정하십시오. 비상 정지 버튼 위치를 확인하십시오. hardware zero 명령은 사용하지 않습니다.",
            foreground="#a33",
        ).pack(anchor="w", padx=12, pady=4)

        common = ttk.LabelFrame(self, text="공통 설정")
        common.pack(fill="x", padx=12, pady=4)
        self.motor_var = StringVar(value="0x00A")
        self.unit_var = StringVar(value="deg")
        self.target_var = StringVar(value="5")
        self.kp_var = StringVar(value="8.0")
        self.kd_var = StringVar(value="0.4")
        self.enter_mit_var = BooleanVar(value=True)
        self.current_angle_var = StringVar(value="현재각: N/A")
        ttk.Label(common, text="motor ID").grid(row=0, column=0, padx=4, pady=3)
        self.motor_combo = ttk.Combobox(common, textvariable=self.motor_var, values=AK70_IDS, width=10)
        self.motor_combo.grid(row=0, column=1, padx=4)
        self.motor_combo.bind("<<ComboboxSelected>>", lambda _event: self.refresh_current_angle())
        self.motor_var.trace_add("write", lambda *_args: self.refresh_current_angle())
        ttk.Label(common, text="목표각").grid(row=0, column=2, padx=4)
        ttk.Entry(common, textvariable=self.target_var, width=10).grid(row=0, column=3, padx=4)
        ttk.Combobox(common, textvariable=self.unit_var, values=["deg", "rad"], width=6, state="readonly").grid(row=0, column=4, padx=4)
        ttk.Label(common, text="Kp").grid(row=0, column=5, padx=4)
        ttk.Entry(common, textvariable=self.kp_var, width=8).grid(row=0, column=6, padx=4)
        ttk.Label(common, text="Kd").grid(row=0, column=7, padx=4)
        ttk.Entry(common, textvariable=self.kd_var, width=8).grid(row=0, column=8, padx=4)
        ttk.Label(common, textvariable=self.current_angle_var).grid(row=0, column=9, padx=8)
        ttk.Label(common, text="MIT 모드: 모터 실행 시 자동 진입").grid(row=0, column=10, padx=8)

        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill="both", expand=True, padx=12, pady=8)
        self.once_tab = ttk.Frame(self.notebook)
        self.hold_tab = ttk.Frame(self.notebook)
        self.traj_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.once_tab, text="1회 이동")
        self.notebook.add(self.hold_tab, text="위치 유지")
        self.notebook.add(self.traj_tab, text="Trajectory")
        self._build_once_tab()
        self._build_hold_tab()
        self._build_traj_tab()

    def on_show(self) -> None:
        candidate_ids = self.app.detected_motors or self.app.calibration_ids or set(AK70_IDS)
        ids = sorted(motor_id for motor_id in candidate_ids if motor_id in AK70_IDS) or AK70_IDS
        self.motor_combo.configure(values=ids)
        if normalize_motor_id(self.motor_var.get()) not in ids:
            self.motor_var.set(ids[0])
        self.refresh_current_angle()

    def refresh_current_angle(self) -> None:
        try:
            motor_id = normalize_motor_id(self.motor_var.get())
        except ValueError:
            self.current_angle_var.set("현재각: N/A")
            return
        current = self.app.current_positions.get(motor_id)
        self.current_angle_var.set(f"현재각: {current:+.2f}°" if current is not None else "현재각: N/A")

    def _build_once_tab(self) -> None:
        self.once_pulses_var = StringVar(value="20")
        self.once_interval_var = StringVar(value="0.02")
        row = ttk.Frame(self.once_tab)
        row.pack(fill="x", pady=8)
        ttk.Label(row, text="pulses").pack(side="left", padx=4)
        ttk.Entry(row, textvariable=self.once_pulses_var, width=8).pack(side="left", padx=4)
        ttk.Label(row, text="interval-sec").pack(side="left", padx=4)
        ttk.Entry(row, textvariable=self.once_interval_var, width=8).pack(side="left", padx=4)
        self._add_single_buttons(self.once_tab, "1회 이동", self.run_move_once, self.show_single_plan, include_home=True)

    def _build_hold_tab(self) -> None:
        self.hold_move_sec_var = StringVar(value="1.0")
        self.hold_hold_sec_var = StringVar(value="3.0")
        self.hold_rate_var = StringVar(value="50")
        self.hold_print_var = StringVar(value="5")
        row = ttk.Frame(self.hold_tab)
        row.pack(fill="x", pady=8)
        for label, var in [
            ("move_sec", self.hold_move_sec_var),
            ("hold_sec", self.hold_hold_sec_var),
            ("rate_hz", self.hold_rate_var),
            ("feedback_print_hz", self.hold_print_var),
        ]:
            ttk.Label(row, text=label).pack(side="left", padx=4)
            ttk.Entry(row, textvariable=var, width=8).pack(side="left", padx=4)
        self._add_single_buttons(self.hold_tab, "위치 유지", self.run_hold, self.show_single_plan, include_home=True)

    def _build_traj_tab(self) -> None:
        self.traj_hold_var = StringVar(value="2.0")
        self.traj_rate_var = StringVar(value="50")
        self.traj_print_var = StringVar(value="5")
        self.traj_error_var = StringVar(value="45")
        top = ttk.Frame(self.traj_tab)
        top.pack(fill="x", pady=6)
        for label, var in [
            ("final_hold_sec", self.traj_hold_var),
            ("rate_hz", self.traj_rate_var),
            ("feedback_print_hz", self.traj_print_var),
            ("max_following_error_deg", self.traj_error_var),
        ]:
            ttk.Label(top, text=label).pack(side="left", padx=4)
            ttk.Entry(top, textvariable=var, width=8).pack(side="left", padx=4)
        self.wp_tree = ttk.Treeview(self.traj_tab, columns=("target", "duration"), show="headings", height=8)
        self.wp_tree.heading("target", text="목표각")
        self.wp_tree.heading("duration", text="이동시간")
        self.wp_tree.pack(fill="both", expand=True, pady=6)
        wp_buttons = ttk.Frame(self.traj_tab)
        wp_buttons.pack(fill="x")
        ttk.Button(wp_buttons, text="Waypoint 추가", command=self.add_waypoint).pack(side="left", padx=3)
        ttk.Button(wp_buttons, text="선택 Waypoint 삭제", command=self.delete_waypoint).pack(side="left", padx=3)
        ttk.Button(wp_buttons, text="위로 이동", command=lambda: self.move_waypoint(-1)).pack(side="left", padx=3)
        ttk.Button(wp_buttons, text="아래로 이동", command=lambda: self.move_waypoint(1)).pack(side="left", padx=3)
        ttk.Button(wp_buttons, text="전체 삭제", command=lambda: self.wp_tree.delete(*self.wp_tree.get_children())).pack(side="left", padx=3)
        self._add_single_buttons(self.traj_tab, "Trajectory", self.run_trajectory, self.show_trajectory_plan)

    def _add_single_buttons(
        self,
        parent: ttk.Frame,
        label: str,
        run_cmd: Callable[[], None],
        plan_cmd: Callable[[], None],
        include_home: bool = False,
    ) -> None:
        buttons = ttk.Frame(parent)
        buttons.pack(fill="x", pady=8)
        ttk.Button(buttons, text="현재 위치 읽기", command=self.read_current).pack(side="left", padx=3)
        ttk.Button(buttons, text="실행 계획 확인", command=plan_cmd).pack(side="left", padx=3)
        ttk.Button(buttons, text=f"{label} 실행", command=run_cmd).pack(side="left", padx=3)
        if include_home:
            ttk.Button(buttons, text="선택 모터 원점 이동", command=self.run_home_to_zero).pack(side="left", padx=3)
        ttk.Button(buttons, text="선택 모터 TORQUE 해제", command=self.release_selected_motor).pack(side="left", padx=3)

    def release_selected_motor(self) -> None:
        try:
            motor_id = normalize_motor_id(self.motor_var.get())
        except ValueError as exc:
            messagebox.showerror("입력 오류", str(exc))
            return
        self.app.release_selected_ak70_ids([motor_id])

    def read_current(self) -> None:
        try:
            motor_id = normalize_motor_id(self.motor_var.get())
        except ValueError as exc:
            messagebox.showerror("입력 오류", str(exc))
            return
        self.app.run_read_positions([motor_id])

    def get_common_values(self) -> tuple[str, float, float, float]:
        motor_id = normalize_motor_id(self.motor_var.get())
        target = parse_finite_float(self.target_var.get(), "목표각")
        target_deg = target if self.unit_var.get() == "deg" else math.degrees(target)
        kp = parse_finite_float(self.kp_var.get(), "Kp")
        kd = parse_finite_float(self.kd_var.get(), "Kd")
        self.app.validate_basic_motion(motor_id, target_deg, kp, kd)
        return motor_id, target, kp, kd

    def show_single_plan(self) -> None:
        try:
            motor_id, target, kp, kd = self.get_common_values()
            target_deg = target if self.unit_var.get() == "deg" else math.degrees(target)
            current = self.app.require_current_angle(motor_id)
        except ValueError as exc:
            messagebox.showerror("계획 확인 실패", str(exc))
            return
        self.app.show_plan_table(
            "단일 모터 실행 계획",
            ["motor ID", "현재각", "목표각", "예상 이동량", "kp", "kd"],
            [[motor_id, f"{current:+.2f}", f"{target_deg:+.2f}", f"{target_deg-current:+.2f}", kp, kd]],
        )

    def show_trajectory_plan(self) -> None:
        try:
            motor_id = normalize_motor_id(self.motor_var.get())
            current = self.app.require_current_angle(motor_id)
            rows = []
            start = current
            for item in self.wp_tree.get_children():
                target = float(self.wp_tree.item(item, "values")[0])
                duration = float(self.wp_tree.item(item, "values")[1])
                rows.append([motor_id, f"{start:+.2f}", f"{target:+.2f}", f"{target-start:+.2f}", duration])
                start = target
            if not rows:
                raise ValueError("Waypoint가 없습니다.")
        except Exception as exc:
            messagebox.showerror("계획 확인 실패", str(exc))
            return
        self.app.show_plan_table("Trajectory 실행 계획", ["motor ID", "시작각", "목표각", "이동량", "이동시간"], rows)

    def add_waypoint(self) -> None:
        self.wp_tree.insert("", "end", values=(self.target_var.get(), "1.0"))

    def delete_waypoint(self) -> None:
        for item in self.wp_tree.selection():
            self.wp_tree.delete(item)

    def move_waypoint(self, direction: int) -> None:
        selected = self.wp_tree.selection()
        if not selected:
            return
        item = selected[0]
        index = self.wp_tree.index(item)
        new_index = max(0, min(len(self.wp_tree.get_children()) - 1, index + direction))
        self.wp_tree.move(item, "", new_index)

    def confirm_execution(self, title: str, text: str) -> bool:
        return messagebox.askokcancel(title, text)

    def run_move_once(self) -> None:
        try:
            motor_id, target, kp, kd = self.get_common_values()
            target_deg = target if self.unit_var.get() == "deg" else math.degrees(target)
            current = self.app.require_current_angle(motor_id)
            pulses = int(parse_finite_float(self.once_pulses_var.get(), "pulses"))
            interval = parse_finite_float(self.once_interval_var.get(), "interval-sec")
            move_sec = max(0.05, pulses * interval)
        except ValueError as exc:
            messagebox.showerror("입력 오류", str(exc))
            return
        if not self.confirm_execution(
            "1회 이동 실행",
            f"실제 모터가 움직입니다.\n\n모터: {motor_id}\n현재각: {current:+.2f}°\n목표각: {target_deg:+.2f}°\n예상 이동량: {target_deg-current:+.2f}°\n\n계속 실행하시겠습니까?",
        ):
            return
        self.app.submit_ak70_targets(
            [{"motor_id": motor_id, "target": target_deg, "kp": kp, "kd": kd}],
            task="단일 목표각 이동 및 유지",
            move_sec=move_sec,
            max_following_error_deg=60.0,
            display_mode="target",
        )

    def run_hold(self) -> None:
        try:
            motor_id, target, kp, kd = self.get_common_values()
            target_deg = target if self.unit_var.get() == "deg" else math.degrees(target)
            current = self.app.require_current_angle(motor_id)
            move_sec = parse_finite_float(self.hold_move_sec_var.get(), "move_sec")
            parse_finite_float(self.hold_hold_sec_var.get(), "hold_sec")
            rate = parse_finite_float(self.hold_rate_var.get(), "rate_hz")
            print_hz = parse_finite_float(self.hold_print_var.get(), "feedback_print_hz")
        except ValueError as exc:
            messagebox.showerror("입력 오류", str(exc))
            return
        if not self.confirm_execution("위치 유지 실행", f"{motor_id}: {current:+.2f}° -> {target_deg:+.2f}°\n실행하시겠습니까?"):
            return
        self.app.submit_ak70_targets(
            [{"motor_id": motor_id, "target": target_deg, "kp": kp, "kd": kd}],
            task="단일 위치 유지",
            move_sec=move_sec,
            max_following_error_deg=60.0,
            display_mode="target",
        )

    def run_trajectory(self) -> None:
        try:
            motor_id = normalize_motor_id(self.motor_var.get())
            self.app.require_current_angle(motor_id)
            kp = parse_finite_float(self.kp_var.get(), "Kp")
            kd = parse_finite_float(self.kd_var.get(), "Kd")
            parse_finite_float(self.traj_hold_var.get(), "final_hold_sec")
            rate = parse_finite_float(self.traj_rate_var.get(), "rate_hz")
            print_hz = parse_finite_float(self.traj_print_var.get(), "feedback_print_hz")
            max_error = parse_finite_float(self.traj_error_var.get(), "max_following_error_deg")
            waypoints = []
            for item in self.wp_tree.get_children():
                values = self.wp_tree.item(item, "values")
                waypoints.append((parse_finite_float(str(values[0]), "waypoint target"), parse_finite_float(str(values[1]), "waypoint duration")))
            if not waypoints:
                raise ValueError("Waypoint가 없습니다.")
        except ValueError as exc:
            messagebox.showerror("입력 오류", str(exc))
            return
        if not self.confirm_execution("Trajectory 실행", f"{motor_id} trajectory를 실행합니다.\nWaypoint 수: {len(waypoints)}\n계속하시겠습니까?"):
            return
        self.app.submit_ak70_trajectory(
            motor_id,
            waypoints,
            kp=kp,
            kd=kd,
            max_following_error_deg=max_error,
            display_mode="target",
        )

    def run_home_to_zero(self) -> None:
        try:
            motor_id = normalize_motor_id(self.motor_var.get())
            current = self.app.require_current_angle(motor_id)
            if motor_id not in self.app.calibration_ids:
                raise ValueError(f"{motor_id} calibration을 찾을 수 없습니다.")
            kp = parse_finite_float(self.kp_var.get(), "Kp")
            kd = parse_finite_float(self.kd_var.get(), "Kd")
            move_sec = 2.0
            effective_move_sec = max(move_sec, abs(current) / 60.0)
        except ValueError as exc:
            messagebox.showerror("원점 이동 실패", f"{exc}\n원점 이동을 시작하지 않았습니다.")
            return

        self.app.show_plan_table(
            "Software Zero 이동 계획",
            ["ID", "현재각", "목표각", "이동량"],
            [[motor_id, f"{current:+.2f}°", "0.00°", f"{-current:+.2f}°"]],
        )
        if not messagebox.askokcancel(
            "Software Zero 이동",
            (
                "저장된 software zero 위치로 이동합니다.\n\n"
                f"모터: {motor_id}\n"
                f"요청 이동시간: {move_sec:.1f}초\n"
                f"예상 적용 이동시간: {effective_move_sec:.1f}초\n"
                "도착 후 사용자가 TORQUE 해제를 누를 때까지 유지\n\n"
                "계속하시겠습니까?"
            ),
        ):
            return
        self.app.submit_ak70_targets(
            [{"motor_id": motor_id, "target": 0.0, "kp": kp, "kd": kd}],
            task="단일 Software Zero 이동 및 유지",
            move_sec=effective_move_sec,
            max_following_error_deg=60.0,
            display_mode="home",
        )


class MultiMotorPage(BasePage):
    def __init__(self, app: "AK70ControlCenterApp", parent: ttk.Frame) -> None:
        super().__init__(app, parent)
        ttk.Label(self, text="다중 모터 제어", font=("TkDefaultFont", 16, "bold")).pack(anchor="w", padx=12, pady=8)
        ttk.Label(
            self,
            text="실제 모터가 움직입니다. 선택된 모터가 동시에 움직입니다. 모터와 연결 구조물이 서로 충돌하지 않는지 확인하십시오.",
            foreground="#a33",
        ).pack(anchor="w", padx=12, pady=4)
        settings = ttk.Frame(self)
        settings.pack(fill="x", padx=12, pady=4)
        self.common_kp_var = StringVar(value="8.0")
        self.common_kd_var = StringVar(value="0.4")
        self.move_sec_var = StringVar(value="2.0")
        self.hold_sec_var = StringVar(value="2.0")
        self.rate_var = StringVar(value="50")
        self.print_var = StringVar(value="5")
        self.error_var = StringVar(value="60")
        self.enter_mit_var = BooleanVar(value=True)
        for label, var in [
            ("공통 Kp", self.common_kp_var), ("공통 Kd", self.common_kd_var), ("move_sec", self.move_sec_var),
            ("hold_sec", self.hold_sec_var), ("rate_hz", self.rate_var), ("feedback_print_hz", self.print_var),
            ("max_error", self.error_var),
        ]:
            ttk.Label(settings, text=label).pack(side="left", padx=3)
            ttk.Entry(settings, textvariable=var, width=8).pack(side="left", padx=3)
        ttk.Label(settings, text="MIT 모드: 모터 실행 시 자동 진입").pack(side="left", padx=8)

        self.tree = ttk.Treeview(self, columns=("use", "id", "current", "target", "kp", "kd", "common"), show="headings", height=12)
        for col, text, width in [
            ("use", "사용", 60), ("id", "motor ID", 90), ("current", "현재각", 90),
            ("target", "목표각", 90), ("kp", "kp", 80), ("kd", "kd", 80), ("common", "공통 gain", 90),
        ]:
            self.tree.heading(col, text=text)
            self.tree.column(col, width=width, anchor="center")
        self.tree.pack(fill="both", expand=True, padx=12, pady=6)
        self.tree.bind("<Double-1>", self.edit_cell)
        self.tree.bind("<Button-1>", self.toggle_use)

        buttons = ttk.Frame(self)
        buttons.pack(fill="x", padx=12, pady=4)
        for text, cmd in [
            ("감지 모터 불러오기", self.load_detected),
            ("행 추가", self.add_row),
            ("선택 행 삭제", self.delete_selected),
            ("모든 목표를 0도로 설정", self.zero_targets),
            ("현재 위치 읽기", self.read_positions),
            ("실행 계획 확인", self.show_plan),
            ("동기 Pose 실행", self.run_pose),
            ("선택 모터 원점 이동", self.run_home_to_zero),
            ("선택 모터 TORQUE 해제", self.release_selected),
        ]:
            ttk.Button(buttons, text=text, command=cmd).pack(side="left", padx=3)

        presets = ttk.LabelFrame(self, text="다중 pose 프리셋", height=140)
        presets.pack(fill="x", padx=12, pady=8)
        presets.pack_propagate(False)
        self.preset_name_var = StringVar()
        ttk.Label(presets, text="프리셋 선택").grid(row=0, column=0, sticky="w", padx=8, pady=12)
        self.preset_combo = ttk.Combobox(presets, textvariable=self.preset_name_var, values=app.preset_manager.names(), width=32)
        self.preset_combo.grid(row=0, column=1, sticky="w", padx=6, pady=12)
        ttk.Button(presets, text="새로고침", command=self.refresh_presets).grid(row=0, column=2, padx=4, pady=12)
        ttk.Button(presets, text="불러오기", command=self.load_preset).grid(row=0, column=3, padx=4, pady=12)
        ttk.Label(presets, text="프리셋 이름").grid(row=1, column=0, sticky="w", padx=8, pady=10)
        ttk.Entry(presets, textvariable=self.preset_name_var, width=34).grid(row=1, column=1, sticky="w", padx=6, pady=10)
        ttk.Button(presets, text="현재 설정 저장", command=self.save_preset).grid(row=1, column=2, padx=4, pady=10)
        ttk.Button(presets, text="이름 변경", command=self.rename_preset).grid(row=1, column=3, padx=4, pady=10)
        ttk.Button(presets, text="삭제", command=self.delete_preset).grid(row=1, column=4, padx=4, pady=10)
        presets.grid_columnconfigure(5, weight=1)

    def on_show(self) -> None:
        self.refresh_current_angles()

    def add_row(self, motor_id: str | None = None) -> None:
        motor_id = normalize_motor_id(motor_id or "0x00A")
        current = self.app.current_positions.get(motor_id)
        self.tree.insert("", "end", values=("☑", motor_id, f"{current:+.2f}" if current is not None else "N/A", "0", self.common_kp_var.get(), self.common_kd_var.get(), "YES"))

    def refresh_current_angles(self) -> None:
        for item in self.tree.get_children():
            values = self.tree.item(item, "values")
            if len(values) < 2:
                continue
            try:
                motor_id = normalize_motor_id(str(values[1]))
            except ValueError:
                continue
            current = self.app.current_positions.get(motor_id)
            current_text = "N/A" if current is None else f"{current:+.2f}"
            self.tree.set(item, "current", current_text)

    def load_detected(self) -> None:
        self.tree.delete(*self.tree.get_children())
        for motor_id in sorted(self.app.detected_motors or self.app.calibration_ids):
            self.add_row(motor_id)

    def delete_selected(self) -> None:
        for item in self.tree.selection():
            self.tree.delete(item)

    def zero_targets(self) -> None:
        for item in self.tree.get_children():
            values = list(self.tree.item(item, "values"))
            values[3] = "0"
            self.tree.item(item, values=values)

    def read_positions(self) -> None:
        ids = self.selected_ids()
        if ids:
            self.app.run_read_positions(ids)

    def release_selected(self) -> None:
        ids = self.selected_ids()
        if ids:
            self.app.release_selected_ak70_ids(ids)

    def selected_ids(self) -> list[str]:
        ids: list[str] = []
        for item in self.tree.get_children():
            values = self.tree.item(item, "values")
            if values and values[0] == "☑":
                try:
                    ids.append(normalize_motor_id(str(values[1])))
                except ValueError as exc:
                    messagebox.showerror("입력 오류", str(exc))
                    return []
        return ids

    def get_rows(self) -> list[dict[str, Any]]:
        rows = []
        seen: set[str] = set()
        for item in self.tree.get_children():
            values = self.tree.item(item, "values")
            if values[0] != "☑":
                continue
            motor_id = normalize_motor_id(str(values[1]))
            if motor_id in seen:
                raise ValueError(f"중복 motor ID: {motor_id}")
            seen.add(motor_id)
            target = parse_finite_float(str(values[3]), f"{motor_id} 목표각")
            kp = parse_finite_float(str(values[4]), f"{motor_id} kp")
            kd = parse_finite_float(str(values[5]), f"{motor_id} kd")
            current = self.app.require_current_angle(motor_id)
            self.app.validate_basic_motion(motor_id, target, kp, kd)
            rows.append({"motor_id": motor_id, "target": target, "kp": kp, "kd": kd, "current": current})
        if not rows:
            raise ValueError("선택된 모터가 없습니다.")
        if any(not is_ak45_id(row["motor_id"]) for row in rows) and len([r for r in rows if not is_ak45_id(r["motor_id"])]) > 10:
            raise ValueError("AK70 다중 Pose는 최대 10개 모터만 선택할 수 있습니다.")
        if len(rows) > 13:
            raise ValueError("최대 13개 모터만 선택할 수 있습니다.")
        return rows

    def get_home_rows(self) -> list[dict[str, Any]]:
        rows = []
        seen: set[str] = set()
        for item in self.tree.get_children():
            values = self.tree.item(item, "values")
            if values[0] != "☑":
                continue
            motor_id = normalize_motor_id(str(values[1]))
            if motor_id in seen:
                raise ValueError(f"중복 motor ID: {motor_id}")
            seen.add(motor_id)
            if motor_id not in self.app.calibration_ids:
                raise ValueError(f"{motor_id}의 calibration을 찾을 수 없습니다.")
            current = self.app.require_current_angle(motor_id)
            kp = parse_finite_float(str(values[4]), f"{motor_id} kp")
            kd = parse_finite_float(str(values[5]), f"{motor_id} kd")
            if not 0.0 < kp <= 10.0:
                raise ValueError(f"{motor_id} Kp 범위가 잘못되었습니다.")
            if not 0.0 <= kd <= 2.0:
                raise ValueError(f"{motor_id} Kd 범위가 잘못되었습니다.")
            rows.append({"motor_id": motor_id, "target": 0.0, "kp": kp, "kd": kd, "current": current})
        if not rows:
            raise ValueError("선택된 모터가 없습니다.")
        if any(not is_ak45_id(row["motor_id"]) for row in rows) and len([r for r in rows if not is_ak45_id(r["motor_id"])]) > 10:
            raise ValueError("AK70 다중 원점 이동은 최대 10개 모터만 선택할 수 있습니다.")
        if len(rows) > 13:
            raise ValueError("최대 13개 모터만 선택할 수 있습니다.")
        return rows

    def show_plan(self) -> None:
        try:
            rows = self.get_rows()
            table = [
                [r["motor_id"], f"{r['current']:+.2f}", f"{r['target']:+.2f}", f"{r['target']-r['current']:+.2f}", r["kp"], r["kd"], self.move_sec_var.get(), self.hold_sec_var.get()]
                for r in rows
            ]
        except ValueError as exc:
            messagebox.showerror("계획 확인 실패", str(exc))
            return
        self.app.show_plan_table("다중 모터 Pose 계획", ["motor ID", "현재각", "목표각", "예상 이동량", "kp", "kd", "move_sec", "hold_sec"], table)

    def run_pose(self) -> None:
        try:
            rows = self.get_rows()
            common_kp = parse_finite_float(self.common_kp_var.get(), "공통 Kp")
            common_kd = parse_finite_float(self.common_kd_var.get(), "공통 Kd")
            move_sec = parse_finite_float(self.move_sec_var.get(), "move_sec")
            hold_sec = parse_finite_float(self.hold_sec_var.get(), "hold_sec")
            rate = parse_finite_float(self.rate_var.get(), "rate_hz")
            print_hz = parse_finite_float(self.print_var.get(), "feedback_print_hz")
            max_error = parse_finite_float(self.error_var.get(), "max_following_error_deg")
        except ValueError as exc:
            messagebox.showerror("입력 오류", str(exc))
            return
        contains_ak45 = any(is_ak45_id(row["motor_id"]) for row in rows)
        if contains_ak45 and not self.app.confirm_ak45_power():
            return
        if not messagebox.askokcancel("동기 Pose 실행", f"선택된 {len(rows)}개 모터가 동시에 움직입니다.\n실행하시겠습니까?"):
            return
        if not contains_ak45:
            self.app.submit_ak70_targets(
                rows,
                task="다중 동기 Pose 유지",
                move_sec=move_sec,
                max_following_error_deg=max_error,
                display_mode="target",
            )
            return
        script_key = "mixed_pose" if contains_ak45 else "multi_pose"
        command = [sys.executable, str(script_path(script_key)), "--channel", self.app.channel_var.get()]
        motor_ids = []
        for row in rows:
            motor_ids.append(row["motor_id"])
            command += ["--target-deg", f"{row['motor_id']},{row['target']}"]
        command += ["--kp", str(common_kp), "--kd", str(common_kd)]
        for row in rows:
            if abs(row["kp"] - common_kp) > 1e-12 or abs(row["kd"] - common_kd) > 1e-12:
                command += ["--gain", f"{row['motor_id']},{row['kp']},{row['kd']}"]
        command += [
            "--move-sec", str(move_sec), "--hold-sec", str(hold_sec), "--rate-hz", str(rate),
            "--feedback-print-hz", str(print_hz), "--max-following-error-deg", str(max_error), "--enter-mit",
        ]
        if contains_ak45:
            command.append("--ak45-power-verified")
            for row in rows:
                if is_ak45_id(row["motor_id"]):
                    command += ["--session-homed", row["motor_id"]]
            self.app.start_control("혼합 동기 Pose", command, motor_ids, [])
        else:
            self.app.start_control("다중 동기 Pose", command, motor_ids, [AutoResponse("Type YES to start synchronized multi-joint pose move:", "YES", "YES")])

    def run_home_to_zero(self) -> None:
        try:
            rows = self.get_home_rows()
            common_kp = parse_finite_float(self.common_kp_var.get(), "공통 Kp")
            common_kd = parse_finite_float(self.common_kd_var.get(), "공통 Kd")
            move_sec = parse_finite_float(self.move_sec_var.get(), "move_sec")
            hold_sec = parse_finite_float(self.hold_sec_var.get(), "hold_sec")
            rate = parse_finite_float(self.rate_var.get(), "rate_hz")
            print_hz = parse_finite_float(self.print_var.get(), "feedback_print_hz")
            max_error = parse_finite_float(self.error_var.get(), "max_following_error_deg")
            max_delta = max(abs(row["current"]) for row in rows)
            effective_move_sec = max(move_sec, max_delta / 60.0)
        except ValueError as exc:
            messagebox.showerror("원점 이동 실패", f"{exc}\n원점 이동을 시작하지 않았습니다.")
            return

        table = [
            [row["motor_id"], f"{row['current']:+.2f}°", "0.00°", f"{-row['current']:+.2f}°"]
            for row in rows
        ]
        self.app.show_plan_table("Software Zero 이동 계획", ["ID", "현재각", "목표각", "이동량"], table)
        if not messagebox.askokcancel(
            "Software Zero 이동",
            (
                "저장된 software zero 위치로 이동합니다.\n\n"
                f"요청 이동시간: {move_sec:.1f}초\n"
                f"예상 적용 이동시간: {effective_move_sec:.1f}초\n"
                "도착 후 사용자가 TORQUE 해제를 누를 때까지 유지\n\n"
                "계속하시겠습니까?"
            ),
        ):
            return

        contains_ak45 = any(is_ak45_id(row["motor_id"]) for row in rows)
        if contains_ak45 and not self.app.confirm_ak45_power():
            return
        if not contains_ak45:
            self.app.submit_ak70_targets(
                rows,
                task="다중 Software Zero 이동 및 유지",
                move_sec=effective_move_sec,
                max_following_error_deg=max_error,
                display_mode="home",
            )
            return
        script_key = "mixed_pose" if contains_ak45 else "multi_pose"
        command = [sys.executable, str(script_path(script_key)), "--channel", self.app.channel_var.get()]
        motor_ids = []
        for row in rows:
            motor_ids.append(row["motor_id"])
            command += ["--target-deg", f"{row['motor_id']},0"]
        command += ["--kp", str(common_kp), "--kd", str(common_kd)]
        for row in rows:
            if abs(row["kp"] - common_kp) > 1e-12 or abs(row["kd"] - common_kd) > 1e-12:
                command += ["--gain", f"{row['motor_id']},{row['kp']},{row['kd']}"]
        command += [
            "--move-sec", str(move_sec), "--hold-sec", str(hold_sec), "--rate-hz", str(rate),
            "--feedback-print-hz", str(print_hz), "--max-following-error-deg", str(max_error), "--home-to-zero", "--enter-mit",
        ]
        if contains_ak45:
            command.append("--ak45-power-verified")
            for row in rows:
                if is_ak45_id(row["motor_id"]):
                    command += ["--session-homed", row["motor_id"]]
            self.app.start_control("혼합 Software Zero 이동", command, motor_ids, [])
        else:
            self.app.start_control(
                "다중 Software Zero 이동",
                command,
                motor_ids,
                [AutoResponse("Type YES to start synchronized multi-joint pose move:", "YES", "YES")],
            )

    def toggle_use(self, event: tk.Event) -> None:
        if self.tree.identify_column(event.x) != "#1":
            return
        item = self.tree.identify_row(event.y)
        if not item:
            return
        values = list(self.tree.item(item, "values"))
        values[0] = "☐" if values[0] == "☑" else "☑"
        self.tree.item(item, values=values)

    def edit_cell(self, event: tk.Event) -> None:
        item = self.tree.identify_row(event.y)
        column = self.tree.identify_column(event.x)
        if not item or column not in {"#2", "#4", "#5", "#6", "#7"}:
            return
        index = int(column[1:]) - 1
        values = list(self.tree.item(item, "values"))
        x, y, width, height = self.tree.bbox(item, column)
        entry = ttk.Entry(self.tree)
        entry.insert(0, values[index])
        entry.place(x=x, y=y, width=width, height=height)
        entry.focus()

        def commit(_event=None) -> None:
            values[index] = entry.get()
            self.tree.item(item, values=values)
            entry.destroy()

        entry.bind("<Return>", commit)
        entry.bind("<FocusOut>", commit)

    def collect_preset_data(self) -> dict[str, Any]:
        rows = [self.tree.item(item, "values") for item in self.tree.get_children()]
        return {
            "rows": [list(row) for row in rows],
            "kp": self.common_kp_var.get(),
            "kd": self.common_kd_var.get(),
            "move_sec": self.move_sec_var.get(),
            "hold_sec": self.hold_sec_var.get(),
            "rate_hz": self.rate_var.get(),
            "feedback_print_hz": self.print_var.get(),
            "max_following_error_deg": self.error_var.get(),
            "enter_mit": self.enter_mit_var.get(),
        }

    def apply_preset_data(self, data: dict[str, Any]) -> None:
        self.tree.delete(*self.tree.get_children())
        for row in data.get("rows", []):
            self.tree.insert("", "end", values=row)
        self.common_kp_var.set(str(data.get("kp", "8.0")))
        self.common_kd_var.set(str(data.get("kd", "0.4")))
        self.move_sec_var.set(str(data.get("move_sec", "2.0")))
        self.hold_sec_var.set(str(data.get("hold_sec", "2.0")))
        self.rate_var.set(str(data.get("rate_hz", "50")))
        self.print_var.set(str(data.get("feedback_print_hz", "5")))
        self.error_var.set(str(data.get("max_following_error_deg", "60")))
        self.enter_mit_var.set(True)
        self.refresh_current_angles()

    def save_preset(self) -> None:
        name = self.preset_name_var.get().strip()
        if not name:
            messagebox.showerror("프리셋", "프리셋 이름을 입력하십시오.")
            return
        self.app.preset_manager.presets[name] = self.collect_preset_data()
        self.app.preset_manager.save()
        self.refresh_presets()

    def load_preset(self) -> None:
        name = self.preset_name_var.get().strip()
        data = self.app.preset_manager.presets.get(name)
        if data is None:
            messagebox.showerror("프리셋", "선택한 프리셋을 찾을 수 없습니다.")
            return
        self.apply_preset_data(data)

    def rename_preset(self) -> None:
        old = self.preset_name_var.get().strip()
        if old not in self.app.preset_manager.presets:
            messagebox.showerror("프리셋", "기존 프리셋을 선택하십시오.")
            return
        new = simpledialog.askstring("프리셋 이름 변경", "새 이름")
        if not new:
            return
        self.app.preset_manager.presets[new] = self.app.preset_manager.presets.pop(old)
        self.app.preset_manager.save()
        self.preset_name_var.set(new)
        self.refresh_presets()

    def delete_preset(self) -> None:
        name = self.preset_name_var.get().strip()
        if name not in self.app.preset_manager.presets:
            return
        if not messagebox.askokcancel("프리셋 삭제", "선택한 프리셋을 삭제하시겠습니까?"):
            return
        self.app.preset_manager.presets.pop(name, None)
        self.app.preset_manager.save()
        self.preset_name_var.set("")
        self.refresh_presets()

    def refresh_presets(self) -> None:
        self.preset_combo.configure(values=self.app.preset_manager.names())


class LogSafetyPage(BasePage):
    def __init__(self, app: "AK70ControlCenterApp", parent: ttk.Frame) -> None:
        super().__init__(app, parent)
        ttk.Label(self, text="로그 / 안전", font=("TkDefaultFont", 16, "bold")).pack(anchor="w", padx=12, pady=8)
        buttons = ttk.Frame(self)
        buttons.pack(fill="x", padx=12, pady=4)
        ttk.Button(buttons, text="비상 정지 / TORQUE 해제", command=app.emergency_stop).pack(side="left", padx=4)
        ttk.Button(buttons, text="로그 전체 지우기", command=app.clear_log).pack(side="left", padx=4)
        ttk.Button(buttons, text="로그 파일 저장", command=app.save_log).pack(side="left", padx=4)
        ttk.Button(buttons, text="최근 명령 복사", command=app.copy_last_command).pack(side="left", padx=4)
        ttk.Button(buttons, text="수평 자세 확인", command=app.confirm_selected_ak45_home).pack(side="left", padx=4)
        ttk.Button(buttons, text="Homing 해제", command=app.clear_selected_ak45_home).pack(side="left", padx=4)
        ttk.Button(buttons, text="실시간 컨트롤러 시작", command=app.start_realtime_controller).pack(side="left", padx=4)
        ttk.Button(buttons, text="실시간 컨트롤러 상태", command=app.realtime_status).pack(side="left", padx=4)
        ttk.Button(buttons, text="실시간 목표 전송", command=app.send_realtime_targets).pack(side="left", padx=4)
        ttk.Button(buttons, text="실시간 제어 정지", command=app.stop_realtime_controller).pack(side="left", padx=4)
        ttk.Button(buttons, text="CAN 상태 확인", command=app.run_can_health).pack(side="left", padx=4)
        ttk.Checkbutton(buttons, text="자동 스크롤", variable=app.autoscroll_var).pack(side="left", padx=12)
        ttk.Label(self, text="비상 정지 버튼은 추가 확인 없이 즉시 SIGINT와 torque 해제를 시도합니다.", foreground="#a33").pack(anchor="w", padx=12, pady=8)


class AK70ControlCenterApp:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("AK70 통합 제어 센터")
        self.root.geometry("1400x850")
        self.root.minsize(1100, 700)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.channel_var = StringVar(value=DEFAULT_CHANNEL)
        self.status_var = StringVar(value="대기")
        self.current_task_var = StringVar(value="대기")
        self.can_state_var = StringVar(value="UNKNOWN")
        self.detected_count_var = StringVar(value="0개")
        self.process_state_var = StringVar(value="대기")
        self.autoscroll_var = BooleanVar(value=True)

        self.main_thread_id = threading.get_ident()
        self.log_queue: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.process_manager = ProcessManager(self, self.log_queue)
        self.can_reader = CanStatusReader()
        self.preset_manager = PresetManager()
        self.can_status = CanStatus()
        self.detected_motors: set[str] = set()
        self.selected_communication_motor_ids: set[str] = set()
        self.current_positions: dict[str, float] = {}
        self.motor_infos: dict[str, MotorInfo] = {motor_id: MotorInfo(motor_id=motor_id) for motor_id in AK70_IDS}
        self.motor_infos.update({motor_id: MotorInfo(motor_id=motor_id) for motor_id in AK45_IDS})
        self.calibration_entries: dict[str, dict[str, Any]] = {}
        self.calibration_ids: set[str] = set()
        self.ak45_calibration_entries: dict[str, dict[str, Any]] = {}
        self.ak45_calibration_ids: set[str] = set()
        self.ak45_homing_states: dict[str, str] = {motor_id: "UNHOMED" for motor_id in AK45_IDS}
        self.last_command: list[str] = []
        self.realtime_process: subprocess.Popen[str] | None = None
        self.realtime_process_owner = False
        self.realtime_process_owner_token: str | None = None
        self.realtime_session_owner = False
        self.realtime_session_token: str | None = None
        self.realtime_session_epoch: int | None = None
        self.realtime_status_cache: dict[str, Any] = {}
        self.realtime_heartbeat_seq = 0
        self.realtime_closing = False
        self.close_state = CloseState.RUNNING
        self.close_deadline_monotonic: float | None = None
        self.ipc_requests: queue.Queue[IpcRequest] = queue.Queue()
        self.ipc_results: queue.Queue[tuple[IpcRequest, dict[str, Any]]] = queue.Queue()
        self.ipc_worker = threading.Thread(target=self._ipc_worker_loop, daemon=True)
        self.ipc_worker.start()
        self._parse_buffer = ""
        self._actual_table_active = False
        self.pages: dict[str, BasePage] = {}
        self.sidebar_buttons: dict[str, ttk.Button] = {}
        self.current_page_name = "홈"

        self.build_ui()
        self.load_calibration()
        self.refresh_can_status()
        self.show_page("홈")
        self.root.after(100, self.process_log_queue)
        self.root.after(100, self.process_ipc_results)
        self.root.after(GUI_HEARTBEAT_INTERVAL_MS, self.heartbeat_tick)
        self.root.after(1000, self.request_realtime_status_tick)

    def build_ui(self) -> None:
        root = ttk.Frame(self.root)
        root.pack(fill="both", expand=True)

        sidebar = ttk.Frame(root, width=180)
        sidebar.pack(side="left", fill="y")
        ttk.Label(sidebar, text="AK70 / AK45", font=("TkDefaultFont", 18, "bold")).pack(pady=14)
        for name in ["홈", "통신 연결", "원점 / 캘리브레이션", "단일 모터 제어", "다중 모터 제어", "로그 / 안전"]:
            button = ttk.Button(sidebar, text=name, command=lambda n=name: self.show_page(n))
            button.pack(fill="x", padx=8, pady=4)
            self.sidebar_buttons[name] = button
        ttk.Button(sidebar, text="종료", command=self.on_close).pack(fill="x", padx=8, pady=18)

        main = ttk.Frame(root)
        main.pack(side="left", fill="both", expand=True)
        self.build_status_bar(main)
        center = ttk.Panedwindow(main, orient="vertical")
        center.pack(fill="both", expand=True)
        self.page_container = ttk.Frame(center)
        center.add(self.page_container, weight=5)
        log_frame = ttk.LabelFrame(center, text="실시간 로그")
        center.add(log_frame, weight=2)
        self.log_text = tk.Text(log_frame, height=12, wrap="word")
        self.log_text.pack(side="left", fill="both", expand=True)
        scroll = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        scroll.pack(side="right", fill="y")
        self.log_text.configure(yscrollcommand=scroll.set)

        emergency = tk.Button(
            main,
            text="비상 정지 / TORQUE 해제",
            command=self.emergency_stop,
            bg="#c62828",
            fg="white",
            font=("TkDefaultFont", 13, "bold"),
        )
        emergency.pack(fill="x", padx=8, pady=6)

        self.pages = {
            "홈": HomePage(self, self.page_container),
            "통신 연결": CommunicationPage(self, self.page_container),
            "원점 / 캘리브레이션": CalibrationPage(self, self.page_container),
            "단일 모터 제어": SingleMotorPage(self, self.page_container),
            "다중 모터 제어": MultiMotorPage(self, self.page_container),
            "로그 / 안전": LogSafetyPage(self, self.page_container),
        }
        for page in self.pages.values():
            page.place(relx=0, rely=0, relwidth=1, relheight=1)

    def build_status_bar(self, parent: ttk.Frame) -> None:
        bar = ttk.Frame(parent)
        bar.pack(fill="x", padx=8, pady=4)
        for label, var in [
            ("CAN", self.can_state_var),
            ("채널", self.channel_var),
            ("감지 모터", self.detected_count_var),
            ("현재 작업", self.current_task_var),
            ("상태", self.process_state_var),
        ]:
            ttk.Label(bar, text=f"{label}:").pack(side="left", padx=(8, 2))
            ttk.Label(bar, textvariable=var).pack(side="left", padx=(0, 12))
        ttk.Button(bar, text="CAN 상태 새로고침", command=self.refresh_can_status).pack(side="right", padx=4)

    def show_page(self, name: str) -> None:
        self.current_page_name = name
        page = self.pages[name]
        page.tkraise()
        page.on_show()
        for key, button in self.sidebar_buttons.items():
            button.state(["pressed"] if key == name else ["!pressed"])

    def append_log(self, text: str) -> None:
        self.log_text.insert("end", text)
        if self.autoscroll_var.get():
            self.log_text.see("end")

    def process_log_queue(self) -> None:
        try:
            while True:
                kind, payload = self.log_queue.get_nowait()
                if kind == "log":
                    self.append_log(str(payload))
                elif kind == "status":
                    status, task = payload
                    self.process_state_var.set(status)
                    self.current_task_var.set(task)
                elif kind == "parse":
                    callback, ch = payload
                    callback(ch)
                elif kind == "finished":
                    return_code, on_finish = payload
                    self.process_manager.clear_active(return_code)
                    if on_finish is not None:
                        on_finish(return_code)
        except queue.Empty:
            pass
        self.root.after(100, self.process_log_queue)

    def _ipc_worker_loop(self) -> None:
        while True:
            request = self.ipc_requests.get()
            response: dict[str, Any] | None = None
            last_error = ""
            for attempt in range(request.retries + 1):
                try:
                    response = send_request(request.payload, DEFAULT_SOCKET_PATH, timeout_sec=IPC_TIMEOUT_SEC)
                    break
                except Exception as exc:
                    last_error = str(exc)
                    if attempt < request.retries:
                        time.sleep(request.retry_delay_sec)
            if response is None:
                response = {
                    "ok": False,
                    "request_id": request.payload.get("request_id"),
                    "command": request.payload.get("command"),
                    "error": last_error or "IPC request failed",
                }
            self.ipc_results.put((request, response))

    def enqueue_ipc(
        self,
        payload: dict[str, Any],
        callback: Callable[[dict[str, Any]], None] | None = None,
        retries: int = 0,
    ) -> None:
        payload = {"request_id": uuid.uuid4().hex, **payload}
        self.ipc_requests.put(IpcRequest(payload=payload, callback=callback, retries=retries))

    def process_ipc_results(self) -> None:
        try:
            while True:
                request, response = self.ipc_results.get_nowait()
                status = response.get("status")
                if isinstance(status, dict):
                    self.apply_realtime_status(status)
                if request.callback is not None:
                    request.callback(response)
        except queue.Empty:
            pass
        if self.close_state != CloseState.CLOSED:
            self.root.after(50, self.process_ipc_results)

    def apply_realtime_status(self, status: dict[str, Any]) -> None:
        self.realtime_status_cache = status
        session = status.get("session", {}) if isinstance(status.get("session"), dict) else {}
        if session.get("session_token") == self.realtime_session_token:
            self.realtime_session_epoch = session.get("session_epoch")
        for motor_id, info in status.get("motors", {}).items():
            try:
                motor_id = normalize_motor_id(motor_id)
            except ValueError:
                continue
            actual = info.get("actual_deg")
            commanded = info.get("commanded_deg")
            if actual is not None:
                self.update_current_position(motor_id, float(actual))
            elif commanded is not None and info.get("owned"):
                self.update_current_position(motor_id, float(commanded))
        mode = status.get("mode", "UNKNOWN")
        heartbeat_age = status.get("heartbeat_age")
        hb = "N/A" if heartbeat_age is None else f"{heartbeat_age:.2f}s"
        self.process_state_var.set(f"{mode} / heartbeat {hb}")

    def heartbeat_tick(self) -> None:
        if self.close_state in {CloseState.CLOSE_RELEASE_PENDING, CloseState.CLOSE_PROCESS_WAIT, CloseState.CLOSED}:
            return
        if self.realtime_session_owner and self.realtime_session_token:
            self.realtime_heartbeat_seq += 1
            self.enqueue_ipc(
                {
                    "command": "HEARTBEAT",
                    "session_token": self.realtime_session_token,
                    "heartbeat_seq": self.realtime_heartbeat_seq,
                    "generated_monotonic": time.monotonic(),
                },
                callback=self._on_heartbeat_ack,
                retries=0,
            )
        self.root.after(GUI_HEARTBEAT_INTERVAL_MS, self.heartbeat_tick)

    def _on_heartbeat_ack(self, response: dict[str, Any]) -> None:
        if not response.get("ok"):
            self.append_log(f"[heartbeat 오류] {response.get('error', response.get('reason', 'unknown'))}\n")

    def request_realtime_status_tick(self) -> None:
        if self.close_state == CloseState.CLOSED:
            return
        if self.realtime_session_owner or self.realtime_process_owner:
            self.enqueue_ipc({"command": "STATUS"}, retries=0)
        self.root.after(1000, self.request_realtime_status_tick)

    def start_realtime_controller_process(self) -> None:
        if self.realtime_process is not None and self.realtime_process.poll() is None:
            return
        self.realtime_process_owner_token = uuid.uuid4().hex
        command = [
            sys.executable,
            str(script_path("realtime_controller")),
            "--channel",
            self.channel_var.get(),
            "--motor-ids",
            ",".join(AK70_IDS),
            "--control-mode",
            "ak70-gui-persistent-lease",
            "--process-owner-token",
            self.realtime_process_owner_token,
        ]
        try:
            self.realtime_process = subprocess.Popen(
                command,
                cwd=PROJECT_DIR,
                shell=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            self.realtime_process_owner = True
            self.last_command = command
            self.append_log(f"\n[{now_text()}] [realtime controller 시작]\n{command_text(command)}\n")
            threading.Thread(target=self._realtime_process_reader, args=(self.realtime_process,), daemon=True).start()
        except Exception as exc:
            self.append_log(f"[realtime controller 시작 실패] {exc}\n")
            self.realtime_process_owner = False
            self.realtime_process_owner_token = None

    def _realtime_process_reader(self, process: subprocess.Popen[str]) -> None:
        try:
            assert process.stdout is not None
            for line in process.stdout:
                self.log_queue.put(("log", f"[controller] {line}"))
        except Exception as exc:
            self.log_queue.put(("log", f"[controller stdout 오류] {exc}\n"))
        code = process.wait()
        self.log_queue.put(("log", f"[controller 종료] code={code}\n"))
        if self.realtime_process is process:
            self.realtime_process_owner = False

    def ensure_ak70_realtime_session(self, after_ready: Callable[[], None]) -> None:
        if self.realtime_session_owner and self.realtime_session_token:
            after_ready()
            return
        self.start_realtime_controller_process()
        requested_token = uuid.uuid4().hex

        def on_arm(response: dict[str, Any]) -> None:
            if not response.get("ok"):
                self.append_log(f"[ARM 실패] {response.get('error')}\n")
                messagebox.showerror("ARM 실패", str(response.get("error", "controller ARM 실패")))
                return
            self.realtime_session_token = response.get("session_token") or response.get("status", {}).get("session_token") or requested_token
            self.realtime_session_epoch = response.get("session_epoch") or response.get("status", {}).get("session_epoch")
            self.realtime_session_owner = True
            self.realtime_heartbeat_seq = 0
            self.append_log(f"[ARM] session={self.realtime_session_token} epoch={self.realtime_session_epoch}\n")
            after_ready()

        self.enqueue_ipc(
            {
                "command": "ARM",
                "owner": "ak70_control_center_gui",
                "session_token": requested_token,
            },
            callback=on_arm,
            retries=20,
        )

    def active_realtime_ids(self) -> set[str]:
        status = self.realtime_status_cache
        ids = status.get("active_owned_ids", []) if isinstance(status, dict) else []
        return {normalize_motor_id(str(motor_id)) for motor_id in ids}

    def controller_motor_generations(self, motor_ids: list[str]) -> dict[str, int]:
        motors = self.realtime_status_cache.get("motors", {}) if isinstance(self.realtime_status_cache, dict) else {}
        generations: dict[str, int] = {}
        for motor_id in motor_ids:
            info = motors.get(motor_id, {})
            if isinstance(info, dict) and info.get("generation") is not None:
                generations[motor_id] = int(info["generation"])
        return generations

    def load_calibration(self) -> None:
        self.calibration_entries = {}
        self.calibration_ids = set()
        self.ak45_calibration_entries = {}
        self.ak45_calibration_ids = set()
        try:
            with CALIBRATION_PATH.open("r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
            motors = data.get("motors", {}) if isinstance(data, dict) else {}
            if isinstance(motors, dict):
                for key, entry in motors.items():
                    motor_id = normalize_motor_id(str(key))
                    self.calibration_entries[motor_id] = entry if isinstance(entry, dict) else {}
                    self.calibration_ids.add(motor_id)
                    self.motor_infos.setdefault(motor_id, MotorInfo(motor_id=motor_id)).calibrated = True
        except Exception as exc:
            self.append_log(f"[calibration 오류] {exc}\n")
            messagebox.showerror("calibration 오류", str(exc))
        try:
            with AK45_CALIBRATION_PATH.open("r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
            motors = data.get("motors", {}) if isinstance(data, dict) else {}
            if isinstance(motors, dict):
                for key, entry in motors.items():
                    motor_id = normalize_motor_id(str(key))
                    if not is_ak45_id(motor_id):
                        continue
                    self.ak45_calibration_entries[motor_id] = entry if isinstance(entry, dict) else {}
                    if isinstance(entry, dict) and entry.get("raw_zero_pos_rad") is not None:
                        self.ak45_calibration_ids.add(motor_id)
                        self.calibration_ids.add(motor_id)
                    self.motor_infos.setdefault(motor_id, MotorInfo(motor_id=motor_id)).calibrated = motor_id in self.ak45_calibration_ids
        except FileNotFoundError:
            self.append_log("[AK45 calibration] ak45_motor_calibration.yaml 없음\n")
        except Exception as exc:
            self.append_log(f"[AK45 calibration 오류] {exc}\n")

    def refresh_can_status(self) -> None:
        status, raw = self.can_reader.read(self.channel_var.get())
        self.can_status = status
        self.can_state_var.set(status.can_state)
        self.append_log(f"[CAN 상태]\n{raw}\n")
        page = self.pages.get("통신 연결")
        if isinstance(page, CommunicationPage):
            page.update_can_text()

    def parse_process_output(self, ch: str) -> None:
        self._parse_buffer += ch
        if ch != "\n":
            return
        line = self._parse_buffer.rstrip("\r\n")
        self._parse_buffer = ""
        self.parse_output_line(line)

    def parse_output_line(self, line: str) -> None:
        stripped = line.strip()
        detect = DETECT_RE.search(line)
        if detect:
            motor_id = normalize_motor_id(detect.group(1))
            calibrated = detect.group(2) == "YES"
            joint_text = detect.group(5)
            info = self.motor_infos.setdefault(motor_id, MotorInfo(motor_id=motor_id))
            info.detected = True
            info.calibrated = calibrated
            info.last_seen = now_text()
            self.detected_motors.add(motor_id)
            if joint_text != "N/A":
                self.update_current_position(motor_id, float(joint_text))
            else:
                self.refresh_position_views()
            self.detected_count_var.set(f"{len(self.detected_motors)}개")
            return
        multi = READ_MULTI_RE.search(line)
        if multi:
            motor_id = normalize_motor_id(multi.group(1))
            joint_deg = float(multi.group(2))
            info = self.motor_infos.setdefault(motor_id, MotorInfo(motor_id=motor_id))
            info.detected = True
            info.last_seen = now_text()
            self.detected_motors.add(motor_id)
            self.detected_count_var.set(f"{len(self.detected_motors)}개")
            self.update_current_position(motor_id, joint_deg)
            return
        if ACTUAL_TABLE_HEADER_RE.search(line):
            self._actual_table_active = True
            return
        if self._actual_table_active:
            actual = ACTUAL_TABLE_ROW_RE.search(line)
            if actual:
                self.update_current_position(actual.group(1), float(actual.group(2)))
                return
            if stripped and not stripped.startswith("["):
                self._actual_table_active = False
        if len(self.process_manager.active_motor_ids) == 1:
            read_single = READ_SINGLE_RE.search(line)
            if read_single:
                self.update_current_position(self.process_manager.active_motor_ids[0], float(read_single.group(1)))
                return
            for pattern in SINGLE_ACTUAL_RES:
                single = pattern.search(line)
                if single:
                    self.update_current_position(self.process_manager.active_motor_ids[0], float(single.group(1)))
                    return

    def update_current_position(self, motor_id: str, joint_deg: float) -> None:
        motor_id = normalize_motor_id(motor_id)
        if not math.isfinite(joint_deg):
            return
        if threading.get_ident() != self.main_thread_id:
            self.root.after(0, lambda: self.update_current_position(motor_id, joint_deg))
            return
        self.current_positions[motor_id] = joint_deg
        info = self.motor_infos.setdefault(motor_id, MotorInfo(motor_id=motor_id))
        info.detected = True
        info.joint_deg = joint_deg
        info.last_seen = now_text()
        self.detected_motors.add(motor_id)
        self.detected_count_var.set(f"{len(self.detected_motors)}개")
        self.refresh_position_views()

    def refresh_position_views(self) -> None:
        page = self.pages.get("통신 연결")
        if isinstance(page, CommunicationPage):
            page.refresh_current_angles()
        page = self.pages.get("단일 모터 제어")
        if isinstance(page, SingleMotorPage):
            page.refresh_current_angle()
        page = self.pages.get("다중 모터 제어")
        if isinstance(page, MultiMotorPage):
            page.refresh_current_angles()
        page = self.pages.get("홈")
        if self.current_page_name == "홈" and isinstance(page, HomePage):
            page.on_show()

    def run_read_positions(self, motor_ids: list[str]) -> None:
        motor_ids = sorted(set(motor_ids))
        active = self.active_realtime_ids()
        active_selected = [motor_id for motor_id in motor_ids if motor_id in active]
        if active_selected:
            self.append_log(f"[위치 읽기 제외] controller active ID는 STATUS 사용: {', '.join(active_selected)}\n")
            self.enqueue_ipc({"command": "STATUS"}, retries=0)
        motor_ids = [motor_id for motor_id in motor_ids if motor_id not in active]
        if not motor_ids:
            return
        if any(is_ak45_id(motor_id) for motor_id in motor_ids):
            messagebox.showinfo("AK45 위치 읽기", "AK45 위치 읽기는 AK45 원점 설정 도구에서 수행하십시오.")
            self.open_ak45_helper()
            return
        if script_exists("read_multi"):
            command = [sys.executable, str(script_path("read_multi")), "--channel", self.channel_var.get(), "--motor-ids", ",".join(motor_ids)]
            responses = [AutoResponse("Type YES to continue:", "YES", "YES")]
        elif len(motor_ids) == 1 and script_exists("read_single"):
            command = [sys.executable, str(script_path("read_single")), "--channel", self.channel_var.get(), "--motor-id", motor_ids[0]]
            responses = [AutoResponse("Type YES to continue:", "YES", "YES")]
        else:
            self.show_missing_script("read_multi")
            return
        self.process_manager.start("현재 위치 읽기", command, motor_ids, responses, on_output=self.parse_process_output)

    def validate_basic_motion(self, motor_id: str, target_deg: float, kp: float, kd: float) -> None:
        motor_id = normalize_motor_id(motor_id)
        if motor_id not in self.calibration_ids:
            raise ValueError(f"{motor_id} calibration이 없습니다.")
        if is_ak45_id(motor_id):
            if motor_id not in self.ak45_calibration_ids:
                raise ValueError(f"{motor_id} AK45 calibration이 없습니다.")
            if self.ak45_homing_states.get(motor_id) != "HOMED":
                raise ValueError(f"{motor_id} AK45는 UNHOMED 상태입니다. 수평 자세 확인이 필요합니다.")
        if not -120.0 <= target_deg <= 120.0:
            raise ValueError("목표각은 -120~+120도 범위여야 합니다.")
        current = self.require_current_angle(motor_id)
        if abs(target_deg - current) > 120.0:
            raise ValueError("현재 위치 대비 이동량은 120도 이하여야 합니다.")
        if not 0.0 < kp <= 10.0:
            raise ValueError("Kp 범위가 잘못되었습니다.")
        if not 0.0 <= kd <= 2.0:
            raise ValueError("Kd 범위가 잘못되었습니다.")

    def require_current_angle(self, motor_id: str) -> float:
        motor_id = normalize_motor_id(motor_id)
        if motor_id not in self.current_positions:
            raise ValueError(f"{motor_id} 현재 위치를 먼저 읽어야 합니다.")
        return self.current_positions[motor_id]

    def submit_ak70_targets(
        self,
        rows: list[dict[str, Any]],
        *,
        task: str,
        move_sec: float,
        max_following_error_deg: float,
        display_mode: str = "target",
    ) -> None:
        motor_ids = [normalize_motor_id(row["motor_id"]) for row in rows]
        if any(is_ak45_id(motor_id) for motor_id in motor_ids):
            raise ValueError("AK70 realtime persistent 경로는 AK70 ID만 허용합니다.")
        control_group_id = f"{task}-{uuid.uuid4().hex[:10]}"

        def send_targets() -> None:
            targets = []
            for row in rows:
                motor_id = normalize_motor_id(row["motor_id"])
                targets.append(
                    {
                        "motor_id": motor_id,
                        "position_deg": float(row["target"]),
                        "target_deg": float(row["target"]),
                        "kp": float(row["kp"]),
                        "kd": float(row["kd"]),
                        "move_sec": float(move_sec),
                        "max_following_error_deg": float(max_following_error_deg),
                        "control_group_id": control_group_id,
                        "display_mode": display_mode,
                        "session_token": self.realtime_session_token,
                        "session_epoch": self.realtime_session_epoch,
                    }
                )

            def on_ack(response: dict[str, Any]) -> None:
                if not response.get("ok"):
                    self.append_log(f"[{task} 실패] {response.get('error', response.get('errors'))}\n")
                    messagebox.showerror(task, str(response.get("error", response.get("errors", "IPC 실패"))))
                    return
                self.current_task_var.set(task)
                self.process_state_var.set("ACK 수신")
                self.append_log(f"[{task}] ACK target_generation={response.get('target_generation')}\n")

            self.enqueue_ipc(
                {
                    "command": "SET_TARGETS",
                    "session_token": self.realtime_session_token,
                    "session_epoch": self.realtime_session_epoch,
                    "control_group_id": control_group_id,
                    "targets": targets,
                },
                callback=on_ack,
                retries=2,
            )

        self.ensure_ak70_realtime_session(send_targets)

    def submit_ak70_trajectory(
        self,
        motor_id: str,
        waypoints: list[tuple[float, float]],
        *,
        kp: float,
        kd: float,
        max_following_error_deg: float,
        display_mode: str = "target",
    ) -> None:
        motor_id = normalize_motor_id(motor_id)
        if is_ak45_id(motor_id):
            raise ValueError("AK70 trajectory persistent 경로는 AK70 ID만 허용합니다.")
        plan_id = uuid.uuid4().hex
        control_group_id = f"trajectory-{plan_id[:10]}"

        def send_trajectory() -> None:
            def on_ack(response: dict[str, Any]) -> None:
                if not response.get("ok"):
                    self.append_log(f"[Trajectory 실패] {response.get('error')}\n")
                    messagebox.showerror("Trajectory 실패", str(response.get("error", "IPC 실패")))
                    return
                self.current_task_var.set("단일 Trajectory")
                self.process_state_var.set("ACK 수신")
                self.append_log(f"[Trajectory] ACK plan_id={response.get('plan_id')}\n")

            self.enqueue_ipc(
                {
                    "command": "SET_TRAJECTORY",
                    "session_token": self.realtime_session_token,
                    "session_epoch": self.realtime_session_epoch,
                    "motor_id": motor_id,
                    "control_group_id": control_group_id,
                    "plan_id": plan_id,
                    "waypoints": [{"target_deg": target, "duration": duration} for target, duration in waypoints],
                    "kp": kp,
                    "kd": kd,
                    "max_following_error_deg": max_following_error_deg,
                    "display_mode": display_mode,
                },
                callback=on_ack,
                retries=2,
            )

        self.ensure_ak70_realtime_session(send_trajectory)

    def release_selected_ak70_ids(self, motor_ids: list[str]) -> None:
        ids = [normalize_motor_id(motor_id) for motor_id in motor_ids if not is_ak45_id(motor_id)]
        if not ids:
            messagebox.showwarning("선택 없음", "TORQUE 해제할 AK70 모터를 선택하십시오.")
            return
        if not messagebox.askokcancel("TORQUE 해제 확인", RELEASE_CONFIRM_TEXT, default="cancel"):
            return

        def send_release() -> None:
            expected = self.controller_motor_generations(ids)

            def on_ack(response: dict[str, Any]) -> None:
                if not response.get("ok") and response.get("errors"):
                    self.append_log(f"[RELEASE_IDS 부분 실패] {response.get('errors')}\n")
                released = response.get("released_ids", []) if response.get("ok") or "released_ids" in response else []
                already = response.get("already_released_ids", []) if isinstance(response.get("already_released_ids"), list) else []
                for motor_id in released + already:
                    info = self.motor_infos.setdefault(motor_id, MotorInfo(motor_id=motor_id))
                    info.last_seen = now_text()
                self.append_log(f"[RELEASE_IDS] released={released} already={already} errors={response.get('errors', {})}\n")
                self.process_state_var.set("TORQUE 해제됨" if released or already else "TORQUE 해제 실패")

            self.enqueue_ipc(
                {
                    "command": "RELEASE_IDS",
                    "session_token": self.realtime_session_token,
                    "motor_ids": ids,
                    "expected_generations": expected,
                },
                callback=on_ack,
                retries=2,
            )

        self.ensure_ak70_realtime_session(send_release)

    def show_plan_table(self, title: str, columns: list[str], rows: list[list[Any]]) -> None:
        top = tk.Toplevel(self.root)
        top.title(title)
        top.geometry("900x350")
        ttk.Label(top, text=title, font=("TkDefaultFont", 14, "bold")).pack(anchor="w", padx=10, pady=8)
        tree = ttk.Treeview(top, columns=[str(i) for i in range(len(columns))], show="headings")
        for idx, col in enumerate(columns):
            tree.heading(str(idx), text=col)
            tree.column(str(idx), width=120, anchor="center")
        for row in rows:
            tree.insert("", "end", values=row)
        tree.pack(fill="both", expand=True, padx=10, pady=8)
        ttk.Button(top, text="닫기", command=top.destroy).pack(pady=8)

    def start_control(self, task: str, command: list[str], motor_ids: list[str], responses: list[AutoResponse]) -> None:
        for key, script in SCRIPTS.items():
            if str(script_path(key)) in command and not Path(script_path(key)).exists():
                self.show_missing_script(key)
                return
        if "--enter-mit" not in command:
            command = [*command, "--enter-mit"]
        self.last_command = command
        self.process_manager.start(task, command, motor_ids, responses, on_output=self.parse_process_output)

    def confirm_ak45_power(self) -> bool:
        return messagebox.askokcancel(
            "AK45 전원 확인",
            (
                "AK45-36 전원 요구사항을 확인하십시오.\n\n"
                "정격 24V\n"
                "허용 16~28V\n"
                "48V 연결 금지\n"
                "AK70과 AK45 전원 레일 분리\n"
                "CAN 통신 기준 GND 공유\n\n"
                "확인한 경우에만 계속하십시오."
            ),
        )

    def open_ak45_helper(self) -> None:
        if not script_exists("ak45_calibration_helper"):
            self.show_missing_script("ak45_calibration_helper")
            return
        if self.process_manager.is_running():
            messagebox.showerror("실행 중", "다른 제어 작업이 실행 중일 때는 AK45 도구를 열 수 없습니다.")
            return
        command = [sys.executable, str(script_path("ak45_calibration_helper")), "--channel", self.channel_var.get()]
        self.process_manager.start("AK45 원점 설정 도구", command, [])

    def selected_ak45_ids(self) -> list[str]:
        ids = sorted(motor_id for motor_id in self.selected_communication_motor_ids if is_ak45_id(motor_id))
        if not ids:
            messagebox.showwarning("AK45 선택 없음", "통신 화면에서 AK45 0x00B~0x00D 중 하나 이상을 선택하십시오.")
        return ids

    def confirm_selected_ak45_home(self) -> None:
        ids = self.selected_ak45_ids()
        if not ids:
            return
        self.load_calibration()
        missing = [motor_id for motor_id in ids if motor_id not in self.ak45_calibration_ids]
        if missing:
            messagebox.showerror("AK45 calibration 없음", f"calibration 없는 AK45: {', '.join(missing)}")
            return
        if not messagebox.askokcancel(
            "수평 자세 확인",
            "발목이 실제 수평 기준면 또는 기계적 지그에 고정되어 있는지 확인하십시오.\n센서값만으로 수평 자세를 확인할 수 없습니다.",
        ):
            return
        for motor_id in ids:
            self.ak45_homing_states[motor_id] = "HOMED"
        self.append_log(f"[AK45 Homing] HOMED: {', '.join(ids)}\n")
        self.refresh_position_views()

    def clear_selected_ak45_home(self) -> None:
        ids = self.selected_ak45_ids()
        if not ids:
            return
        for motor_id in ids:
            self.ak45_homing_states[motor_id] = "HOMING_REQUIRED"
        self.append_log(f"[AK45 Homing] 해제: {', '.join(ids)}\n")
        self.refresh_position_views()

    def start_realtime_controller(self) -> None:
        if not script_exists("realtime_controller"):
            self.show_missing_script("realtime_controller")
            return
        self.ensure_ak70_realtime_session(lambda: self.append_log("[realtime controller] ARM 완료\n"))

    def _run_quick_script(self, task: str, command: list[str]) -> None:
        self.last_command = command
        try:
            result = subprocess.run(
                command,
                cwd=PROJECT_DIR,
                shell=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=3.0,
                check=False,
            )
            self.append_log(f"\n[{now_text()}] [{task}]\n{command_text(command)}\n{result.stdout}\n")
        except Exception as exc:
            self.append_log(f"[{task} 실패] {exc}\n")
            messagebox.showerror(task, str(exc))

    def realtime_status(self) -> None:
        self.enqueue_ipc(
            {"command": "STATUS"},
            callback=lambda response: self.append_log(f"[실시간 컨트롤러 상태]\n{json.dumps(response, ensure_ascii=False, indent=2)}\n"),
            retries=1,
        )

    def send_realtime_targets(self) -> None:
        if not script_exists("realtime_targets"):
            self.show_missing_script("realtime_targets")
            return
        ids = sorted(self.selected_communication_motor_ids)
        if not ids:
            messagebox.showwarning("선택 없음", "실시간 목표를 보낼 모터를 선택하십시오.")
            return
        if any(is_ak45_id(motor_id) for motor_id in ids) and not self.confirm_ak45_power():
            return
        rows = [{"motor_id": motor_id, "target": 0.0, "kp": 8.0, "kd": 0.4} for motor_id in ids if not is_ak45_id(motor_id)]
        if rows:
            self.submit_ak70_targets(rows, task="실시간 목표 전송", move_sec=1.0, max_following_error_deg=60.0, display_mode="home")

    def stop_realtime_controller(self) -> None:
        if self.realtime_session_owner and self.realtime_session_token:
            self.enqueue_ipc(
                {"command": "RELEASE_SESSION", "session_token": self.realtime_session_token},
                callback=lambda response: self.append_log(f"[RELEASE_SESSION] ok={response.get('ok')} error={response.get('error')}\n"),
                retries=2,
            )
            self.realtime_session_owner = False
            self.realtime_session_token = None
        else:
            self.append_log("[실시간 제어 정지] 소유한 session이 없습니다.\n")

    def run_can_health(self) -> None:
        if not script_exists("can_health"):
            self.show_missing_script("can_health")
            return
        self._run_quick_script("CAN 상태 확인", [sys.executable, str(script_path("can_health")), "--channel", self.channel_var.get()])

    def emergency_stop(self) -> None:
        self.process_manager.emergency_stop()
        self.realtime_session_owner = False
        self.realtime_session_token = None
        self.enqueue_ipc(
            {"command": "ESTOP"},
            callback=lambda response: self.append_log(f"[ESTOP] ok={response.get('ok')} error={response.get('error')}\n"),
            retries=2,
        )
        self.process_state_var.set("ESTOP 요청")

    def clear_log(self) -> None:
        if messagebox.askokcancel("로그 지우기", "현재 표시된 로그를 지우시겠습니까?"):
            self.log_text.delete("1.0", "end")

    def save_log(self) -> None:
        path = filedialog.asksaveasfilename(defaultextension=".log", filetypes=[("Log", "*.log"), ("Text", "*.txt")])
        if not path:
            return
        Path(path).write_text(self.log_text.get("1.0", "end"), encoding="utf-8")

    def copy_last_command(self) -> None:
        self.root.clipboard_clear()
        self.root.clipboard_append(command_text(self.last_command))

    def show_missing_script(self, key: str) -> None:
        text = f"사용 불가: 파일을 찾을 수 없습니다. ({SCRIPTS.get(key, key)})"
        self.append_log(f"[오류] {text}\n")
        messagebox.showerror("사용 불가", text)

    def direct_zero_torque_fallback(self, motor_ids: list[str], source: str = "GUI") -> None:
        packet = pack_mit_command(0.0, 0.0, 0.0, 0.0, 0.0)
        if packet != EXPECTED_ZERO_TORQUE_PACKET:
            self.append_log("[DIRECT_ZERO_TORQUE_FALLBACK] zero-torque packet 검증 실패\n")
            return
        status_ok = False
        try:
            status_response = send_request({"command": "STATUS", "request_id": uuid.uuid4().hex}, DEFAULT_SOCKET_PATH, timeout_sec=IPC_TIMEOUT_SEC)
            status_ok = bool(status_response.get("ok"))
        except Exception:
            status_ok = False
        socket_alive = probe_datagram_socket(DEFAULT_SOCKET_PATH, timeout_sec=0.05)
        process_dead = self.realtime_process is None or self.realtime_process.poll() is not None
        if status_ok or socket_alive or (self.realtime_process_owner and not process_dead):
            self.append_log(
                "[CONTROLLER_UNREACHABLE] 직접 송신 금지: controller가 살아 있을 가능성이 있습니다. "
                "필요 시 전원을 차단하십시오.\n"
            )
            return
        lock = None
        try:
            lock = acquire_controller_lock(DEFAULT_LOCK_PATH)
        except ControllerAlreadyRunning:
            self.append_log("[DIRECT_ZERO_TORQUE_FALLBACK] lock 획득 실패, 직접 CAN 송신 금지. 전원 차단 필요.\n")
            return
        except Exception as exc:
            self.append_log(f"[DIRECT_ZERO_TORQUE_FALLBACK] lock 확인 실패: {exc}\n")
            return
        self.append_log(f"[DIRECT_ZERO_TORQUE_FALLBACK] source={source} lock=acquired ids={', '.join(sorted(set(motor_ids)))}\n")
        bus = None
        try:
            bus = can.Bus(interface="socketcan", channel=self.channel_var.get().strip() or DEFAULT_CHANNEL)
            for motor_text in sorted(set(motor_ids)):
                try:
                    motor_id = int(normalize_motor_id(motor_text), 16)
                    bus.send(can.Message(arbitration_id=motor_id, data=packet, is_extended_id=False))
                    self.append_log(f"[DIRECT_ZERO_TORQUE_FALLBACK] {motor_text} zero-torque 송신 성공\n")
                except Exception as exc:
                    self.append_log(f"[DIRECT_ZERO_TORQUE_FALLBACK] {motor_text} 실패: {exc}\n")
        except Exception as exc:
            self.append_log(f"[DIRECT_ZERO_TORQUE_FALLBACK] CAN open 실패: {exc}\n")
        finally:
            if bus is not None:
                try:
                    bus.shutdown()
                except Exception:
                    pass
            lock.close()

    def on_close(self) -> None:
        if self.close_state != CloseState.RUNNING:
            return
        if self.process_manager.is_running():
            if not messagebox.askokcancel("종료 확인", "현재 모터 제어가 실행 중입니다.\n제어를 중단하고 앱을 종료하시겠습니까?"):
                return
            self.process_manager.emergency_stop()
        self.close_state = CloseState.CLOSE_STATUS_PENDING
        self.enqueue_ipc({"command": "STATUS"}, callback=self._on_close_status, retries=1)

    def _on_close_status(self, response: dict[str, Any]) -> None:
        if self.close_state != CloseState.CLOSE_STATUS_PENDING:
            return
        status = response.get("status") if response.get("ok") else None
        active_possible = True
        if isinstance(status, dict):
            active_possible = bool(status.get("active_owned_ids")) or self.realtime_session_owner
        self.close_state = CloseState.CLOSE_CONFIRM_PENDING
        if active_possible:
            confirmed = messagebox.askokcancel(
                "종료 확인",
                (
                    "현재 하나 이상의 모터가 위치를 유지하며\n"
                    "TORQUE를 발생시키고 있습니다.\n\n"
                    "앱을 종료하면 해당 모터의 TORQUE가 해제됩니다.\n"
                    "로봇과 구조물을 지지하거나 고정했는지 확인하십시오.\n\n"
                    "정말 앱을 종료하시겠습니까?"
                ),
            )
            if not confirmed:
                self.close_state = CloseState.RUNNING
                self.append_log("[종료] 사용자가 취소했습니다. heartbeat 유지.\n")
                return
        self._send_close_release_or_shutdown()

    def _send_close_release_or_shutdown(self) -> None:
        command = close_command_for_ownership(self.realtime_process_owner, self.realtime_session_owner)
        if command is None or not self.realtime_session_token:
            self._finish_close()
            return
        payload = {"command": command, "session_token": self.realtime_session_token}
        if command == "SHUTDOWN" and self.realtime_process_owner_token:
            payload["process_owner_token"] = self.realtime_process_owner_token
        self.close_state = CloseState.CLOSE_RELEASE_PENDING
        self.append_log(f"[종료] {command} 요청\n")
        self.enqueue_ipc(payload, callback=lambda response, cmd=command: self._on_close_release_ack(cmd, response), retries=3)

    def _on_close_release_ack(self, command: str, response: dict[str, Any]) -> None:
        if self.close_state != CloseState.CLOSE_RELEASE_PENDING:
            return
        self.append_log(f"[종료] {command} ACK ok={response.get('ok')} error={response.get('error')}\n")
        if not response.get("ok"):
            messagebox.showerror("종료 실패", f"{command} ACK 실패: {response.get('error')}")
            self.close_state = CloseState.RUNNING
            return
        self.realtime_session_owner = False
        self.realtime_session_token = None
        if command == "SHUTDOWN" and self.realtime_process is not None and self.realtime_process_owner:
            self.close_state = CloseState.CLOSE_PROCESS_WAIT
            self.close_deadline_monotonic = time.monotonic() + 1.5
            self.root.after(50, self._wait_close_process)
            return
        self._finish_close()

    def _wait_close_process(self) -> None:
        if self.close_state != CloseState.CLOSE_PROCESS_WAIT:
            return
        if self.realtime_process is None or self.realtime_process.poll() is not None:
            self._finish_close()
            return
        if self.close_deadline_monotonic is not None and time.monotonic() >= self.close_deadline_monotonic:
            self.append_log("[종료] controller process 종료 확인 timeout, GUI 종료 진행\n")
            self._finish_close()
            return
        self.root.after(50, self._wait_close_process)

    def _finish_close(self) -> None:
        self.close_state = CloseState.CLOSED
        self.realtime_closing = True
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    app = AK70ControlCenterApp()
    app.run()


if __name__ == "__main__":
    main()
