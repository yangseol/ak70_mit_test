#!/usr/bin/env python3
"""Three-screen real-hardware control center for the 12-joint lower body."""

from __future__ import annotations

import json
import csv
import io
import importlib.util
import math
import os
import queue
import shutil
import subprocess
import sys
import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from tkinter import StringVar, Text, Tk, messagebox, ttk
import tkinter as tk
from typing import Any, Callable

import yaml

from mit_packet import float_to_uint
from motion_monitor import (
    build_joint_rows,
    draw_front_view,
    draw_side_leg_view,
    format_deg,
    max_error_row,
    to_bar_display_deg,
    to_bar_display_range,
    to_ui_display_deg,
    to_ui_display_range,
)
from motor_profiles import AK70_KD_MAX, get_motor_profile, is_ak45
from realtime_ipc import DEFAULT_LOCK_PATH, DEFAULT_SOCKET_PATH, probe_datagram_socket, send_request


BASE_DIR = Path(__file__).resolve().parent
CONTROLLER_PATH = BASE_DIR / "run_realtime_controller.py"
AK70_CALIBRATION_PATH = BASE_DIR / "motor_calibration.yaml"
AK45_CALIBRATION_PATH = BASE_DIR / "ak45_motor_calibration.yaml"
WALK_CYCLE_PATH = BASE_DIR / "humanoid_12dof_walk_cycle_20260706_090542.txt"
CONTROLLER_LOG_PATH = BASE_DIR / "controller.log"
IPC_SOCKET_PATH = DEFAULT_SOCKET_PATH
IPC_LOCK_PATH = DEFAULT_LOCK_PATH
CHANNEL = "can0"
TOPIC = "/humanoid/joint_targets"
ROS_TYPE = "std_msgs/msg/String"
TAB_LABELS = ("1. 시작 / 원점", "2. 수동 / 모니터", "3. Isaac Sim")
CONTROL_RATE_HZ = 100.0
HEARTBEAT_MS = 250
STATUS_MS = 200
STREAM_MS = 20
WALK_TRANSITION_SEC = 1.0
IPC_TIMEOUT_BY_COMMAND = {
    "PING": 1.0,
    "STATUS": 1.0,
    "HEARTBEAT": 1.0,
    "SET_STREAM_TARGETS": 1.0,
    "SET_GAINS": 1.0,
    "ARM": 10.0,
    "START_MOTORS": 10.0,
    "RELOAD_CALIBRATION": 3.0,
}

MOTORS = (
    (1, "right_hip_pitch", "AK70"),
    (2, "right_hip_roll", "AK70"),
    (3, "right_hip_yaw", "AK70"),
    (4, "right_knee", "AK70"),
    (5, "right_ankle_pitch", "AK70"),
    (6, "right_ankle_roll", "AK45"),
    (7, "left_hip_pitch", "AK70"),
    (8, "left_hip_roll", "AK70"),
    (9, "left_hip_yaw", "AK70"),
    (10, "left_knee", "AK70"),
    (11, "left_ankle_pitch", "AK70"),
    (12, "left_ankle_roll", "AK45"),
)
NAME_TO_ID = {name: motor_id for motor_id, name, _model in MOTORS}
ID_TO_NAME = {motor_id: name for motor_id, name, _model in MOTORS}
ID_TO_MODEL = {motor_id: model for motor_id, _name, model in MOTORS}
LEGACY_NAMES = ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")
WALK_COLUMNS = {
    "right_hip_f_joint": 1,
    "right_hip_a_joint": 2,
    "right_hip_r_joint": 3,
    "right_knee_joint": 4,
    "right_ankle_f_joint": 5,
    "right_ankle_r_joint": 6,
    "left_hip_f_joint": 7,
    "left_hip_a_joint": 8,
    "left_hip_r_joint": 9,
    "left_knee_joint": 10,
    "left_ankle_f_joint": 11,
    "left_ankle_r_joint": 12,
}
SHORT_JOINT_NAMES = {
    "hip_pitch": "Hip P",
    "hip_roll": "Hip R",
    "hip_yaw": "Hip Y",
    "knee": "Knee",
    "ankle_pitch": "Ankle P",
    "ankle_roll": "Ankle R",
}


def compact_window_geometry(screen_w: int, screen_h: int) -> tuple[int, int]:
    return min(1500, int(screen_w * 0.94)), min(900, int(screen_h * 0.90))


def compact_joint_name(joint_name: str) -> str:
    for suffix, short_name in SHORT_JOINT_NAMES.items():
        if joint_name.endswith(suffix):
            return short_name
    return joint_name


@dataclass(frozen=True)
class WalkSample:
    time_sec: float
    targets_deg: dict[int, float]


@dataclass(frozen=True)
class WalkTrajectory:
    path: Path
    samples: tuple[WalkSample, ...]
    cycle_sec: float
    sample_dt: float
    unit: str


def load_walk_cycle(path: str | Path) -> WalkTrajectory:
    path = Path(path).resolve()
    if not path.is_file():
        raise FileNotFoundError(f"보행 파일 없음:\n{path}")
    metadata: dict[str, str] = {}
    csv_lines: list[str] = []
    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line:
                continue
            if line.startswith("#"):
                content = line[1:].strip()
                if "=" in content:
                    key, value = content.split("=", 1)
                    metadata[key.strip()] = value.strip()
                continue
            csv_lines.append(line)

    if not csv_lines:
        raise ValueError(f"보행 CSV 데이터 없음: {path}")
    reader = csv.DictReader(io.StringIO("\n".join(csv_lines)))
    required = {"time_sec", *WALK_COLUMNS}
    missing = sorted(required - set(reader.fieldnames or ()))
    if missing:
        raise ValueError(f"보행 파일 필수 column 누락: {', '.join(missing)}")
    unit = metadata.get("unit", "").lower()
    if unit != "degree":
        raise ValueError(f"보행 파일 unit은 degree여야 합니다: {unit or 'missing'}")
    cycle_sec = finite_float(metadata.get("cycle_sec"))
    sample_dt = finite_float(metadata.get("sample_dt"))
    if cycle_sec <= 0.0 or sample_dt <= 0.0:
        raise ValueError("cycle_sec와 sample_dt는 0보다 커야 합니다")

    samples: list[WalkSample] = []
    previous_time: float | None = None
    for row_number, row in enumerate(reader, 2):
        try:
            sample_time = finite_float(row["time_sec"])
            targets = {motor_id: finite_float(row[column]) for column, motor_id in WALK_COLUMNS.items()}
        except Exception as exc:
            raise ValueError(f"보행 파일 row {row_number} 값 오류: {exc}") from exc
        if previous_time is not None and sample_time <= previous_time:
            raise ValueError(f"time_sec가 단조 증가하지 않음: row {row_number}")
        samples.append(WalkSample(sample_time, targets))
        previous_time = sample_time
    if len(samples) < 2:
        raise ValueError("보행 파일은 최소 2개 sample이 필요합니다")
    if samples[0].time_sec < 0.0 or cycle_sec <= samples[-1].time_sec:
        raise ValueError("cycle_sec는 마지막 sample time보다 커야 합니다")
    return WalkTrajectory(path, tuple(samples), cycle_sec, sample_dt, unit)


def interpolate_walk_cycle(trajectory: WalkTrajectory, phase: float) -> dict[int, float]:
    phase = min(max(float(phase), 0.0), trajectory.cycle_sec)
    samples = trajectory.samples
    for index in range(len(samples) - 1):
        left, right = samples[index], samples[index + 1]
        if phase <= right.time_sec:
            span = right.time_sec - left.time_sec
            ratio = 0.0 if span <= 0.0 else (phase - left.time_sec) / span
            ratio = min(max(ratio, 0.0), 1.0)
            return {
                motor_id: left.targets_deg[motor_id] + (right.targets_deg[motor_id] - left.targets_deg[motor_id]) * ratio
                for motor_id in ID_TO_NAME
            }
    last, first = samples[-1], samples[0]
    span = trajectory.cycle_sec - last.time_sec
    ratio = 1.0 if span <= 0.0 else (phase - last.time_sec) / span
    ratio = min(max(ratio, 0.0), 1.0)
    return {
        motor_id: last.targets_deg[motor_id] + (first.targets_deg[motor_id] - last.targets_deg[motor_id]) * ratio
        for motor_id in ID_TO_NAME
    }


def _atomic_write_calibration(path: Path, data: dict[str, Any]) -> None:
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Calibration file missing:\n{path}")
    backup_path = path.with_name(path.name + ".bak")
    temporary_path = path.with_name(f".{path.name}.tmp")
    shutil.copy2(path, backup_path)
    try:
        with temporary_path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(data, handle, sort_keys=False, allow_unicode=True)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_path, path)
    finally:
        if temporary_path.exists():
            temporary_path.unlink()


def save_software_zero_files(
    raw_positions: dict[int, float],
    direction_signs: dict[int, int],
) -> list[int]:
    saved: list[int] = []
    captured_at = datetime.now().astimezone().replace(microsecond=0).isoformat()
    groups = (
        (AK70_CALIBRATION_PATH, [motor_id for motor_id in raw_positions if not is_ak45(motor_id)]),
        (AK45_CALIBRATION_PATH, [motor_id for motor_id in raw_positions if is_ak45(motor_id)]),
    )
    for path, motor_ids in groups:
        if not motor_ids:
            continue
        path = path.resolve()
        if not path.is_file():
            raise FileNotFoundError(f"Calibration file missing:\n{path}")
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
        if not isinstance(data, dict) or not isinstance(data.get("motors"), dict):
            raise ValueError(f"Invalid calibration file: {path}")
        for motor_id in sorted(motor_ids):
            raw_position = finite_float(raw_positions[motor_id])
            key = motor_key(motor_id)
            entry = data["motors"].setdefault(key, {})
            entry["name"] = entry.get("name") or ID_TO_NAME[motor_id]
            entry["raw_zero_pos_rad"] = raw_position
            entry["direction_sign"] = int(entry.get("direction_sign", direction_signs.get(motor_id, 1)))
            if entry["direction_sign"] not in (-1, 1):
                raise ValueError(f"ID {motor_id} direction_sign must be -1 or 1")
            if is_ak45(motor_id):
                entry["model"] = "AK45-36-KV80"
                entry["captured_at"] = captured_at
                entry["power_cycle_verified"] = False
            else:
                if entry.get("model") == "AK45-36-KV80":
                    entry["model"] = "AK70-10"
                profile = get_motor_profile(motor_id)
                entry["raw_zero_p_uint"] = f"0x{float_to_uint(raw_position, profile.p_min, profile.p_max, 16):04X}"
                entry.setdefault("zero_command_persistent", False)
            saved.append(motor_id)
        _atomic_write_calibration(path, data)
    return saved


def load_project_model_gains() -> dict[str, dict[str, float]]:
    result: dict[str, dict[str, float]] = {}
    for model, representative_id, path, motor_ids in (
        ("AK70", 1, AK70_CALIBRATION_PATH, [1, 2, 3, 4, 5, 7, 8, 9, 10, 11]),
        ("AK45", 6, AK45_CALIBRATION_PATH, [6, 12]),
    ):
        profile = get_motor_profile(representative_id)
        kp, kd = profile.default_kp, profile.default_kd
        try:
            with path.open("r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle)
            motors = data.get("motors", {}) if isinstance(data, dict) else {}
            for motor_id in motor_ids:
                entry = motors.get(motor_key(motor_id))
                if isinstance(entry, dict):
                    kp = float(entry.get("kp", kp))
                    kd = float(entry.get("kd", kd))
                    if "kp" in entry or "kd" in entry:
                        break
        except (OSError, TypeError, ValueError, yaml.YAMLError):
            pass
        kp_low, kp_high = gain_limits(model, "kp")
        kd_low, kd_high = gain_limits(model, "kd")
        kp = min(max(kp, kp_low), kp_high)
        kd = min(max(kd, kd_low), kd_high)
        result[model] = {"kp": kp, "kd": kd}
    return result


def gain_limits(model: str, gain_name: str) -> tuple[float, float]:
    representative_id = 6 if model == "AK45" else 1
    profile = get_motor_profile(representative_id)
    if model == "AK70":
        return (0.1, 100.0) if gain_name == "kp" else (0.0, AK70_KD_MAX)
    return (
        (profile.kp_min, profile.kp_max)
        if gain_name == "kp"
        else (profile.kd_min, profile.kd_max)
    )


def motor_key(motor_id: int) -> str:
    return f"0x{motor_id:03X}"


def token_fingerprint(token: str | None) -> str:
    return f"{str(token)[:8]}..." if token else "none"


def filter_targets_for_ids(targets: dict[int, float], active_ids: set[int]) -> dict[int, float]:
    return {
        motor_id: target
        for motor_id, target in targets.items()
        if motor_id in active_ids
    }


def retain_ready_ids(active_ids: set[int], ready_ids: set[int]) -> set[int]:
    return set(active_ids).intersection(ready_ids)


def build_stream_target_items(
    snapshot: dict[int, float],
    gains_by_motor: dict[int, tuple[float, float]] | None = None,
) -> list[dict[str, Any]]:
    items: list[dict[str, Any]] = []
    for motor_id, value in sorted(snapshot.items()):
        item: dict[str, Any] = {
            "motor_id": motor_key(motor_id),
            "position_deg": value,
            "target_deg": value,
            "move_sec": 0.0,
        }
        if gains_by_motor and motor_id in gains_by_motor:
            item["kp"], item["kd"] = gains_by_motor[motor_id]
        items.append(item)
    return items


def finite_float(value: Any) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("joint target must be finite")
    return result


def ros2_available() -> bool:
    try:
        return importlib.util.find_spec("rclpy") is not None and importlib.util.find_spec("std_msgs.msg") is not None
    except (ImportError, ModuleNotFoundError, ValueError):
        return False


def parse_isaac_payload(text: str) -> tuple[dict[int, float], bool]:
    data = json.loads(text)
    if not isinstance(data, dict):
        raise ValueError("JSON root must be an object")
    targets: dict[int, float] = {}
    legacy_without_side = False

    for name, motor_id in NAME_TO_ID.items():
        if name in data:
            targets[motor_id] = finite_float(data[name])

    for side in ("right", "left"):
        nested = data.get(side)
        if isinstance(nested, dict):
            for short_name in LEGACY_NAMES:
                if short_name in nested:
                    targets[NAME_TO_ID[f"{side}_{short_name}"]] = finite_float(nested[short_name])

    present_legacy = [name for name in LEGACY_NAMES if name in data]
    if present_legacy:
        side = str(data.get("side", "right")).lower()
        if side not in {"right", "left"}:
            raise ValueError("side must be right or left")
        legacy_without_side = "side" not in data
        for short_name in present_legacy:
            targets[NAME_TO_ID[f"{side}_{short_name}"]] = finite_float(data[short_name])

    if not targets:
        raise ValueError("supported joint target not found")
    return targets, legacy_without_side


@dataclass
class IpcWork:
    payload: dict[str, Any]
    callback: Callable[[dict[str, Any]], None] | None = None
    timeout: float = 1.0


class ActualMarker(tk.Canvas):
    def __init__(self, parent: tk.Misc) -> None:
        super().__init__(parent, height=10, highlightthickness=0, bg="#eeeeee")
        self.low = -120.0
        self.high = 120.0
        self.actual: float | None = None
        self.bind("<Configure>", lambda _event: self.redraw())

    def update_value(self, actual: float | None, low: float, high: float) -> None:
        self.actual, self.low, self.high = actual, low, high
        self.redraw()

    def redraw(self) -> None:
        self.delete("all")
        width = max(2, self.winfo_width())
        self.create_line(2, 5, width - 2, 5, fill="#aaaaaa", width=2)
        if self.actual is None or self.high <= self.low:
            return
        ratio = (min(max(self.actual, self.low), self.high) - self.low) / (self.high - self.low)
        x = 2 + ratio * (width - 4)
        self.create_rectangle(x - 3, 1, x + 3, 9, fill="#1565c0", outline="")


class RosSubscriber:
    def __init__(self, output: queue.Queue[tuple[str, Any]]) -> None:
        self.output = output
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None

    def start(self) -> None:
        if self.thread is not None and self.thread.is_alive():
            return
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run, name="isaac-ros2-subscriber", daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()

    def _run(self) -> None:
        node = None
        try:
            import rclpy
            from std_msgs.msg import String

            rclpy.init(args=None)
            node = rclpy.create_node("ak70_control_center_joint_targets")

            def receive(message: String) -> None:
                self.output.put(("ros_message", (message.data, time.monotonic(), time.time())))

            node.create_subscription(String, TOPIC, receive, 10)
            self.output.put(("ros_state", "수신 대기"))
            while not self.stop_event.is_set() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
        except Exception as exc:
            self.output.put(("ros_error", str(exc)))
        finally:
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    pass
            try:
                import rclpy
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass
            self.output.put(("ros_state", "정지"))


class ControlCenterApp:
    def __init__(self) -> None:
        self.root = Tk()
        self.root.title("AK70 / AK45 실물 로봇 제어")
        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()
        window_w, window_h = compact_window_geometry(screen_w, screen_h)
        origin_x = max(0, (screen_w - window_w) // 2)
        origin_y = max(0, (screen_h - window_h) // 2)
        self.root.geometry(f"{window_w}x{window_h}+{origin_x}+{origin_y}")
        self.root.minsize(1050, 680)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.can_var = StringVar(value="UNKNOWN")
        self.controller_var = StringVar(value="STOPPED")
        self.detected_var = StringVar(value="0 / 12")
        self.isaac_top_var = StringVar(value="STOPPED")
        self.mode_var = StringVar(value="MANUAL")
        self.start_busy = False
        self.closing = False
        self._closing = False
        self._destroyed = False
        self._process_events_after_id: str | None = None
        self._heartbeat_after_id: str | None = None
        self._status_after_id: str | None = None
        self._stream_after_id: str | None = None
        self._walk_after_id: str | None = None
        self._can_after_id: str | None = None
        self._finish_close_after_id: str | None = None
        self.motion_monitor_after_id: str | None = None
        self.session_token: str | None = None
        self.session_epoch: int | None = None
        self.heartbeat_seq = 0
        self._one_click_arm_complete = False
        self._heartbeat_enabled = False
        self._first_heartbeat_logged = False
        self._session_lost_logged = False
        self.controller_process: subprocess.Popen[str] | None = None
        self.controller_reader_thread: threading.Thread | None = None
        self.controller_output_tail: deque[str] = deque(maxlen=20)
        self.process_owner_token: str | None = None
        self.controller_owned = False
        self.status_cache: dict[str, Any] = {}
        self.ready_ids: set[int] = set()
        self.actual: dict[int, float | None] = {motor_id: None for motor_id in ID_TO_NAME}
        self.raw_positions: dict[int, float | None] = {motor_id: None for motor_id in ID_TO_NAME}
        self.direction_signs: dict[int, int] = {motor_id: 1 for motor_id in ID_TO_NAME}
        self.targets: dict[int, float] = {motor_id: 0.0 for motor_id in ID_TO_NAME}
        self.limits: dict[int, tuple[float, float]] = {motor_id: (-120.0, 120.0) for motor_id in ID_TO_NAME}
        self.suppress_scale = False
        self._updating_slider_programmatically = False
        self._last_slider_values: dict[int, float] = {motor_id: 0.0 for motor_id in ID_TO_NAME}
        self.manual_panel_visible = False
        self.dirty_targets: dict[int, float] = {}
        self.stream_inflight = False
        self.status_inflight = False
        self._ipc_error_log_times: dict[tuple[str, str], float] = {}
        self.pending_after_start: list[Callable[[], None]] = []
        self.legacy_notice_logged = False
        self.isaac_following = False
        self.ros_available = ros2_available()
        self.ros_receive_times: deque[float] = deque(maxlen=200)
        self.walk_trajectory: WalkTrajectory | None = None
        self.walk_load_error: str | None = None
        try:
            self.walk_trajectory = load_walk_cycle(WALK_CYCLE_PATH)
        except Exception as exc:
            self.walk_load_error = str(exc)
        self.walk_running = False
        self.walk_repeat = False
        self.walk_stage = "STOPPED"
        self.walk_transition_started = 0.0
        self.walk_cycle_started = 0.0
        self.walk_transition_from: dict[int, float] = {}
        self.walk_clamp_logged: set[int] = set()
        self.walk_active_motor_ids: set[int] = set()
        self._isaac_logged_ready_signature: tuple[int, ...] | None = None
        self.model_gains = load_project_model_gains()
        self.gain_applied_values: dict[int, tuple[float, float]] = {}

        self.ipc_queue: queue.Queue[IpcWork | None] = queue.Queue()
        self.event_queue: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.ipc_thread = threading.Thread(target=self._ipc_loop, name="gui-ipc", daemon=True)
        self.ipc_thread.start()
        self.ros = RosSubscriber(self.event_queue)

        self.scale_vars: dict[int, tk.DoubleVar] = {}
        self.actual_vars: dict[int, StringVar] = {}
        self.target_vars: dict[int, StringVar] = {}
        self.state_vars: dict[int, StringVar] = {}
        self.markers: dict[int, ActualMarker] = {}
        self.log_lines: list[str] = []
        self.log_window: tk.Toplevel | None = None
        self.log_text: Text | None = None
        self.build_ui()

        self._process_events_after_id = self.root.after(10, self.process_events)
        self._heartbeat_after_id = self.root.after(HEARTBEAT_MS, self.heartbeat_tick)
        self._status_after_id = self.root.after(STATUS_MS, self.status_tick)
        self._stream_after_id = self.root.after(STREAM_MS, self.stream_tick)
        self._walk_after_id = self.root.after(STREAM_MS, self.walk_tick)
        self._can_after_id = self.root.after(200, self.refresh_can_async)
        self.motion_monitor_after_id = self.root.after(50, self.update_motion_monitor)

    def build_ui(self) -> None:
        top = ttk.Frame(self.root, padding=(10, 5))
        top.pack(fill="x")
        ttk.Label(top, text="AK70 / AK45 실물 로봇 제어", font=("TkDefaultFont", 12, "bold")).pack(side="left")
        ttk.Button(top, text="로그 보기", command=self.show_log_window).pack(side="right", padx=(8, 0))
        tk.Button(
            top,
            text="전체 TORQUE 해제",
            command=self.release_all,
            bg="#c62828",
            fg="white",
            activebackground="#b71c1c",
            activeforeground="white",
            font=("TkDefaultFont", 10, "bold"),
            padx=12,
            pady=5,
        ).pack(side="right")

        notebook = ttk.Notebook(self.root)
        self.notebook = notebook
        notebook.pack(fill="both", expand=True, padx=8, pady=(0, 4))
        start_tab = ttk.Frame(notebook)
        manual_monitor_tab = ttk.Frame(notebook)
        isaac_tab = ttk.Frame(notebook)
        notebook.add(start_tab, text=TAB_LABELS[0])
        notebook.add(manual_monitor_tab, text=TAB_LABELS[1])
        notebook.add(isaac_tab, text=TAB_LABELS[2])
        self.build_start_tab(start_tab)
        self.build_manual_monitor_tab(manual_monitor_tab)
        self.build_isaac_tab(isaac_tab)

        self.recent_status_var = StringVar(value="최근 상태: 대기")
        ttk.Label(self.root, textvariable=self.recent_status_var, anchor="w").pack(fill="x", padx=10, pady=(0, 5))

    def build_start_tab(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(0, weight=1)
        parent.columnconfigure(1, weight=1)
        parent.rowconfigure(3, weight=1)

        status = ttk.LabelFrame(parent, text="연결 상태", padding=8)
        status.grid(row=0, column=0, columnspan=2, sticky="ew", padx=10, pady=(8, 4))
        for column, (label, variable) in enumerate(
            (
                ("CAN", self.can_var),
                ("Controller", self.controller_var),
                ("감지 motor", self.detected_var),
                ("Isaac ROS", self.isaac_top_var),
            )
        ):
            block = ttk.Frame(status)
            block.grid(row=0, column=column, sticky="w", padx=(0, 22))
            ttk.Label(block, text=label, font=("TkDefaultFont", 9)).pack(anchor="w")
            ttk.Label(block, textvariable=variable, font=("TkDefaultFont", 10, "bold")).pack(anchor="w")

        buttons = ttk.Frame(parent, padding=(10, 4))
        buttons.grid(row=1, column=0, columnspan=2, sticky="ew")
        buttons.columnconfigure(0, weight=1)
        buttons.columnconfigure(1, weight=1)
        ttk.Button(buttons, text="전체 모터 원클릭 시작", command=self.start_all_motors).grid(row=0, column=0, sticky="ew", padx=(0, 5), ipady=10)
        ttk.Button(buttons, text="현재 자세 전체 원점 저장", command=self.save_all_software_zero).grid(row=0, column=1, sticky="ew", padx=(5, 0), ipady=10)

        gains = ttk.LabelFrame(parent, text="Kp / Kd 빠른 조절", padding=8)
        gains.grid(row=2, column=0, columnspan=2, sticky="ew", padx=10, pady=4)
        self.gain_value_vars: dict[tuple[str, str], StringVar] = {}
        ttk.Label(gains, text="Motor", width=8).grid(row=0, column=0, padx=4)
        ttk.Label(gains, text="Gain", width=6).grid(row=0, column=1, padx=4)
        ttk.Label(gains, text="현재값", width=12).grid(row=0, column=2, padx=4)
        for model_index, model in enumerate(("AK70", "AK45")):
            for gain_index, (gain_name, step) in enumerate((("kp", 5.0), ("kd", 0.1))):
                row = 1 + model_index * 2 + gain_index
                variable = StringVar()
                self.gain_value_vars[(model, gain_name)] = variable
                self._refresh_gain_label(model, gain_name)
                ttk.Label(gains, text=model).grid(row=row, column=0, padx=4, pady=2)
                ttk.Label(gains, text=gain_name.upper()).grid(row=row, column=1, padx=4, pady=2)
                ttk.Label(gains, textvariable=variable, width=12).grid(row=row, column=2, padx=4, pady=2)
                ttk.Button(
                    gains,
                    text=f"-{step:g}",
                    command=lambda m=model, g=gain_name, s=step: self.adjust_model_gain(m, g, -s),
                ).grid(row=row, column=3, padx=3, pady=2)
                ttk.Button(
                    gains,
                    text=f"+{step:g}",
                    command=lambda m=model, g=gain_name, s=step: self.adjust_model_gain(m, g, s),
                ).grid(row=row, column=4, padx=3, pady=2)

        status_table = ttk.LabelFrame(parent, text="Motor 상태", padding=8)
        status_table.grid(row=3, column=0, columnspan=2, sticky="nsew", padx=10, pady=(4, 8))
        status_table.columnconfigure(0, weight=1)
        status_table.columnconfigure(1, weight=1)
        self._build_start_motor_column(status_table, "오른쪽 다리", MOTORS[:6], 0)
        self._build_start_motor_column(status_table, "왼쪽 다리", MOTORS[6:], 1)

    def _build_start_motor_column(
        self,
        parent: ttk.Frame,
        title: str,
        motors: tuple[tuple[int, str, str], ...],
        column: int,
    ) -> None:
        frame = ttk.Frame(parent)
        frame.grid(row=0, column=column, sticky="nsew", padx=6)
        ttk.Label(frame, text=title, font=("TkDefaultFont", 10, "bold")).grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 4))
        for header_col, text in enumerate(("ID", "Joint", "Type", "State")):
            ttk.Label(frame, text=text, font=("TkDefaultFont", 9, "bold")).grid(row=1, column=header_col, sticky="w", padx=3)
        for row, (motor_id, name, model) in enumerate(motors, 2):
            ttk.Label(frame, text=str(motor_id), width=4).grid(row=row, column=0, sticky="w", padx=3, pady=1)
            ttk.Label(frame, text=name, width=20).grid(row=row, column=1, sticky="w", padx=3, pady=1)
            ttk.Label(frame, text=model, width=6).grid(row=row, column=2, sticky="w", padx=3, pady=1)
            state = StringVar(value="NOT STARTED")
            self.state_vars[motor_id] = state
            ttk.Label(frame, textvariable=state, width=14).grid(row=row, column=3, sticky="w", padx=3, pady=1)

    def build_manual_monitor_tab(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(1, weight=1)

        status = ttk.Frame(parent, padding=(8, 5))
        status.grid(row=0, column=0, sticky="ew")
        self.motion_mode_var = StringVar(value="Source: MANUAL")
        self.motion_ready_var = StringVar(value="READY: 0/12")
        self.motion_error_var = StringVar(value="Max Error: N/A")
        self.motion_state_var = StringVar(value="Animation: RUNNING")
        self.manual_toggle_var = StringVar(value="수동 조작 열기")
        for variable in (self.motion_mode_var, self.motion_ready_var, self.motion_error_var, self.motion_state_var):
            ttk.Label(status, textvariable=variable, font=("TkDefaultFont", 10, "bold")).pack(side="left", padx=(0, 18))
        ttk.Button(status, textvariable=self.manual_toggle_var, command=self.toggle_manual_panel).pack(side="right", padx=(6, 0))
        ttk.Button(status, text="모든 목표 0°", command=self.all_targets_zero).pack(side="right", padx=(6, 0))
        ttk.Button(status, text="현재값 유지", command=self.align_targets_to_actual).pack(side="right", padx=(6, 0))

        canvas_area = ttk.Frame(parent, padding=(8, 0, 8, 4))
        canvas_area.grid(row=1, column=0, sticky="nsew")
        canvas_area.columnconfigure(0, weight=1)
        canvas_area.columnconfigure(1, weight=1)
        canvas_area.rowconfigure(0, weight=1)
        canvas_area.rowconfigure(1, weight=1)
        self.front_canvas = tk.Canvas(canvas_area, width=430, height=420, bg="white", highlightthickness=1, highlightbackground="#cfd8dc")
        self.right_side_canvas = tk.Canvas(canvas_area, width=390, height=205, bg="white", highlightthickness=1, highlightbackground="#cfd8dc")
        self.left_side_canvas = tk.Canvas(canvas_area, width=390, height=205, bg="white", highlightthickness=1, highlightbackground="#cfd8dc")
        self.front_canvas.grid(row=0, column=0, rowspan=2, sticky="nsew", padx=(0, 5), pady=(0, 2))
        self.right_side_canvas.grid(row=0, column=1, sticky="nsew", padx=(5, 0), pady=(0, 3))
        self.left_side_canvas.grid(row=1, column=1, sticky="nsew", padx=(5, 0), pady=(3, 2))
        self.front_canvas.bind("<Configure>", lambda _event: self.redraw_motion_monitor_once())
        self.right_side_canvas.bind("<Configure>", lambda _event: self.redraw_motion_monitor_once())
        self.left_side_canvas.bind("<Configure>", lambda _event: self.redraw_motion_monitor_once())

        self.manual_panel_container = ttk.LabelFrame(parent, text="수동 조작 패널", padding=(6, 4))
        self.manual_panel_container.grid(row=2, column=0, sticky="ew", padx=8, pady=(0, 6))
        self.build_manual_controls(self.manual_panel_container)
        if not self.manual_panel_visible:
            self.manual_panel_container.grid_remove()

    def build_manual_controls(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(0, weight=1)
        parent.columnconfigure(1, weight=1)
        self._build_manual_motor_column(parent, MOTORS[:6], 0)
        self._build_manual_motor_column(parent, MOTORS[6:], 1)

    def _build_manual_motor_column(
        self,
        parent: ttk.Frame,
        motors: tuple[tuple[int, str, str], ...],
        column: int,
    ) -> None:
        frame = ttk.Frame(parent)
        frame.grid(row=0, column=column, sticky="ew", padx=4)
        frame.columnconfigure(4, weight=1)
        font = ("TkDefaultFont", 8)
        for row, (motor_id, name, _model) in enumerate(motors):
            short_name = compact_joint_name(name)
            ttk.Label(frame, text=str(motor_id), width=3, font=font).grid(row=row, column=0, sticky="w", padx=2, pady=1)
            ttk.Label(frame, text=short_name, width=8, font=font).grid(row=row, column=1, sticky="w", padx=2, pady=1)
            actual_var = StringVar(value="A:N/A")
            target_var = StringVar(value="T:N/A")
            self.actual_vars[motor_id] = actual_var
            self.target_vars[motor_id] = target_var
            ttk.Label(frame, textvariable=actual_var, width=8, font=font).grid(row=row, column=2, sticky="w", padx=2, pady=1)
            ttk.Label(frame, textvariable=target_var, width=8, font=font).grid(row=row, column=3, sticky="w", padx=2, pady=1)
            marker = ActualMarker(frame)
            self.markers[motor_id] = marker
            variable = tk.DoubleVar(value=0.0)
            self.scale_vars[motor_id] = variable
            scale = ttk.Scale(
                frame,
                from_=-120.0,
                to=120.0,
                variable=variable,
                command=lambda value, mid=motor_id: self.on_slider(mid, value),
            )
            scale.grid(row=row, column=4, sticky="ew", padx=2, pady=1)
            scale.configure(state="disabled")
            setattr(self, f"scale_{motor_id}", scale)

    def toggle_manual_panel(self) -> None:
        self.manual_panel_visible = not self.manual_panel_visible
        if self.manual_panel_visible:
            self.manual_panel_container.grid()
            self.manual_toggle_var.set("수동 조작 닫기")
        else:
            self.manual_panel_container.grid_remove()
            self.manual_toggle_var.set("수동 조작 열기")
        self.redraw_motion_monitor_once()

    def build_isaac_tab(self, parent: ttk.Frame) -> None:
        panel = ttk.Frame(parent, padding=10)
        panel.pack(fill="both", expand=True)
        panel.columnconfigure(0, weight=1)
        panel.columnconfigure(1, weight=1)
        panel.rowconfigure(0, weight=1)
        ros_panel = ttk.LabelFrame(panel, text="Isaac Sim 실시간 수신", padding=10)
        ros_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        self.ros_topic_var = StringVar(value=TOPIC)
        self.ros_last_time_var = StringVar(value="--")
        self.ros_last_values_var = StringVar(value="--")
        self.ros_hz_var = StringVar(value="0.0 Hz")
        ros_initial = "STOPPED" if self.ros_available else "Isaac Sim ROS 2: 사용 불가"
        self.ros_connection_var = StringVar(value=ros_initial)
        self.isaac_mode_detail_var = StringVar(value="Isaac mode: STOPPED")
        self.isaac_ready_detail_var = StringVar(value="READY motors: 없음")
        self.isaac_applied_detail_var = StringVar(value="Applied joints: 없음")
        self.isaac_ignored_detail_var = StringVar(value="Ignored joints: 없음")
        self.isaac_top_var.set(ros_initial)
        fields = (
            ("Topic", self.ros_topic_var),
            ("ROS 사용", StringVar(value="가능" if self.ros_available else "사용 불가")),
            ("최근 수신 시간", self.ros_last_time_var),
            ("수신 Hz", self.ros_hz_var),
        )
        for row, (label, variable) in enumerate(fields):
            ttk.Label(ros_panel, text=label, width=14).grid(row=row, column=0, sticky="nw", pady=3)
            ttk.Label(ros_panel, textvariable=variable).grid(row=row, column=1, sticky="nw", pady=3)
        ttk.Label(ros_panel, text="최근 target", width=14).grid(row=4, column=0, sticky="nw", pady=3)
        ttk.Label(ros_panel, textvariable=self.ros_last_values_var, justify="left", wraplength=420).grid(row=4, column=1, sticky="nw", pady=3)
        ros_buttons = ttk.Frame(ros_panel)
        ros_buttons.grid(row=5, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        self.isaac_start_button = ttk.Button(ros_buttons, text="Isaac Sim 추종 시작", command=self.start_isaac_follow)
        self.isaac_start_button.pack(side="left", padx=(0, 10), ipady=5)
        ttk.Button(ros_buttons, text="Isaac Sim 추종 정지", command=self.stop_isaac_follow).pack(side="left", ipady=5)
        if not self.ros_available:
            self.isaac_start_button.configure(state="disabled")

        walk_panel = ttk.LabelFrame(panel, text="로컬 보행 프리셋", padding=10)
        walk_panel.grid(row=0, column=1, sticky="nsew", padx=(5, 0))
        trajectory = self.walk_trajectory
        self.walk_file_var = StringVar(value=WALK_CYCLE_PATH.name)
        self.walk_load_var = StringVar(value="로드 완료" if trajectory else (self.walk_load_error or "로드 실패"))
        self.walk_samples_var = StringVar(value=str(len(trajectory.samples)) if trajectory else "0")
        self.walk_time_var = StringVar(value="0.000 s")
        self.walk_cycle_var = StringVar(value="0")
        self.walk_cycle_sec_var = StringVar(value=f"{trajectory.cycle_sec:.6f} s" if trajectory else "--")
        self.walk_state_var = StringVar(value="로컬 보행 프리셋: 사용 가능" if trajectory else "사용 불가")
        walk_fields = (
            ("파일명", self.walk_file_var),
            ("샘플 수", self.walk_samples_var),
            ("cycle_sec", self.walk_cycle_sec_var),
            ("현재 cycle", self.walk_cycle_var),
            ("상태", self.walk_state_var),
        )
        for row, (label, variable) in enumerate(walk_fields):
            ttk.Label(walk_panel, text=label, width=12).grid(row=row, column=0, sticky="w", pady=3)
            ttk.Label(walk_panel, textvariable=variable, wraplength=420).grid(row=row, column=1, sticky="w", pady=3)
        walk_buttons = ttk.Frame(walk_panel)
        walk_buttons.grid(row=len(walk_fields), column=0, columnspan=2, sticky="ew", pady=(10, 0))
        self.walk_once_button = ttk.Button(walk_buttons, text="보행 1회 실행", command=lambda: self.start_walk(False))
        self.walk_repeat_button = ttk.Button(walk_buttons, text="보행 반복 시작", command=lambda: self.start_walk(True))
        self.walk_stop_button = ttk.Button(walk_buttons, text="보행 정지", command=self.stop_walk)
        self.walk_once_button.pack(side="left", padx=(0, 8), ipady=5)
        self.walk_repeat_button.pack(side="left", padx=(0, 8), ipady=5)
        self.walk_stop_button.pack(side="left", ipady=5)
        if trajectory is None:
            self.walk_once_button.configure(state="disabled")
            self.walk_repeat_button.configure(state="disabled")

    def _motion_value_maps(self) -> tuple[dict[int, float | None], dict[int, float | None]]:
        motors = self.status_cache.get("motors", {}) if isinstance(self.status_cache, dict) else {}
        actuals: dict[int, float | None] = {}
        targets: dict[int, float | None] = {}
        for motor_id in ID_TO_NAME:
            if motor_id not in self.ready_ids:
                actuals[motor_id] = None
                targets[motor_id] = None
                continue
            info = motors.get(motor_key(motor_id), {}) if isinstance(motors, dict) else {}
            actual = info.get("actual_deg") if isinstance(info, dict) else None
            target = info.get("target_deg") if isinstance(info, dict) else None
            if actual is None:
                actual = self.actual.get(motor_id)
            if target is None:
                target = self.targets.get(motor_id)
            actuals[motor_id] = to_ui_display_deg(motor_id, None if actual is None else float(actual))
            targets[motor_id] = to_ui_display_deg(motor_id, None if target is None else float(target))
        return actuals, targets

    def _display_deg(self, motor_id: int, value: float | None) -> float | None:
        return to_ui_display_deg(motor_id, value)

    def _display_range(self, motor_id: int, low: float, high: float) -> tuple[float, float]:
        return to_ui_display_range(motor_id, low, high)

    def _bar_display_deg(self, motor_id: int, value: float | None) -> float | None:
        return to_bar_display_deg(motor_id, self._display_deg(motor_id, value))

    def _bar_display_range(self, motor_id: int, low: float, high: float) -> tuple[float, float]:
        display_low, display_high = self._display_range(motor_id, low, high)
        return to_bar_display_range(motor_id, display_low, display_high)

    def _format_compact_display(self, prefix: str, motor_id: int, value: float | None) -> str:
        display = self._display_deg(motor_id, value)
        return f"{prefix}:N/A" if display is None else f"{prefix}:{display:+.1f}"

    def redraw_motion_monitor_once(self) -> None:
        if self._closing:
            return
        if (
            not hasattr(self, "front_canvas")
            or not hasattr(self, "right_side_canvas")
            or not hasattr(self, "left_side_canvas")
        ):
            return
        try:
            actuals, targets = self._motion_value_maps()
            draw_front_view(self.front_canvas, actuals, targets, set(self.ready_ids))
            draw_side_leg_view(self.right_side_canvas, actuals, targets, set(self.ready_ids), "right")
            draw_side_leg_view(self.left_side_canvas, actuals, targets, set(self.ready_ids), "left")
        except tk.TclError:
            return

    def update_motion_monitor(self) -> None:
        self.motion_monitor_after_id = None
        if self._closing:
            return
        try:
            actuals, targets = self._motion_value_maps()
            state_by_id = {
                motor_id: self.state_vars[motor_id].get()
                for motor_id in ID_TO_NAME
                if motor_id in self.state_vars
            }
            controller_fault = bool(self.status_cache.get("bus_fault")) or str(self.status_cache.get("mode")) in {"FAULT", "ESTOPPED"}
            rows = build_joint_rows(MOTORS, set(self.ready_ids), targets, actuals, state_by_id, controller_fault)
            max_row = max_error_row(rows)
            self.motion_mode_var.set(f"Source: {self.mode_var.get()}")
            self.motion_ready_var.set(f"READY: {len(self.ready_ids)}/12")
            if max_row is None or max_row.error is None:
                self.motion_error_var.set("Max Error: N/A")
            else:
                self.motion_error_var.set(
                    f"Max Error: {abs(max_row.error):.1f}° / "
                    f"ID {max_row.motor_id} {compact_joint_name(max_row.joint)}"
                )
            self.motion_state_var.set("Animation: STOPPING" if self._closing else "Animation: RUNNING")
            self._update_recent_status(rows)
            draw_front_view(self.front_canvas, actuals, targets, set(self.ready_ids))
            draw_side_leg_view(self.right_side_canvas, actuals, targets, set(self.ready_ids), "right")
            draw_side_leg_view(self.left_side_canvas, actuals, targets, set(self.ready_ids), "left")
            if self._closing:
                return
            self.motion_monitor_after_id = self.root.after(50, self.update_motion_monitor)
        except tk.TclError:
            return

    def _update_recent_status(self, rows: list[Any]) -> None:
        if not hasattr(self, "recent_status_var"):
            return
        max_row = max_error_row(rows)
        if max_row is None:
            self.recent_status_var.set(f"최근 상태: Source {self.mode_var.get()} / READY {len(self.ready_ids)}/12")
            return
        self.recent_status_var.set(
            f"최근 상태: ID {max_row.motor_id} {compact_joint_name(max_row.joint)} "
            f"target {format_deg(max_row.target)}, actual {format_deg(max_row.actual)}"
        )

    def log(self, message: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S")
        line = f"[{stamp}] {message.rstrip()}"
        self.log_lines.append(line)
        if hasattr(self, "recent_status_var"):
            self.recent_status_var.set(f"최근 상태: {message.rstrip().splitlines()[0]}")
        if self.log_text is not None:
            try:
                self.log_text.configure(state="normal")
                self.log_text.insert("end", line + "\n")
                self.log_text.see("end")
                self.log_text.configure(state="disabled")
            except tk.TclError:
                self.log_text = None

    def show_log_window(self) -> None:
        if self.log_window is not None:
            try:
                if self.log_window.winfo_exists():
                    self.log_window.deiconify()
                    self.log_window.lift()
                    self.log_window.focus_force()
                    return
            except tk.TclError:
                self.log_window = None
                self.log_text = None
        window = tk.Toplevel(self.root)
        window.title("상세 로그")
        window.geometry("900x420")
        text = Text(window, height=20, wrap="word", state="normal")
        text.pack(fill="both", expand=True)
        text.insert("end", "\n".join(self.log_lines) + ("\n" if self.log_lines else ""))
        text.configure(state="disabled")
        self.log_window = window
        self.log_text = text

        def closed() -> None:
            self.log_window = None
            self.log_text = None
            try:
                window.destroy()
            except tk.TclError:
                pass

        window.protocol("WM_DELETE_WINDOW", closed)

    def log_ipc_error(self, command: str, error: str) -> None:
        error_text = str(error)
        dedupe_error = "IPC request timeout" if error_text.startswith("IPC request timeout:") else error_text
        key = (str(command), dedupe_error)
        now = time.monotonic()
        previous = self._ipc_error_log_times.get(key)
        if previous is not None and now - previous < 1.0:
            return
        self._ipc_error_log_times[key] = now
        self.log(f"{command} 실패: {error_text}")

    def _ipc_loop(self) -> None:
        while True:
            work = self.ipc_queue.get()
            if work is None:
                return
            try:
                payload = dict(work.payload)
                payload.setdefault("request_id", uuid.uuid4().hex)
                response = send_request(payload, IPC_SOCKET_PATH, timeout_sec=work.timeout)
            except Exception as exc:
                response = {"ok": False, "command": work.payload.get("command"), "error": str(exc)}
            self.event_queue.put(("ipc_result", (work, response)))

    def enqueue_ipc(
        self,
        payload: dict[str, Any],
        callback: Callable[[dict[str, Any]], None] | None = None,
        timeout: float | None = None,
    ) -> None:
        command = str(payload.get("command", "UNKNOWN"))
        effective_timeout = IPC_TIMEOUT_BY_COMMAND.get(command, 1.0) if timeout is None else float(timeout)
        self.ipc_queue.put(IpcWork(payload, callback, effective_timeout))

    def process_events(self) -> None:
        if self._closing:
            return
        self._process_events_after_id = None
        try:
            for _ in range(100):
                try:
                    event, value = self.event_queue.get_nowait()
                except queue.Empty:
                    break
                if event == "ipc_result":
                    work, response = value
                    if work.callback is not None:
                        work.callback(response)
                    if not response.get("ok") and response.get("error") and not response.get("_gui_error_handled"):
                        self.log_ipc_error(
                            str(work.payload.get("command", "UNKNOWN")),
                            str(response["error"]),
                        )
                    if isinstance(response.get("status"), dict) and not response.get("_session_lost"):
                        self.apply_status(response["status"])
                elif event == "start_result":
                    self.on_start_result(value)
                elif event == "can_state":
                    self.can_var.set(value)
                elif event == "controller_log":
                    self.log(value)
                elif event == "ros_message":
                    self.on_ros_message(*value)
                elif event == "ros_state":
                    self.ros_connection_var.set(value)
                    self.isaac_top_var.set(value)
                elif event == "ros_error":
                    self.ros_connection_var.set("ERROR")
                    self.isaac_top_var.set("ERROR")
                    self.log(f"ROS 2 오류: {value}")
                elif event == "zero_save_result":
                    self.on_zero_save_result(value)
                elif event == "close_done":
                    self.finish_close()
        except tk.TclError:
            return

        if self._closing:
            return
        try:
            if self.root.winfo_exists():
                self._process_events_after_id = self.root.after(10, self.process_events)
        except tk.TclError:
            return

    def _can_details(self) -> tuple[bool, str]:
        ip = shutil.which("ip") or "/usr/sbin/ip"
        result = subprocess.run([ip, "-details", "link", "show", CHANNEL], text=True, capture_output=True, check=False)
        text = f"{result.stdout}\n{result.stderr}"
        is_up = result.returncode == 0 and ("state UP" in text or "<NOARP,UP" in text or ",UP," in text)
        configured = "bitrate 1000000" in text and "restart-ms 100" in text
        return is_up and configured, text.strip()

    def _ensure_can(self) -> None:
        okay, _details = self._can_details()
        if okay:
            self.event_queue.put(("can_state", "UP / 1Mbps"))
            return
        ip = shutil.which("ip") or "/usr/sbin/ip"
        command = (
            f"{ip} link set {CHANNEL} down 2>/dev/null || true; "
            f"{ip} link set {CHANNEL} type can bitrate 1000000 restart-ms 100; "
            f"{ip} link set {CHANNEL} up"
        )
        result = subprocess.run(["pkexec", "/bin/sh", "-c", command], text=True, capture_output=True, check=False)
        if result.returncode != 0:
            raise RuntimeError(result.stderr.strip() or "can0 설정 권한 승인 실패")
        okay, details = self._can_details()
        if not okay:
            raise RuntimeError(f"can0 설정 확인 실패: {details}")
        self.event_queue.put(("can_state", "UP / 1Mbps"))

    def refresh_can_async(self) -> None:
        self._can_after_id = None
        if self._closing:
            return

        def check() -> None:
            try:
                okay, _details = self._can_details()
                state = "UP / 1Mbps" if okay else "DOWN / 설정 필요"
            except Exception:
                state = "UNKNOWN"
            self.event_queue.put(("can_state", state))

        threading.Thread(target=check, daemon=True).start()
        self._can_after_id = self.root.after(2000, self.refresh_can_async)

    def _start_controller_process(self) -> None:
        controller_path = CONTROLLER_PATH.resolve()
        if not controller_path.is_file():
            raise FileNotFoundError(f"Controller executable missing:\n{controller_path}")
        if not BASE_DIR.is_dir():
            raise FileNotFoundError(f"Controller working directory missing:\n{BASE_DIR}")
        if not AK70_CALIBRATION_PATH.is_file():
            raise FileNotFoundError(f"AK70 calibration missing:\n{AK70_CALIBRATION_PATH}")
        self.process_owner_token = uuid.uuid4().hex
        self.controller_output_tail.clear()
        command = [
            sys.executable,
            "-u",
            str(controller_path),
            "--channel", CHANNEL,
            "--motor-ids", ",".join(motor_key(i) for i in range(1, 13)),
            "--rate-hz", str(CONTROL_RATE_HZ),
            "--control-mode", "ak70-gui-persistent-lease",
            "--calibration-path", str(AK70_CALIBRATION_PATH),
            "--socket-path", str(IPC_SOCKET_PATH),
            "--lock-path", str(IPC_LOCK_PATH),
            "--process-owner-token", self.process_owner_token,
        ]
        try:
            self.controller_process = subprocess.Popen(
                command,
                cwd=str(BASE_DIR),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except FileNotFoundError as exc:
            missing = exc.filename or str(controller_path)
            raise FileNotFoundError(f"Controller start path missing:\n{missing}") from exc
        self.controller_owned = True
        self.controller_reader_thread = threading.Thread(
            target=self._read_controller_output,
            args=(self.controller_process,),
            daemon=True,
        )
        self.controller_reader_thread.start()

    def _read_controller_output(self, process: subprocess.Popen[str]) -> None:
        if process.stdout is not None:
            for line in process.stdout:
                text = line.rstrip()
                self.controller_output_tail.append(text)
                try:
                    with CONTROLLER_LOG_PATH.open("a", encoding="utf-8") as log_handle:
                        log_handle.write(text + "\n")
                except OSError as exc:
                    self.event_queue.put(("controller_log", f"Controller log write failed: {CONTROLLER_LOG_PATH}: {exc}"))
                self.event_queue.put(("controller_log", f"controller: {text}"))
        code = process.wait()
        self.event_queue.put(("controller_log", f"controller 종료 code={code}"))

    def _active_session_is_reusable(self) -> bool:
        status = self.status_cache
        return (
            bool(self.session_token)
            and status.get("running") is True
            and status.get("active_session") is True
            and status.get("session_token") == self.session_token
            and status.get("session_owner") == "ak70_control_center_gui"
            and bool(self.ready_ids)
        )

    def _other_control_owner_active(self) -> bool:
        status = self.status_cache
        return status.get("active_session") is True and (
            not self.session_token
            or status.get("session_token") != self.session_token
            or status.get("session_owner") != "ak70_control_center_gui"
        )

    def start_all_motors(self, after_ready: Callable[[], None] | None = None) -> None:
        if after_ready is not None:
            self.pending_after_start.append(after_ready)
        if self.start_busy:
            return
        if self._active_session_is_reusable():
            self.log(f"기존 control session 재사용 token={token_fingerprint(self.session_token)}")
            self.apply_status(self.status_cache)
            self._apply_saved_gains_to_ready()
            callbacks, self.pending_after_start = self.pending_after_start, []
            for callback in callbacks:
                callback()
            return
        if self._other_control_owner_active():
            self.log("다른 control owner가 사용 중")
            self.pending_after_start.clear()
            return
        self.start_busy = True
        self.controller_var.set("STARTING")
        for motor_id in ID_TO_NAME:
            self.state_vars[motor_id].set("STARTING")
        threading.Thread(target=self._start_sequence, name="one-click-start", daemon=True).start()

    def _store_arm_session(
        self,
        response: dict[str, Any],
        requested_token: str,
        previous_token: str | None,
    ) -> tuple[str, int]:
        if response.get("ok") is not True:
            raise RuntimeError(str(response.get("error", "ARM 실패")))
        token = response.get("session_token")
        epoch = response.get("session_epoch")
        if not isinstance(token, str) or not token:
            raise RuntimeError("ARM ACK session_token missing")
        if epoch is None:
            raise RuntimeError("ARM ACK session_epoch missing")
        if previous_token and token == previous_token:
            raise RuntimeError("ARM ACK returned previous session_token")
        if token != requested_token:
            raise RuntimeError("ARM ACK session_token mismatch")
        self.session_token = token
        self.session_epoch = int(epoch)
        self.heartbeat_seq = 0
        self._one_click_arm_complete = True
        self._heartbeat_enabled = False
        self._first_heartbeat_logged = False
        self._session_lost_logged = False
        self.event_queue.put(("controller_log", f"ARM success token={token_fingerprint(token)}"))
        return token, int(epoch)

    @staticmethod
    def _validate_one_click_status(
        status: dict[str, Any],
        session_token: str,
        ready_ids: list[str],
    ) -> None:
        if status.get("running") is not True:
            raise RuntimeError("controller STATUS is not RUNNING")
        if status.get("active_session") is not True:
            raise RuntimeError("controller STATUS has no active session")
        if status.get("session_token") != session_token:
            raise RuntimeError("controller active session token mismatch")
        active_targets = set(status.get("active_target_ids", []))
        if not ready_ids or not set(ready_ids).issubset(active_targets):
            raise RuntimeError("READY motor active HOLD target missing")

    def _start_sequence(self) -> None:
        armed_token: str | None = None
        try:
            self._ensure_can()
            if not probe_datagram_socket(IPC_SOCKET_PATH, timeout_sec=0.15):
                self._start_controller_process()
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline and not probe_datagram_socket(IPC_SOCKET_PATH, timeout_sec=0.15):
                    if self.controller_process is not None and self.controller_process.poll() is not None:
                        if self.controller_reader_thread is not None:
                            self.controller_reader_thread.join(timeout=0.2)
                        tail = "\n".join(self.controller_output_tail) or "controller output 없음"
                        raise RuntimeError(f"Controller exited before IPC socket ready:\n{tail}")
                    time.sleep(0.08)
                if not probe_datagram_socket(IPC_SOCKET_PATH, timeout_sec=0.15):
                    tail = "\n".join(self.controller_output_tail) or f"socket missing: {IPC_SOCKET_PATH}"
                    raise RuntimeError(f"Controller IPC socket timeout (3s):\n{tail}")

            previous_token = self.session_token
            requested_token = uuid.uuid4().hex
            arm = send_request(
                {"command": "ARM", "owner": "ak70_control_center_gui", "session_token": requested_token},
                IPC_SOCKET_PATH,
                timeout_sec=IPC_TIMEOUT_BY_COMMAND["ARM"],
            )
            token, epoch = self._store_arm_session(arm, requested_token, previous_token)
            armed_token = token
            started = send_request(
                {"command": "START_MOTORS", "session_token": self.session_token, "session_epoch": self.session_epoch},
                IPC_SOCKET_PATH,
                timeout_sec=IPC_TIMEOUT_BY_COMMAND["START_MOTORS"],
            )
            if not started.get("ok"):
                raise RuntimeError(str(started.get("error", "motor start 실패")))
            self.event_queue.put(("controller_log", f"SET_TARGETS token={token_fingerprint(self.session_token)}"))

            first_heartbeat_seq = 1
            heartbeat = send_request(
                {
                    "command": "HEARTBEAT",
                    "session_token": self.session_token,
                    "heartbeat_seq": first_heartbeat_seq,
                    "generated_monotonic": time.monotonic(),
                },
                IPC_SOCKET_PATH,
                timeout_sec=IPC_TIMEOUT_BY_COMMAND["HEARTBEAT"],
            )
            if heartbeat.get("ok") is not True or heartbeat.get("heartbeat_accepted") is not True:
                raise RuntimeError(str(heartbeat.get("error") or heartbeat.get("reason") or "first HEARTBEAT failed"))
            self.event_queue.put(("controller_log", f"HEARTBEAT token={token_fingerprint(self.session_token)}"))

            status_response = send_request(
                {"command": "STATUS"},
                IPC_SOCKET_PATH,
                timeout_sec=IPC_TIMEOUT_BY_COMMAND["STATUS"],
            )
            if status_response.get("ok") is not True or not isinstance(status_response.get("status"), dict):
                raise RuntimeError(str(status_response.get("error", "STATUS verification failed")))
            status = status_response["status"]
            ready_ids = list(started.get("detected_ids", []))
            self._validate_one_click_status(status, token, ready_ids)
            self.event_queue.put(("controller_log", f"controller active token={token_fingerprint(status.get('session_token'))}"))
            self.event_queue.put(("start_result", {
                "ok": True,
                "token": token,
                "epoch": epoch,
                "heartbeat_seq": first_heartbeat_seq,
                "response": started,
                "status": status,
            }))
        except Exception as exc:
            if armed_token and self.session_token == armed_token:
                self.session_token = None
                self.session_epoch = None
                self._one_click_arm_complete = False
                self._heartbeat_enabled = False
            detail = str(exc)
            if isinstance(exc, FileNotFoundError) and getattr(exc, "filename", None):
                detail = f"{detail}\nMissing path: {exc.filename}"
            self.event_queue.put(("start_result", {"ok": False, "error": detail}))

    def on_start_result(self, result: dict[str, Any]) -> None:
        self.start_busy = False
        if not result.get("ok"):
            self.controller_var.set("START FAILED")
            for motor_id in ID_TO_NAME:
                if motor_id not in self.ready_ids:
                    self.state_vars[motor_id].set("NOT FOUND")
            self.log(f"원클릭 시작 실패: {result.get('error')}")
            messagebox.showerror("전체 모터 시작 실패", str(result.get("error")))
            self.pending_after_start.clear()
            return
        if self.session_token != result["token"] or self.session_epoch != result["epoch"]:
            self.controller_var.set("START FAILED")
            self.log("원클릭 시작 실패: persistent ARM session mismatch")
            self.pending_after_start.clear()
            return
        self.heartbeat_seq = int(result["heartbeat_seq"])
        self._heartbeat_enabled = True
        self._first_heartbeat_logged = True
        response = result["response"]
        self.apply_status(result["status"])
        self._apply_saved_gains_to_ready()
        detected = response.get("detected_ids", [])
        missing = response.get("missing_ids", [])
        self.log(f"원클릭 시작 완료: READY={detected}, NOT FOUND={missing}")
        callbacks, self.pending_after_start = self.pending_after_start, []
        for callback in callbacks:
            callback()

    def apply_status(self, status: dict[str, Any]) -> None:
        self.status_cache = status
        if self._session_lost_logged:
            self.controller_var.set("SESSION LOST")
        else:
            self.controller_var.set(str(status.get("mode", "RUNNING")))
        motors = status.get("motors", {})
        ready: set[int] = set()
        self.suppress_scale = True
        self._updating_slider_programmatically = True
        try:
            for motor_id in ID_TO_NAME:
                info = motors.get(motor_key(motor_id), {})
                actual = info.get("actual_deg")
                actual_value = None if actual is None else float(actual)
                self.actual[motor_id] = actual_value
                raw_position = info.get("raw_position_rad")
                self.raw_positions[motor_id] = None if raw_position is None else float(raw_position)
                direction_sign = int(info.get("direction_sign", self.direction_signs[motor_id]))
                if direction_sign in (-1, 1):
                    self.direction_signs[motor_id] = direction_sign
                low = float(info.get("joint_limit_min_deg", -120.0))
                high = float(info.get("joint_limit_max_deg", 120.0))
                self.limits[motor_id] = (low, high)
                scale = getattr(self, f"scale_{motor_id}")
                scale.configure(from_=low, to=high)
                owned = bool(info.get("owned"))
                feedback_valid = bool(info.get("feedback_valid"))
                if owned and feedback_valid:
                    ready.add(motor_id)
                target = info.get("target_deg")
                if target is not None and owned:
                    self.targets[motor_id] = float(target)
                    self.scale_vars[motor_id].set(self.targets[motor_id])
                    self._last_slider_values[motor_id] = self.targets[motor_id]
                if motor_id in ready:
                    self.actual_vars[motor_id].set(self._format_compact_display("A", motor_id, actual_value))
                    self.target_vars[motor_id].set(self._format_compact_display("T", motor_id, self.targets[motor_id]))
                    scale.configure(state="normal")
                else:
                    self.actual_vars[motor_id].set("A:N/A")
                    self.target_vars[motor_id].set("T:N/A")
                    scale.configure(state="disabled")
                bar_low, bar_high = self._bar_display_range(motor_id, low, high)
                self.markers[motor_id].update_value(self._bar_display_deg(motor_id, actual_value), bar_low, bar_high)
                if motor_id in ready:
                    state = str(info.get("state", "READY"))
                    self.state_vars[motor_id].set("READY" if state in {"MOVING", "HOLDING", "HOME_MOVING", "HOME_HOLDING"} else state)
                elif feedback_valid:
                    self.state_vars[motor_id].set(str(info.get("state", "DETECTED")))
                else:
                    self.state_vars[motor_id].set("NOT FOUND" if self.session_token else "NOT STARTED")
        finally:
            self.suppress_scale = False
            self._updating_slider_programmatically = False
        self.ready_ids = ready
        self.detected_var.set(f"{len(ready)} / 12")
        controller_fault = bool(status.get("bus_fault")) or str(status.get("mode")) in {"FAULT", "ESTOPPED"}
        if self.walk_running:
            if controller_fault:
                self.stop_walk()
                self.walk_state_var.set("WALK STOPPED: controller fault")
            else:
                lost = self.walk_active_motor_ids - ready
                if lost:
                    self.walk_active_motor_ids = retain_ready_ids(self.walk_active_motor_ids, ready)
                    self.dirty_targets = {
                        motor_id: value
                        for motor_id, value in self.dirty_targets.items()
                        if motor_id in self.walk_active_motor_ids
                    }
                    self.log(f"PARTIAL WALK motor removed: {', '.join(map(str, sorted(lost)))}")
                if not self.walk_active_motor_ids:
                    self.walk_running = False
                    self.walk_stage = "STOPPED"
                    self.mode_var.set("MANUAL")
                    self.walk_state_var.set("NO READY MOTORS")
                else:
                    state_label = "WALK TRANSITION" if self.walk_stage == "TRANSITION" else "WALK RUNNING"
                    self._set_walk_state(state_label)
        if self.isaac_following:
            self.dirty_targets = {
                motor_id: value for motor_id, value in self.dirty_targets.items() if motor_id in ready
            }
            if not ready:
                self.isaac_following = False
                self.mode_var.set("MANUAL")
                self.ros_connection_var.set("NO READY MOTORS")
                self.isaac_mode_detail_var.set("Isaac mode: NO READY MOTORS")
        if self._heartbeat_enabled and self.session_token:
            if status.get("active_session") is not True or status.get("session_token") != self.session_token:
                self._handle_session_lost("controller STATUS session mismatch")

    def _controller_running_for_heartbeat(self) -> bool:
        return bool(self.status_cache.get("running")) and str(self.status_cache.get("mode")) == "ARMED"

    def _handle_session_lost(self, error: str) -> None:
        if self._session_lost_logged:
            return
        self._session_lost_logged = True
        lost_token = self.session_token
        self._heartbeat_enabled = False
        self._one_click_arm_complete = False
        self.session_token = None
        self.session_epoch = None
        self.dirty_targets.clear()
        self.stream_inflight = False
        self.stop_walk()
        self.stop_isaac_follow()
        self.controller_var.set("SESSION LOST")
        self.log_ipc_error(
            "HEARTBEAT",
            f"{error} token={token_fingerprint(lost_token)}",
        )

    def _on_heartbeat_response(self, response: dict[str, Any]) -> None:
        if response.get("ok") is True and response.get("heartbeat_accepted") is True:
            if not self._first_heartbeat_logged:
                self.log(f"HEARTBEAT token={token_fingerprint(self.session_token)}")
                self._first_heartbeat_logged = True
            return
        error = str(response.get("error") or response.get("reason") or "heartbeat rejected")
        if "no active control session" in error.lower():
            self._handle_session_lost(error)
            response["_gui_error_handled"] = True
            response["_session_lost"] = True
        else:
            self.log_ipc_error("HEARTBEAT", error)

    def heartbeat_tick(self) -> None:
        self._heartbeat_after_id = None
        if self._closing:
            return
        if (
            not self._closing
            and self._controller_running_for_heartbeat()
            and self.session_token
            and self._one_click_arm_complete
            and self._heartbeat_enabled
        ):
            self.heartbeat_seq += 1
            self.enqueue_ipc({
                "command": "HEARTBEAT",
                "session_token": self.session_token,
                "heartbeat_seq": self.heartbeat_seq,
                "generated_monotonic": time.monotonic(),
            }, self._on_heartbeat_response)
        self._heartbeat_after_id = self.root.after(HEARTBEAT_MS, self.heartbeat_tick)

    def status_tick(self) -> None:
        self._status_after_id = None
        if self._closing:
            return
        if self.status_inflight:
            self._status_after_id = self.root.after(STATUS_MS, self.status_tick)
            return
        self.status_inflight = True

        def checked(response: dict[str, Any]) -> None:
            self.status_inflight = False
            if not response.get("ok") and not self.start_busy:
                self.controller_var.set("STOPPED")
                if self.walk_running:
                    self.stop_walk()
                    self.walk_state_var.set("WALK STOPPED: controller unavailable")

        self.enqueue_ipc({"command": "STATUS"}, checked)
        self._status_after_id = self.root.after(STATUS_MS, self.status_tick)

    def manual_takeover(self, _motor_id: int) -> None:
        self.stop_walk(set_manual=False)
        if self.isaac_following:
            self.stop_isaac_follow()
        self.mode_var.set("MANUAL")

    def on_slider(self, motor_id: int, value: str) -> None:
        if self.suppress_scale or self._updating_slider_programmatically:
            return
        target = float(value)
        low, high = self.limits[motor_id]
        target = min(max(target, low), high)
        if abs(target - self._last_slider_values.get(motor_id, target)) < 1e-6:
            return
        self._last_slider_values[motor_id] = target
        self.manual_takeover(motor_id)
        self.targets[motor_id] = target
        self.target_vars[motor_id].set(self._format_compact_display("T", motor_id, target) if motor_id in self.ready_ids else "T:N/A")
        if motor_id in self.ready_ids:
            self.dirty_targets[motor_id] = target

    def _gain_for_motor(self, motor_id: int) -> tuple[float, float]:
        model = ID_TO_MODEL[motor_id]
        gains = self.model_gains[model]
        return float(gains["kp"]), float(gains["kd"])

    def _refresh_gain_label(self, model: str, gain_name: str) -> None:
        variable = self.gain_value_vars.get((model, gain_name))
        if variable is not None:
            variable.set(f"{self.model_gains[model][gain_name]:.1f}")

    def adjust_model_gain(self, model: str, gain_name: str, delta: float) -> None:
        low, high = gain_limits(model, gain_name)
        current = float(self.model_gains[model][gain_name])
        updated = round(min(max(current + float(delta), low), high), 4)
        self.model_gains[model][gain_name] = updated
        self._refresh_gain_label(model, gain_name)
        self.log(f"{model} {gain_name.upper()} 설정: {updated:.1f}")
        self._apply_saved_gains_to_ready(model)

    def _apply_saved_gains_to_ready(self, model: str | None = None) -> None:
        if not self.session_token or not self._heartbeat_enabled:
            return
        motor_ids = [
            motor_id
            for motor_id in sorted(self.ready_ids)
            if model is None or ID_TO_MODEL[motor_id] == model
        ]
        if not motor_ids:
            return
        updates = {
            motor_key(motor_id): {
                "kp": self._gain_for_motor(motor_id)[0],
                "kd": self._gain_for_motor(motor_id)[1],
            }
            for motor_id in motor_ids
        }

        def applied(response: dict[str, Any]) -> None:
            if not response.get("ok"):
                self.log_ipc_error("SET_GAINS", str(response.get("error", "gain update failed")))
                return
            for motor_id in motor_ids:
                self.gain_applied_values[motor_id] = self._gain_for_motor(motor_id)

        self.enqueue_ipc({
            "command": "SET_GAINS",
            "session_token": self.session_token,
            "session_epoch": self.session_epoch,
            "updates": updates,
        }, applied)

    def queue_targets(self, targets: dict[int, float]) -> None:
        for motor_id, value in targets.items():
            if motor_id not in self.ready_ids:
                continue
            low, high = self.limits[motor_id]
            target = min(max(float(value), low), high)
            self.targets[motor_id] = target
            self.dirty_targets[motor_id] = target
            self.target_vars[motor_id].set(self._format_compact_display("T", motor_id, target))
            self.suppress_scale = True
            self._updating_slider_programmatically = True
            try:
                self.scale_vars[motor_id].set(target)
                self._last_slider_values[motor_id] = target
            finally:
                self.suppress_scale = False
                self._updating_slider_programmatically = False

    def stream_tick(self) -> None:
        self._stream_after_id = None
        if self._closing:
            return
        if (
            self.dirty_targets
            and not self.stream_inflight
            and self.session_token
            and self._heartbeat_enabled
            and self._controller_running_for_heartbeat()
        ):
            snapshot = dict(self.dirty_targets)
            self.dirty_targets.clear()
            self.stream_inflight = True
            payload = {
                "command": "SET_STREAM_TARGETS",
                "session_token": self.session_token,
                "session_epoch": self.session_epoch,
                "control_group_id": f"{self.mode_var.get().lower()}-stream",
                "targets": build_stream_target_items(
                    snapshot,
                    {motor_id: self._gain_for_motor(motor_id) for motor_id in snapshot},
                ),
            }

            def acknowledged(response: dict[str, Any]) -> None:
                self.stream_inflight = False

            self.enqueue_ipc(payload, acknowledged)
        self._stream_after_id = self.root.after(STREAM_MS, self.stream_tick)

    def align_targets_to_actual(self) -> None:
        self.stop_walk(set_manual=False)
        self.stop_isaac_follow()
        self.mode_var.set("MANUAL")
        self.queue_targets({motor_id: actual for motor_id, actual in self.actual.items() if actual is not None})

    def all_targets_zero(self) -> None:
        self.stop_walk(set_manual=False)
        self.stop_isaac_follow()
        self.mode_var.set("MANUAL")
        self.queue_targets({motor_id: 0.0 for motor_id in self.ready_ids})

    def save_all_software_zero(self) -> None:
        if not self.ready_ids:
            self.log("원점 저장 실패: READY 모터가 없습니다.")
            return
        self.stop_walk(set_manual=False)
        self.stop_isaac_follow()
        self.dirty_targets.clear()
        ready_raw = {
            motor_id: float(self.raw_positions[motor_id])
            for motor_id in self.ready_ids
            if self.raw_positions[motor_id] is not None
        }
        if len(ready_raw) != len(self.ready_ids):
            missing = sorted(self.ready_ids - set(ready_raw))
            self.log(f"Software Zero 저장 실패: raw feedback 없음 ID {missing}")
            return

        def save_worker() -> None:
            try:
                saved = save_software_zero_files(ready_raw, dict(self.direction_signs))
                result = {"ok": True, "saved_ids": saved}
            except Exception as exc:
                result = {"ok": False, "error": str(exc)}
            self.event_queue.put(("zero_save_result", result))

        threading.Thread(target=save_worker, name="software-zero-save", daemon=True).start()

    def on_zero_save_result(self, result: dict[str, Any]) -> None:
        if not result.get("ok"):
            error = str(result.get("error", "unknown error"))
            self.log(f"Software Zero 저장 실패: {error}")
            messagebox.showerror("원점 저장 실패", error)
            return
        saved_ids = [int(motor_id) for motor_id in result.get("saved_ids", [])]
        for motor_id in saved_ids:
            self.actual[motor_id] = 0.0
            self.targets[motor_id] = 0.0
            self.actual_vars[motor_id].set(self._format_compact_display("A", motor_id, 0.0))
            self.target_vars[motor_id].set(self._format_compact_display("T", motor_id, 0.0))
            self.suppress_scale = True
            self._updating_slider_programmatically = True
            try:
                self.scale_vars[motor_id].set(0.0)
                self._last_slider_values[motor_id] = 0.0
            finally:
                self.suppress_scale = False
                self._updating_slider_programmatically = False

        if not self.session_token or self.controller_var.get() == "STOPPED":
            self.log("Software Zero 저장 완료\ncontroller 재시작 시 적용")
            return

        def reloaded(response: dict[str, Any]) -> None:
            if not response.get("ok"):
                self.log(f"Software Zero 저장 완료, calibration reload 실패: {response.get('error')}")
                return
            self.queue_targets({motor_id: 0.0 for motor_id in saved_ids if motor_id in self.ready_ids})
            self.log(f"Software Zero 저장 완료: ID {saved_ids}")

        self.enqueue_ipc({
            "command": "RELOAD_CALIBRATION",
            "session_token": self.session_token,
            "session_epoch": self.session_epoch,
        }, reloaded)

    def start_walk(self, repeat: bool) -> None:
        if self.walk_trajectory is None:
            self.walk_state_var.set(self.walk_load_error or f"보행 파일 없음:\n{WALK_CYCLE_PATH}")
            return
        if self._active_session_is_reusable():
            self._begin_walk(repeat)
            return
        if self._other_control_owner_active():
            self.walk_state_var.set("다른 control owner가 사용 중")
            self.log("다른 control owner가 사용 중")
            return
        self.start_all_motors(lambda: self._begin_walk(repeat))

    def _set_walk_state(self, state: str) -> None:
        active = sorted(self.walk_active_motor_ids)
        missing = sorted(set(range(1, 13)) - self.walk_active_motor_ids)
        if active and missing:
            self.walk_state_var.set(
                f"{state} / PARTIAL WALK\n"
                f"Active motors: {', '.join(map(str, active))}\n"
                f"Missing motors: {', '.join(map(str, missing))}"
            )
        else:
            self.walk_state_var.set(state)

    def _begin_walk(self, repeat: bool) -> None:
        self.walk_active_motor_ids = set(self.ready_ids)
        if not self.walk_active_motor_ids:
            message = "보행 시작 불가\nREADY: 없음"
            self.walk_state_var.set("보행 시작 불가")
            self.log(message)
            return
        self.stop_walk(set_manual=False, log_stop=False)
        self.stop_isaac_follow()
        self.dirty_targets.clear()
        self.mode_var.set("WALK_PRESET")
        self.walk_running = True
        self.walk_repeat = bool(repeat)
        self.walk_stage = "TRANSITION"
        self.walk_transition_started = time.monotonic()
        self.walk_cycle_started = 0.0
        self.walk_transition_from = {
            motor_id: float(self.actual[motor_id] if self.actual[motor_id] is not None else self.targets[motor_id])
            for motor_id in self.walk_active_motor_ids
        }
        self.walk_clamp_logged.clear()
        self.walk_time_var.set("0.000 s")
        self.walk_cycle_var.set("0")
        self._set_walk_state("WALK TRANSITION")
        self.log("보행 반복 시작" if repeat else "보행 1회 실행")

    def _clamp_walk_targets(self, targets: dict[int, float]) -> dict[int, float]:
        clamped: dict[int, float] = {}
        for motor_id, requested in targets.items():
            low, high = self.limits[motor_id]
            applied = min(max(float(requested), low), high)
            clamped[motor_id] = applied
            if applied != requested and motor_id not in self.walk_clamp_logged:
                self.log(f"ID {motor_id} walk target clamped: {requested:.3f}° → {applied:.3f}°")
                self.walk_clamp_logged.add(motor_id)
        return clamped

    def walk_tick(self) -> None:
        self._walk_after_id = None
        if self._closing:
            return
        trajectory = self.walk_trajectory
        if self.walk_running and trajectory is not None:
            self.walk_active_motor_ids = retain_ready_ids(
                self.walk_active_motor_ids, self.ready_ids
            )
            if not self.walk_active_motor_ids:
                self.walk_running = False
                self.walk_stage = "STOPPED"
                self.mode_var.set("MANUAL")
                self.walk_state_var.set("NO READY MOTORS")
                self._walk_after_id = self.root.after(STREAM_MS, self.walk_tick)
                return
            now = time.monotonic()
            if self.walk_stage == "TRANSITION":
                elapsed = max(0.0, now - self.walk_transition_started)
                u = min(elapsed / WALK_TRANSITION_SEC, 1.0)
                smooth = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
                first = trajectory.samples[0].targets_deg
                targets = {
                    motor_id: self.walk_transition_from[motor_id]
                    + (first[motor_id] - self.walk_transition_from[motor_id]) * smooth
                    for motor_id in self.walk_active_motor_ids
                }
                self.walk_time_var.set(f"{min(elapsed, WALK_TRANSITION_SEC):.3f} s")
                if u >= 1.0:
                    self.walk_stage = "CYCLE"
                    self.walk_cycle_started = now
                    self._set_walk_state("WALK RUNNING")
                    self.walk_cycle_var.set("1")
            else:
                elapsed = max(0.0, now - self.walk_cycle_started)
                if not self.walk_repeat and elapsed >= trajectory.cycle_sec:
                    all_walk_targets = interpolate_walk_cycle(trajectory, trajectory.cycle_sec)
                    targets = filter_targets_for_ids(all_walk_targets, self.walk_active_motor_ids)
                    self.queue_targets(self._clamp_walk_targets(targets))
                    self.walk_time_var.set(f"{trajectory.cycle_sec:.3f} s")
                    self.walk_cycle_var.set("1")
                    self.walk_running = False
                    self.walk_stage = "HOLDING"
                    self._set_walk_state("WALK HOLDING")
                    self._walk_after_id = self.root.after(STREAM_MS, self.walk_tick)
                    return
                phase = elapsed % trajectory.cycle_sec if self.walk_repeat else elapsed
                cycle_number = int(elapsed / trajectory.cycle_sec) + 1
                all_walk_targets = interpolate_walk_cycle(trajectory, phase)
                targets = filter_targets_for_ids(all_walk_targets, self.walk_active_motor_ids)
                self.walk_time_var.set(f"{phase:.3f} s")
                self.walk_cycle_var.set(str(cycle_number))
            self.queue_targets(self._clamp_walk_targets(targets))
        self._walk_after_id = self.root.after(STREAM_MS, self.walk_tick)

    def stop_walk(self, set_manual: bool = True, log_stop: bool = True) -> None:
        was_running = self.walk_running
        was_walk_mode = self.mode_var.get() == "WALK_PRESET"
        self.walk_running = False
        if was_running or was_walk_mode:
            self.dirty_targets.clear()
        if self.walk_stage not in {"STOPPED", "HOLDING"}:
            self.walk_stage = "HOLDING"
            if hasattr(self, "walk_state_var"):
                self.walk_state_var.set("WALK HOLDING")
        if set_manual and self.mode_var.get() == "WALK_PRESET":
            self.mode_var.set("MANUAL")
        if was_running and log_stop:
            self.log("보행 정지: 마지막 target HOLD")

    def start_isaac_follow(self) -> None:
        if not self.ros_available:
            self.log("Isaac Sim ROS 2: 사용 불가")
            return
        if self._active_session_is_reusable():
            self._activate_isaac()
            return
        if self._other_control_owner_active():
            self.ros_connection_var.set("다른 control owner가 사용 중")
            self.log("다른 control owner가 사용 중")
            return
        self.start_all_motors(self._activate_isaac)

    def _activate_isaac(self) -> None:
        if not self.ready_ids:
            self.ros_connection_var.set("NO READY MOTORS")
            self.isaac_mode_detail_var.set("Isaac mode: NO READY MOTORS")
            return
        self.stop_walk(set_manual=False)
        self.dirty_targets.clear()
        self.mode_var.set("ISAAC")
        self.isaac_following = True
        self.ros_connection_var.set("STARTING")
        self.isaac_top_var.set("STARTING")
        self._isaac_logged_ready_signature = None
        self.isaac_mode_detail_var.set("Isaac mode: PARTIAL" if len(self.ready_ids) < 12 else "Isaac mode: FULL")
        self.isaac_ready_detail_var.set(
            f"READY motors: {', '.join(map(str, sorted(self.ready_ids)))}"
        )
        self.ros.start()
        self.log("Isaac Sim 추종 시작: 정상 JSON 수신 전까지 현재 자세 HOLD")

    def stop_isaac_follow(self) -> None:
        was_following = self.isaac_following
        self.isaac_following = False
        if was_following:
            self.dirty_targets.clear()
        self.mode_var.set("MANUAL")
        if was_following:
            self.log("Isaac Sim 추종 정지: 마지막 target HOLD")

    def on_ros_message(self, text: str, monotonic_time: float, wall_time: float) -> None:
        try:
            targets, legacy = parse_isaac_payload(text)
        except Exception as exc:
            self.log(f"Isaac JSON 무시: {exc}")
            return
        self.ros_receive_times.append(monotonic_time)
        while self.ros_receive_times and monotonic_time - self.ros_receive_times[0] > 2.0:
            self.ros_receive_times.popleft()
        hz = 0.0
        if len(self.ros_receive_times) >= 2:
            span = self.ros_receive_times[-1] - self.ros_receive_times[0]
            hz = (len(self.ros_receive_times) - 1) / span if span > 0 else 0.0
        self.ros_last_time_var.set(datetime.fromtimestamp(wall_time).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3])
        self.ros_hz_var.set(f"{hz:.1f} Hz")
        self.ros_last_values_var.set(", ".join(f"{ID_TO_NAME[mid]}:{value:+.1f}°" for mid, value in sorted(targets.items())))
        self.ros_connection_var.set("RECEIVING")
        self.isaac_top_var.set("RECEIVING")
        if legacy and not self.legacy_notice_logged:
            self.log("Legacy 6-joint payload: right leg로 적용")
            self.legacy_notice_logged = True
        if self.isaac_following and self.mode_var.get() == "ISAAC":
            applied = filter_targets_for_ids(targets, self.ready_ids)
            ignored_ids = sorted(set(targets) - set(applied))
            applied_names = [ID_TO_NAME[motor_id] for motor_id in sorted(applied)]
            ignored_names = [ID_TO_NAME[motor_id] for motor_id in ignored_ids]
            self.isaac_mode_detail_var.set(
                "Isaac mode: PARTIAL" if len(self.ready_ids) < 12 else "Isaac mode: FULL"
            )
            self.isaac_ready_detail_var.set(
                f"READY motors: {', '.join(map(str, sorted(self.ready_ids))) or '없음'}"
            )
            self.isaac_applied_detail_var.set(
                f"Applied joints: {', '.join(applied_names) or '없음'}"
            )
            self.isaac_ignored_detail_var.set(
                f"Ignored joints: {', '.join(ignored_names) or '없음'}"
            )
            ready_signature = tuple(sorted(self.ready_ids))
            if ready_signature != self._isaac_logged_ready_signature:
                if ignored_names:
                    self.log(
                        "Isaac ignored joints for READY set "
                        f"{list(ready_signature)}: {', '.join(ignored_names)}"
                    )
                self._isaac_logged_ready_signature = ready_signature
            if applied:
                self.queue_targets(applied)

    def release_all(self) -> None:
        self.stop_walk(set_manual=False)
        self.stop_isaac_follow()
        self.dirty_targets.clear()
        token = self.session_token
        if token:
            self.log(f"RELEASE_SESSION source=전체 TORQUE 해제 token={token_fingerprint(token)}")
        self._heartbeat_enabled = False
        self._one_click_arm_complete = False
        self.session_token = None
        self.session_epoch = None
        self.ready_ids.clear()
        self.detected_var.set("0 / 12")
        for motor_id in ID_TO_NAME:
            self.state_vars[motor_id].set("RELEASED")
            self.actual_vars[motor_id].set("A:N/A")
            self.target_vars[motor_id].set("T:N/A")
            try:
                getattr(self, f"scale_{motor_id}").configure(state="disabled")
            except tk.TclError:
                pass
        if not token:
            return

        def complete(response: dict[str, Any]) -> None:
            self.controller_var.set("DISARMED" if response.get("ok") else "RELEASE ERROR")
            self.log(f"전체 TORQUE 해제: ok={response.get('ok')} error={response.get('error')}")

        self.enqueue_ipc({"command": "RELEASE_SESSION", "session_token": token}, complete, timeout=1.5)

    def on_close(self) -> None:
        if self._closing:
            return
        self._closing = True
        self.closing = True
        self.stop_walk(set_manual=False)
        self.ros.stop()
        self.dirty_targets.clear()
        token = self.session_token
        if token:
            close_command = "SHUTDOWN" if self.controller_owned and self.process_owner_token else "RELEASE_SESSION"
            self.log(f"{close_command} source=GUI 종료 token={token_fingerprint(token)}")
        self._heartbeat_enabled = False
        self._one_click_arm_complete = False
        self.session_token = None

        def shutdown() -> None:
            try:
                if token:
                    if self.controller_owned and self.process_owner_token:
                        send_request({
                            "command": "SHUTDOWN",
                            "session_token": token,
                            "process_owner_token": self.process_owner_token,
                        }, IPC_SOCKET_PATH, timeout_sec=1.5)
                    else:
                        send_request(
                            {"command": "RELEASE_SESSION", "session_token": token},
                            IPC_SOCKET_PATH,
                            timeout_sec=1.5,
                        )
            except Exception as exc:
                self.event_queue.put(("controller_log", f"종료 release 오류: {exc}"))
            self.event_queue.put(("close_done", None))

        threading.Thread(target=shutdown, name="async-close", daemon=True).start()
        try:
            self._finish_close_after_id = self.root.after(2200, self.finish_close)
        except tk.TclError:
            self.finish_close()

    def finish_close(self) -> None:
        if self._destroyed:
            return
        after_attributes = (
            "_process_events_after_id",
            "_heartbeat_after_id",
            "_status_after_id",
            "_stream_after_id",
            "_walk_after_id",
            "_can_after_id",
            "motion_monitor_after_id",
            "_finish_close_after_id",
        )
        for attribute in after_attributes:
            after_id = getattr(self, attribute)
            if after_id is not None:
                try:
                    self.root.after_cancel(after_id)
                except tk.TclError:
                    pass
                setattr(self, attribute, None)
        self.ipc_queue.put(None)
        self._destroyed = True
        if self.log_window is not None:
            try:
                self.log_window.destroy()
            except tk.TclError:
                pass
            self.log_window = None
            self.log_text = None
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    ControlCenterApp().run()


if __name__ == "__main__":
    main()
