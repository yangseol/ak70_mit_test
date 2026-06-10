"""AK70 모터 감지, 캘리브레이션, 원점 이동 helper를 실행하는 Tkinter GUI."""

from __future__ import annotations

import datetime as dt
import re
import shutil
import subprocess
import sys
import threading
from pathlib import Path
from tkinter import BooleanVar, StringVar, Tk, messagebox
from tkinter import ttk
import tkinter as tk

import yaml


PROJECT_DIR = Path(__file__).resolve().parent
CALIBRATION_PATH = PROJECT_DIR / "motor_calibration.yaml"
BACKUP_DIR = PROJECT_DIR / "backups"
LOG_DIR = PROJECT_DIR / "logs"
DEFAULT_MOTOR_IDS = "0x001,0x002,0x003,0x004,0x005,0x006,0x007,0x008,0x009,0x00A"

DETECT_RE = re.compile(
    r"ID:\s*(0x[0-9A-Fa-f]+)\s*\|\s*calibrated:\s*(YES|NO)\s*\|\s*"
    r"raw_pos_rad:\s*([+-]?[0-9.]+|N/A)\s*\|\s*"
    r"joint_rad:\s*([+-]?[0-9.]+|N/A)\s*\|\s*"
    r"joint_deg:\s*([+-]?[0-9.]+|N/A)"
)
PLAN_RE = re.compile(r"ID:\s*(0x[0-9A-Fa-f]+)\s*\|.*action:\s*([A-Z_]+)")


def now_text() -> str:
    return dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def timestamp() -> str:
    return dt.datetime.now().strftime("%Y%m%d_%H%M%S")


def normalize_motor_id(motor_id: str) -> str:
    return f"0x{int(motor_id, 16):03X}"


def parse_motor_ids(text: str) -> list[str]:
    ids: list[str] = []
    for part in text.split(","):
        value = part.strip()
        if not value:
            continue
        try:
            if value.lower().startswith("0x"):
                motor_id = int(value, 16)
            else:
                motor_id = int(value, 10)
        except ValueError as exc:
            raise ValueError(f"잘못된 모터 ID: {value}") from exc
        ids.append(f"0x{motor_id:03X}")
    if not ids:
        raise ValueError("모터 ID 목록이 비어 있습니다.")
    return ids


def load_calibration_keys() -> set[str]:
    if not CALIBRATION_PATH.exists():
        return set()
    with CALIBRATION_PATH.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        return set()
    motors = data.get("motors", {})
    if not isinstance(motors, dict):
        return set()
    return {normalize_motor_id(str(key)) for key in motors}


class AK70MotorManagerGUI:
    def __init__(self, root: Tk) -> None:
        self.root = root
        self.root.title("AK70 모터 관리 GUI")
        self.running = False
        self.rows: dict[str, dict[str, str]] = {}
        self.detected_ids: list[str] = []
        self.calibrated_ids: set[str] = set()

        self.channel_var = StringVar(value="can0")
        self.motor_ids_var = StringVar(value=DEFAULT_MOTOR_IDS)
        self.mode_var = StringVar(value="벤치탑")
        self.tolerance_var = StringVar(value="0.05")
        self.max_start_error_var = StringVar(value="0.9")
        self.kp_var = StringVar(value="2.0")
        self.kd_var = StringVar(value="0.1")
        self.pulses_var = StringVar(value="10")
        self.interval_var = StringVar(value="0.02")
        self.enter_mit_var = BooleanVar(value=False)

        self.hardware_buttons: list[ttk.Button] = []
        self.zero_save_button: ttk.Button | None = None

        self.build_ui()
        self.initialize_rows()
        self.refresh_calibration_table(show_log=False)

    def build_ui(self) -> None:
        settings = ttk.LabelFrame(self.root, text="설정")
        settings.pack(fill="x", padx=8, pady=6)

        ttk.Label(settings, text="CAN 채널").grid(row=0, column=0, sticky="w", padx=4, pady=3)
        ttk.Entry(settings, textvariable=self.channel_var, width=12).grid(row=0, column=1, sticky="w", padx=4)

        ttk.Label(settings, text="모터 ID 목록").grid(row=0, column=2, sticky="w", padx=4)
        ttk.Entry(settings, textvariable=self.motor_ids_var, width=56).grid(row=0, column=3, columnspan=5, sticky="ew", padx=4)

        ttk.Label(settings, text="모드 선택").grid(row=1, column=0, sticky="w", padx=4, pady=3)
        mode = ttk.Combobox(settings, textvariable=self.mode_var, values=["벤치탑", "로봇장착"], width=10, state="readonly")
        mode.grid(row=1, column=1, sticky="w", padx=4)
        mode.bind("<<ComboboxSelected>>", self.on_mode_changed)

        labels = ["허용 오차", "최대 시작 오차", "Kp", "Kd", "펄스 수", "간격 sec"]
        vars_ = [
            self.tolerance_var,
            self.max_start_error_var,
            self.kp_var,
            self.kd_var,
            self.pulses_var,
            self.interval_var,
        ]
        for index, (label, var) in enumerate(zip(labels, vars_)):
            col = 2 + index
            ttk.Label(settings, text=label).grid(row=1, column=col, sticky="w", padx=4)
            ttk.Entry(settings, textvariable=var, width=10).grid(row=2, column=col, sticky="w", padx=4, pady=3)

        ttk.Checkbutton(settings, text="MIT 진입 후 감지", variable=self.enter_mit_var).grid(
            row=2, column=0, columnspan=2, sticky="w", padx=4
        )

        settings.columnconfigure(3, weight=1)

        table_frame = ttk.LabelFrame(self.root, text="모터 상태")
        table_frame.pack(fill="both", expand=True, padx=8, pady=6)

        columns = ("selected", "id", "detected", "calibrated", "raw_pos", "joint_rad", "joint_deg", "status")
        self.tree = ttk.Treeview(table_frame, columns=columns, show="headings", height=10)
        headings = {
            "selected": "선택",
            "id": "ID",
            "detected": "감지",
            "calibrated": "캘리브레이션",
            "raw_pos": "Raw Pos",
            "joint_rad": "Joint Rad",
            "joint_deg": "Joint Deg",
            "status": "상태",
        }
        widths = {
            "selected": 50,
            "id": 80,
            "detected": 70,
            "calibrated": 100,
            "raw_pos": 100,
            "joint_rad": 100,
            "joint_deg": 90,
            "status": 160,
        }
        for col in columns:
            self.tree.heading(col, text=headings[col])
            self.tree.column(col, width=widths[col], anchor="center")
        self.tree.pack(side="left", fill="both", expand=True)
        self.tree.bind("<Button-1>", self.on_tree_click)

        scrollbar = ttk.Scrollbar(table_frame, orient="vertical", command=self.tree.yview)
        scrollbar.pack(side="right", fill="y")
        self.tree.configure(yscrollcommand=scrollbar.set)

        buttons = ttk.Frame(self.root)
        buttons.pack(fill="x", padx=8, pady=4)

        self.add_button(buttons, "CAN 확인", self.check_can)
        self.add_button(buttons, "CAN 설정", self.configure_can)
        self.add_button(buttons, "모터 감지", self.detect_motors)
        self.add_button(buttons, "캘리브레이션 새로고침", self.refresh_calibration_table)
        self.zero_save_button = self.add_button(buttons, "원점 저장", self.save_zero)
        self.add_button(buttons, "선택 계획확인", self.plan_selected)
        self.add_button(buttons, "전체 계획확인", self.plan_all)
        self.add_button(buttons, "선택 원점이동", self.nudge_selected)
        self.add_button(buttons, "전체 원점이동", self.nudge_all)
        ttk.Button(buttons, text="로그 지우기", command=self.clear_log).pack(side="left", padx=3, pady=3)
        ttk.Button(buttons, text="로그 저장", command=self.save_log).pack(side="left", padx=3, pady=3)
        ttk.Button(buttons, text="종료", command=self.root.destroy).pack(side="right", padx=3, pady=3)

        log_frame = ttk.LabelFrame(self.root, text="로그")
        log_frame.pack(fill="both", expand=True, padx=8, pady=6)
        self.log_text = tk.Text(log_frame, height=16, wrap="word")
        self.log_text.pack(side="left", fill="both", expand=True)
        log_scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        log_scroll.pack(side="right", fill="y")
        self.log_text.configure(yscrollcommand=log_scroll.set)

    def add_button(self, parent: ttk.Frame, text: str, command) -> ttk.Button:
        button = ttk.Button(parent, text=text, command=command)
        button.pack(side="left", padx=3, pady=3)
        self.hardware_buttons.append(button)
        return button

    def initialize_rows(self) -> None:
        for item in self.tree.get_children():
            self.tree.delete(item)
        self.rows.clear()
        for motor_id in [f"0x{i:03X}" for i in range(1, 11)]:
            row = {
                "selected": "☐",
                "id": motor_id,
                "detected": "NO",
                "calibrated": "NO",
                "raw_pos": "N/A",
                "joint_rad": "N/A",
                "joint_deg": "N/A",
                "status": "MISSING",
            }
            self.rows[motor_id] = row
            self.tree.insert("", "end", iid=motor_id, values=self.row_values(row))

    def row_values(self, row: dict[str, str]) -> tuple[str, ...]:
        return (
            row["selected"],
            row["id"],
            row["detected"],
            row["calibrated"],
            row["raw_pos"],
            row["joint_rad"],
            row["joint_deg"],
            row["status"],
        )

    def update_row(self, motor_id: str, **updates: str) -> None:
        if motor_id not in self.rows:
            return
        self.rows[motor_id].update(updates)
        self.tree.item(motor_id, values=self.row_values(self.rows[motor_id]))

    def on_tree_click(self, event) -> None:
        region = self.tree.identify("region", event.x, event.y)
        column = self.tree.identify_column(event.x)
        item = self.tree.identify_row(event.y)
        if region != "cell" or column != "#1" or not item:
            return
        current = self.rows[item]["selected"]
        self.update_row(item, selected="☑" if current == "☐" else "☐")

    def on_mode_changed(self, _event=None) -> None:
        if self.mode_var.get() == "벤치탑":
            self.max_start_error_var.set("0.9")
            self.kp_var.set("2.0")
            self.pulses_var.set("10")
        else:
            self.max_start_error_var.set("0.3")
            self.kp_var.set("1.0")
            self.pulses_var.set("5")

    def log(self, message: str) -> None:
        self.log_text.insert("end", message + "\n")
        self.log_text.see("end")

    def clear_log(self) -> None:
        self.log_text.delete("1.0", "end")

    def save_log(self) -> None:
        LOG_DIR.mkdir(exist_ok=True)
        path = LOG_DIR / f"ak70_gui_{timestamp()}.txt"
        path.write_text(self.log_text.get("1.0", "end"), encoding="utf-8")
        self.log(f"로그 저장: {path}")

    def set_running(self, running: bool) -> None:
        self.running = running
        state = "disabled" if running else "normal"
        for button in self.hardware_buttons:
            button.configure(state=state)
        self.update_zero_save_button()

    def update_zero_save_button(self) -> None:
        if self.zero_save_button is None:
            return
        if self.running or len(self.detected_ids) != 1:
            self.zero_save_button.configure(state="disabled")
        else:
            self.zero_save_button.configure(state="normal")

    def validate_common_inputs(self) -> dict | None:
        channel = self.channel_var.get().strip()
        if not channel:
            messagebox.showerror("입력 오류", "CAN 채널이 비어 있습니다.")
            return None
        try:
            motor_ids = parse_motor_ids(self.motor_ids_var.get())
            tolerance_rad = float(self.tolerance_var.get())
            max_start_error_rad = float(self.max_start_error_var.get())
            kp = float(self.kp_var.get())
            kd = float(self.kd_var.get())
            pulses = int(self.pulses_var.get())
            interval_sec = float(self.interval_var.get())
        except ValueError as exc:
            messagebox.showerror("입력 오류", str(exc))
            return None

        errors = []
        if not 0.0 < kp <= 5.0:
            errors.append("Kp는 0.0보다 크고 5.0 이하여야 합니다.")
        if not 0.0 <= kd <= 2.0:
            errors.append("Kd는 0.0 이상 2.0 이하여야 합니다.")
        if not 1 <= pulses <= 20:
            errors.append("펄스 수는 1 이상 20 이하여야 합니다.")
        if interval_sec < 0.005:
            errors.append("간격 sec는 0.005 이상이어야 합니다.")
        if tolerance_rad <= 0.0:
            errors.append("허용 오차는 0.0보다 커야 합니다.")
        if max_start_error_rad <= tolerance_rad:
            errors.append("최대 시작 오차는 허용 오차보다 커야 합니다.")
        if errors:
            messagebox.showerror("입력 오류", "\n".join(errors))
            return None

        return {
            "channel": channel,
            "motor_ids": motor_ids,
            "motor_ids_text": ",".join(motor_ids),
            "tolerance_rad": tolerance_rad,
            "max_start_error_rad": max_start_error_rad,
            "kp": kp,
            "kd": kd,
            "pulses": pulses,
            "interval_sec": interval_sec,
        }

    def selected_ids(self) -> list[str]:
        return [motor_id for motor_id, row in self.rows.items() if row["selected"] == "☑"]

    def calibrated_detected_missing(self, ids: list[str]) -> list[str]:
        return [motor_id for motor_id in ids if self.rows.get(motor_id, {}).get("calibrated") != "YES"]

    def run_subprocess(
        self,
        title: str,
        cmd: list[str],
        stdin_text: str | None = None,
        on_done=None,
    ) -> None:
        if self.running:
            messagebox.showwarning("실행 중", "다른 작업이 실행 중입니다.")
            return
        self.set_running(True)
        self.log(f"[{now_text()}] 시작: {title}")
        self.log("$ " + " ".join(cmd))

        def worker() -> None:
            try:
                proc = subprocess.run(
                    cmd,
                    input=stdin_text,
                    text=True,
                    cwd=PROJECT_DIR,
                    capture_output=True,
                    check=False,
                )
                stdout = proc.stdout or ""
                stderr = proc.stderr or ""
                returncode = proc.returncode
            except Exception as exc:
                stdout = ""
                stderr = str(exc)
                returncode = -1

            def finish() -> None:
                if stdout:
                    self.log(stdout.rstrip())
                if stderr:
                    self.log("[stderr]")
                    self.log(stderr.rstrip())
                self.log(f"return code: {returncode}")
                self.log(f"[{now_text()}] 종료: {title}")
                if on_done is not None:
                    try:
                        on_done(stdout, stderr, returncode)
                    except Exception as exc:
                        self.log(f"파싱 경고: {exc}")
                self.set_running(False)

            self.root.after(0, finish)

        threading.Thread(target=worker, daemon=True).start()

    def check_can(self) -> None:
        channel = self.channel_var.get().strip()
        if not channel:
            messagebox.showerror("입력 오류", "CAN 채널이 비어 있습니다.")
            return
        self.run_subprocess("CAN 확인", ["ip", "-details", "link", "show", channel])

    def configure_can(self) -> None:
        channel = self.channel_var.get().strip()
        if not channel:
            messagebox.showerror("입력 오류", "CAN 채널이 비어 있습니다.")
            return
        if not messagebox.askyesno("CAN 설정", "CAN 인터페이스를 1Mbps로 설정할까요?"):
            return
        script = (
            f"ip link set {channel} down && "
            f"ip link set {channel} type can bitrate 1000000 && "
            f"ip link set {channel} up"
        )

        def done(_stdout: str, stderr: str, returncode: int) -> None:
            if returncode != 0:
                self.log("pkexec 실행 실패 또는 취소. 아래 명령을 직접 실행하세요:")
                self.log(f"sudo ip link set {channel} down")
                self.log(f"sudo ip link set {channel} type can bitrate 1000000")
                self.log(f"sudo ip link set {channel} up")

        self.run_subprocess("CAN 설정", ["pkexec", "sh", "-c", script], on_done=done)

    def detect_motors(self) -> None:
        values = self.validate_common_inputs()
        if values is None:
            return
        cmd = [
            sys.executable,
            "detect_multi_motors_once.py",
            "--channel",
            values["channel"],
            "--motor-ids",
            values["motor_ids_text"],
            "--yes",
        ]
        if self.enter_mit_var.get():
            cmd.append("--enter-mit")
        self.run_subprocess("모터 감지", cmd, on_done=lambda out, _err, _code: self.parse_detect_output(out, values["motor_ids"]))

    def parse_detect_output(self, output: str, scanned_ids: list[str]) -> None:
        detected: set[str] = set()
        parsed_any = False
        for match in DETECT_RE.finditer(output):
            parsed_any = True
            motor_id = normalize_motor_id(match.group(1))
            calibrated = match.group(2)
            raw_pos = match.group(3)
            joint_rad = match.group(4)
            joint_deg = match.group(5)
            detected.add(motor_id)
            if motor_id in self.rows:
                self.update_row(
                    motor_id,
                    detected="YES",
                    calibrated=calibrated,
                    raw_pos=raw_pos,
                    joint_rad=joint_rad,
                    joint_deg=joint_deg,
                    status="DETECTED",
                )

        for motor_id in scanned_ids:
            if motor_id in self.rows and motor_id not in detected:
                calibrated = "YES" if motor_id in self.calibrated_ids else "NO"
                self.update_row(
                    motor_id,
                    detected="NO",
                    calibrated=calibrated,
                    raw_pos="N/A",
                    joint_rad="N/A",
                    joint_deg="N/A",
                    status="MISSING",
                )

        if not parsed_any:
            self.log("파싱 경고: 모터 감지 결과를 파싱하지 못했습니다.")
        self.detected_ids = sorted(detected, key=lambda x: int(x, 16))
        self.update_zero_save_button()

    def refresh_calibration_table(self, show_log: bool = True) -> None:
        try:
            self.calibrated_ids = load_calibration_keys()
        except Exception as exc:
            messagebox.showerror("캘리브레이션 오류", str(exc))
            return
        for motor_id, row in self.rows.items():
            self.update_row(motor_id, calibrated="YES" if motor_id in self.calibrated_ids else "NO")
        self.update_zero_save_button()
        if show_log:
            self.log("캘리브레이션 새로고침 완료")

    def save_zero(self) -> None:
        values = self.validate_common_inputs()
        if values is None:
            return
        if len(self.detected_ids) != 1:
            messagebox.showerror("실행 거부", "원점 저장은 감지된 모터가 정확히 1개일 때만 가능합니다.")
            return
        motor_id = self.detected_ids[0]
        if not messagebox.askyesno("원점 저장", "현재 감지된 1개 모터의 현재 위치를 원점으로 저장합니다. 계속할까요?"):
            return
        if not messagebox.askyesno("원점 저장", "motor_calibration.yaml 파일이 수정됩니다. 정말 저장할까요?"):
            return
        try:
            BACKUP_DIR.mkdir(exist_ok=True)
            if CALIBRATION_PATH.exists():
                backup = BACKUP_DIR / f"motor_calibration_{timestamp()}.yaml"
                shutil.copy2(CALIBRATION_PATH, backup)
                self.log(f"백업 생성: {backup}")
            else:
                self.log("백업 경고: motor_calibration.yaml 파일이 없습니다.")
        except OSError as exc:
            messagebox.showerror("백업 실패", str(exc))
            return

        cmd = [
            sys.executable,
            "capture_motor_zero_once.py",
            "--channel",
            values["channel"],
            "--motor-id",
            motor_id,
            "--name",
            f"motor_{int(motor_id, 16):03X}",
            "--notes",
            "Software zero captured by ak70_motor_manager_gui.py",
        ]

        def done(_out: str, _err: str, _code: int) -> None:
            self.refresh_calibration_table(show_log=True)

        self.run_subprocess("원점 저장", cmd, stdin_text="YES\nSAVE\n", on_done=done)

    def plan_selected(self) -> None:
        values = self.validate_common_inputs()
        if values is None:
            return
        selected = self.selected_ids()
        if not selected:
            messagebox.showerror("입력 오류", "선택된 모터가 없습니다.")
            return
        self.run_plan("선택 계획확인", selected, values)

    def plan_all(self) -> None:
        values = self.validate_common_inputs()
        if values is None:
            return
        if not self.detected_ids:
            messagebox.showerror("실행 거부", "감지된 모터가 없습니다.")
            return
        missing_cal = self.calibrated_detected_missing(self.detected_ids)
        if missing_cal:
            messagebox.showwarning("캘리브레이션 경고", "캘리브레이션 없는 감지 모터가 있습니다: " + ", ".join(missing_cal))
        self.run_plan("전체 계획확인", self.detected_ids, values)

    def run_plan(self, title: str, motor_ids: list[str], values: dict) -> None:
        cmd = [
            sys.executable,
            "plan_multi_zero_move.py",
            "--channel",
            values["channel"],
            "--motor-ids",
            ",".join(motor_ids),
            "--tolerance-rad",
            str(values["tolerance_rad"]),
            "--max-start-error-rad",
            str(values["max_start_error_rad"]),
        ]
        self.run_subprocess(title, cmd, stdin_text="YES\n", on_done=lambda out, _err, _code: self.parse_plan_output(out))

    def parse_plan_output(self, output: str) -> None:
        parsed_any = False
        for match in PLAN_RE.finditer(output):
            parsed_any = True
            motor_id = normalize_motor_id(match.group(1))
            action = match.group(2)
            if motor_id in self.rows:
                self.update_row(motor_id, status=action)
        if not parsed_any:
            self.log("파싱 경고: 계획확인 결과를 파싱하지 못했습니다.")

    def nudge_selected(self) -> None:
        values = self.validate_common_inputs()
        if values is None:
            return
        selected = self.selected_ids()
        if not selected:
            messagebox.showerror("입력 오류", "선택된 모터가 없습니다.")
            return
        if not messagebox.askyesno("선택 원점이동", "선택한 모터를 순차적으로 원점 이동합니다. 계속할까요?"):
            return
        self.run_nudge("선택 원점이동", selected, values)

    def nudge_all(self) -> None:
        values = self.validate_common_inputs()
        if values is None:
            return
        if not self.detected_ids:
            messagebox.showerror("실행 거부", "감지된 모터가 없습니다.")
            return
        missing_cal = self.calibrated_detected_missing(self.detected_ids)
        if missing_cal:
            messagebox.showerror("실행 거부", "캘리브레이션 없는 감지 모터가 있습니다: " + ", ".join(missing_cal))
            return
        if not messagebox.askyesno("전체 원점이동", "감지된 모든 모터를 순차적으로 원점 이동합니다. 계속할까요?"):
            return
        self.run_nudge("전체 원점이동", self.detected_ids, values)

    def run_nudge(self, title: str, motor_ids: list[str], values: dict) -> None:
        cmd = [
            sys.executable,
            "nudge_multi_joints_once.py",
            "--channel",
            values["channel"],
            "--motor-ids",
            ",".join(motor_ids),
            "--tolerance-rad",
            str(values["tolerance_rad"]),
            "--max-start-error-rad",
            str(values["max_start_error_rad"]),
            "--kp",
            str(values["kp"]),
            "--kd",
            str(values["kd"]),
            "--pulses",
            str(values["pulses"]),
            "--interval-sec",
            str(values["interval_sec"]),
        ]
        self.run_subprocess(
            title,
            cmd,
            stdin_text="YES\nNUDGE\n",
            on_done=lambda out, _err, _code: self.parse_plan_output(out),
        )


def main() -> None:
    root = Tk()
    root.geometry("1180x760")
    AK70MotorManagerGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
