"""Read-only 2D motion monitor helpers for the lower-body GUI.

This module intentionally contains no CAN, IPC, calibration-write, or control
logic.  It consumes already-converted logical joint degrees from the GUI.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping


Point = tuple[float, float]
UI_DISPLAY_SIGN_BY_MOTOR = {
    1: -1.0,
}
FRONT_KINEMATIC_SIGN_BY_MOTOR = {
    2: -1.0,
}
LEFT_SIDE_KINEMATIC_SIGN_BY_MOTOR = {
    11: -1.0,
}
BAR_DISPLAY_SIGN_BY_MOTOR = {
    10: -1.0,
}
ACTUAL_COLOR = "#1565c0"
TARGET_COLOR = "#90a4ae"
NEUTRAL_COLOR = "#b0b0b0"
STRUCTURE_COLOR = "#546e7a"


@dataclass(frozen=True)
class LegPoints:
    hip: Point
    knee: Point
    ankle: Point
    toe: Point


@dataclass(frozen=True)
class JointDisplayRow:
    motor_id: int
    joint: str
    model: str
    target: float | None
    actual: float | None
    error: float | None
    state: str


def joint_error(actual_deg: float | None, target_deg: float | None) -> float | None:
    if actual_deg is None or target_deg is None:
        return None
    return float(actual_deg) - float(target_deg)


def to_ui_display_deg(motor_id: int, logical_deg: float | None) -> float | None:
    if logical_deg is None:
        return None
    sign = UI_DISPLAY_SIGN_BY_MOTOR.get(int(motor_id), 1.0)
    return float(logical_deg) * sign


def to_ui_display_range(motor_id: int, low_deg: float, high_deg: float) -> tuple[float, float]:
    low = to_ui_display_deg(motor_id, low_deg)
    high = to_ui_display_deg(motor_id, high_deg)
    if low is None or high is None:
        return float(low_deg), float(high_deg)
    return min(low, high), max(low, high)


def to_bar_display_deg(motor_id: int, display_deg: float | None) -> float | None:
    if display_deg is None:
        return None
    sign = BAR_DISPLAY_SIGN_BY_MOTOR.get(int(motor_id), 1.0)
    return float(display_deg) * sign


def to_bar_display_range(motor_id: int, display_low_deg: float, display_high_deg: float) -> tuple[float, float]:
    low = to_bar_display_deg(motor_id, display_low_deg)
    high = to_bar_display_deg(motor_id, display_high_deg)
    if low is None or high is None:
        return float(display_low_deg), float(display_high_deg)
    return min(low, high), max(low, high)


def to_side_kinematic_deg(_motor_id: int, display_deg: float | None) -> float | None:
    if display_deg is None:
        return None
    return float(display_deg)


def to_front_kinematic_deg(motor_id: int, display_deg: float | None) -> float | None:
    if display_deg is None:
        return None
    sign = FRONT_KINEMATIC_SIGN_BY_MOTOR.get(int(motor_id), 1.0)
    return float(display_deg) * sign


def to_left_side_kinematic_deg(motor_id: int, display_deg: float | None) -> float | None:
    if display_deg is None:
        return None
    sign = LEFT_SIDE_KINEMATIC_SIGN_BY_MOTOR.get(int(motor_id), 1.0)
    return float(display_deg) * sign


def error_state(error_deg: float | None, ready: bool, controller_fault: bool = False) -> str:
    if controller_fault:
        return "FAULT"
    if not ready:
        return "NOT READY"
    if error_deg is None:
        return "N/A"
    magnitude = abs(error_deg)
    if magnitude < 2.0:
        return "OK"
    if magnitude < 5.0:
        return "CHECK"
    return "LARGE ERROR"


def format_deg(value: float | None) -> str:
    return "N/A" if value is None else f"{value:+.1f}°"


def build_joint_rows(
    motors: tuple[tuple[int, str, str], ...],
    ready_ids: set[int],
    targets: Mapping[int, float | None],
    actuals: Mapping[int, float | None],
    state_by_id: Mapping[int, str] | None = None,
    controller_fault: bool = False,
) -> list[JointDisplayRow]:
    rows: list[JointDisplayRow] = []
    state_by_id = state_by_id or {}
    for motor_id, joint, model in motors:
        target = targets.get(motor_id)
        actual = actuals.get(motor_id)
        error = joint_error(actual, target)
        state = str(state_by_id.get(motor_id) or error_state(error, motor_id in ready_ids, controller_fault))
        if controller_fault:
            state = "FAULT"
        elif motor_id not in ready_ids:
            state = "NOT READY"
        rows.append(JointDisplayRow(motor_id, joint, model, target, actual, error, state))
    return rows


def max_error_row(rows: list[JointDisplayRow]) -> JointDisplayRow | None:
    candidates = [row for row in rows if row.error is not None and row.state != "NOT READY"]
    if not candidates:
        return None
    return max(candidates, key=lambda row: abs(float(row.error)))


def compute_side_leg_points(
    hip_origin: Point,
    hip_pitch_deg: float,
    knee_deg: float,
    ankle_pitch_deg: float,
    thigh_length: float,
    shank_length: float,
    foot_length: float,
) -> LegPoints:
    hip_x, hip_y = hip_origin
    hip_angle = math.pi / 2.0 + math.radians(float(hip_pitch_deg))
    knee_angle = hip_angle + math.radians(float(knee_deg))
    foot_angle = knee_angle - math.pi / 2.0 - math.radians(float(ankle_pitch_deg))

    knee_x = hip_x + thigh_length * math.cos(hip_angle)
    knee_y = hip_y + thigh_length * math.sin(hip_angle)
    ankle_x = knee_x + shank_length * math.cos(knee_angle)
    ankle_y = knee_y + shank_length * math.sin(knee_angle)
    toe_x = ankle_x + foot_length * math.cos(foot_angle)
    toe_y = ankle_y + foot_length * math.sin(foot_angle)
    return LegPoints((hip_x, hip_y), (knee_x, knee_y), (ankle_x, ankle_y), (toe_x, toe_y))


def compute_front_leg_points(
    hip_origin: Point,
    hip_roll_deg: float,
    ankle_roll_deg: float,
    outward_sign: int,
    thigh_length: float,
    shank_length: float,
    foot_length: float,
) -> LegPoints:
    hip_x, hip_y = hip_origin
    frontal_angle = math.pi / 2.0 + int(outward_sign) * math.radians(float(hip_roll_deg))
    knee_x = hip_x + thigh_length * math.cos(frontal_angle)
    knee_y = hip_y + thigh_length * math.sin(frontal_angle)
    ankle_x = knee_x + shank_length * math.cos(frontal_angle)
    ankle_y = knee_y + shank_length * math.sin(frontal_angle)

    # The toe point carries ankle-roll direction for the simplified front-view
    # ankle marker; no calibration direction is applied here.
    foot_angle = int(outward_sign) * math.radians(float(ankle_roll_deg))
    toe_x = ankle_x + int(outward_sign) * foot_length * math.cos(foot_angle)
    toe_y = ankle_y + foot_length * math.sin(foot_angle)
    return LegPoints((hip_x, hip_y), (knee_x, knee_y), (ankle_x, ankle_y), (toe_x, toe_y))


def _finite_value(value: float | None) -> float | None:
    if value is None:
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def _value(values: Mapping[int, float | None], motor_id: int) -> float | None:
    return _finite_value(values.get(motor_id))


def _leg_values(values: Mapping[int, float | None], motor_ids: tuple[int, int, int]) -> tuple[float, float, float] | None:
    found = [_value(values, motor_id) for motor_id in motor_ids]
    if all(value is None for value in found):
        return None
    # Missing joints are neutral only for connecting the visible partial chain;
    # table/error data still reports those joints as N/A.
    return tuple(0.0 if value is None else value for value in found)  # type: ignore[return-value]


def _side_kinematic_values(
    values: Mapping[int, float | None],
    motor_ids: tuple[int, int, int],
    side: str | None = None,
) -> tuple[float, float, float] | None:
    display_values = _leg_values(values, motor_ids)
    if display_values is None:
        return None
    hip_pitch, knee, ankle_pitch = display_values
    if side is not None and side.lower().startswith("l"):
        ankle_pitch = to_left_side_kinematic_deg(motor_ids[2], ankle_pitch) or 0.0
    return (
        to_side_kinematic_deg(motor_ids[0], hip_pitch) or 0.0,
        knee,
        ankle_pitch,
    )


def _draw_polyline(
    canvas: Any,
    points: LegPoints,
    color: str,
    width: int,
    dash: tuple[int, int] | None = None,
    joints: bool = False,
) -> None:
    coords = [
        points.hip,
        points.knee,
        points.ankle,
        points.toe,
    ]
    for start, end in zip(coords, coords[1:]):
        canvas.create_line(*start, *end, fill=color, width=width, dash=dash)
    if joints:
        for x, y in coords[:-1]:
            radius = 4
            canvas.create_oval(x - radius, y - radius, x + radius, y + radius, fill=color, outline="")


def _draw_front_balance_leg(
    canvas: Any,
    points: LegPoints,
    color: str,
    width: int,
    dash: tuple[int, int] | None = None,
    joints: bool = False,
) -> None:
    canvas.create_line(*points.hip, *points.knee, fill=color, width=width, dash=dash)
    canvas.create_line(*points.knee, *points.ankle, fill=color, width=width, dash=dash)

    ankle_x, ankle_y = points.ankle
    roll_dx = points.toe[0] - ankle_x
    roll_dy = points.toe[1] - ankle_y
    length = math.hypot(roll_dx, roll_dy) or 1.0
    marker = 22.0
    unit_x = roll_dx / length
    unit_y = roll_dy / length
    canvas.create_line(
        ankle_x - unit_x * marker / 2.0,
        ankle_y - unit_y * marker / 2.0,
        ankle_x + unit_x * marker / 2.0,
        ankle_y + unit_y * marker / 2.0,
        fill=color,
        width=max(2, width - 1),
        dash=dash,
    )

    if joints:
        for x, y in (points.hip, points.knee, points.ankle):
            radius = 4
            canvas.create_oval(x - radius, y - radius, x + radius, y + radius, fill=color, outline="")


def _canvas_size(canvas: Any) -> tuple[int, int]:
    width = int(canvas.winfo_width())
    height = int(canvas.winfo_height())
    if width <= 1 and hasattr(canvas, "cget"):
        try:
            width = int(float(canvas.cget("width")))
        except (TypeError, ValueError):
            width = 1
    if height <= 1 and hasattr(canvas, "cget"):
        try:
            height = int(float(canvas.cget("height")))
        except (TypeError, ValueError):
            height = 1
    width = max(width, 1)
    height = max(height, 1)
    return width, height


def _label(canvas: Any, x: float, y: float, text: str, anchor: str = "nw") -> None:
    canvas.create_text(x, y, text=text, anchor=anchor, fill="#263238", font=("TkDefaultFont", 8))


def _summary(
    values_actual: Mapping[int, float | None],
    values_target: Mapping[int, float | None],
    items: tuple[tuple[str, int], ...],
) -> str:
    lines: list[str] = []
    for label, motor_id in items:
        lines.append(
            f"{label}  A:{format_deg(_value(values_actual, motor_id))}  "
            f"T:{format_deg(_value(values_target, motor_id))}"
        )
    return "\n".join(lines)


def draw_side_leg_view(
    canvas: Any,
    actuals: Mapping[int, float | None],
    targets: Mapping[int, float | None],
    ready_ids: set[int],
    side: str,
) -> None:
    canvas.delete("all")
    width, height = _canvas_size(canvas)
    is_right = side.lower().startswith("r")
    title = "오른쪽 다리 측면 보기" if is_right else "왼쪽 다리 측면 보기"
    side_label = "R" if is_right else "L"
    motor_ids = (1, 4, 5) if is_right else (7, 10, 11)
    summary_items = (
        (("R Hip Pitch", 1), ("R Knee", 4), ("R Ankle Pitch", 5))
        if is_right
        else (("L Hip Pitch", 7), ("L Knee", 10), ("L Ankle Pitch", 11))
    )

    thigh = min(height * 0.17, width * 0.18)
    shank = min(height * 0.17, width * 0.18)
    foot = min(height * 0.08, width * 0.09)
    center_x = width / 2.0
    hip = (center_x, max(68.0, height * 0.30))

    canvas.create_text(width / 2, 13, text=title, font=("TkDefaultFont", 10, "bold"))
    canvas.create_text(8, 13, text="Actual / Target", anchor="w", fill="#455a64", font=("TkDefaultFont", 8))
    canvas.create_line(24, 34, width * 0.38, 34, fill="#78909c", width=2, arrow="first")
    canvas.create_text(width * 0.38 + 6, 34, text="BACK", anchor="w", fill="#455a64", font=("TkDefaultFont", 8, "bold"))
    canvas.create_line(width * 0.62, 34, width - 24, 34, fill="#78909c", width=2, arrow="last")
    canvas.create_text(width * 0.62 - 6, 34, text="FRONT", anchor="e", fill="#455a64", font=("TkDefaultFont", 8, "bold"))
    canvas.create_rectangle(hip[0] - 46, hip[1] - 12, hip[0] + 46, hip[1] + 10, outline=STRUCTURE_COLOR, width=2)
    canvas.create_line(hip[0], max(42, hip[1] - 54), hip[0], hip[1] - 12, fill=STRUCTURE_COLOR, width=4)
    canvas.create_text(hip[0] + 8, max(44, hip[1] - 48), text="torso", anchor="w", fill=STRUCTURE_COLOR, font=("TkDefaultFont", 8))

    neutral = compute_side_leg_points(hip, 0.0, 0.0, 0.0, thigh, shank, foot)
    if not set(motor_ids).intersection(ready_ids):
        _draw_polyline(canvas, neutral, NEUTRAL_COLOR, 3, joints=True)
    target_values = _side_kinematic_values(targets, motor_ids, side)
    actual_values = _side_kinematic_values(actuals, motor_ids, side)
    if target_values is not None:
        _draw_polyline(
            canvas,
            compute_side_leg_points(hip, *target_values, thigh, shank, foot),
            TARGET_COLOR,
            3,
            dash=(6, 4),
        )
    if actual_values is not None:
        _draw_polyline(
            canvas,
            compute_side_leg_points(hip, *actual_values, thigh, shank, foot),
            ACTUAL_COLOR,
            5,
            joints=True,
        )
    canvas.create_text(hip[0], hip[1] + 13, text=side_label, anchor="n", fill="#263238", font=("TkDefaultFont", 9, "bold"))

    _label(
        canvas,
        8,
        max(46, height - 54),
        _summary(actuals, targets, summary_items),
    )


def draw_side_view(
    canvas: Any,
    actuals: Mapping[int, float | None],
    targets: Mapping[int, float | None],
    ready_ids: set[int],
) -> None:
    draw_side_leg_view(canvas, actuals, targets, ready_ids, "right")


def draw_front_view(
    canvas: Any,
    actuals: Mapping[int, float | None],
    targets: Mapping[int, float | None],
    ready_ids: set[int],
) -> None:
    canvas.delete("all")
    width, height = _canvas_size(canvas)
    thigh = height * 0.22
    shank = height * 0.22
    foot = height * 0.05
    hip_y = height * 0.25
    right_hip = (width * 0.43, hip_y)
    left_hip = (width * 0.57, hip_y)

    canvas.create_text(width / 2, 14, text="정면 보기", font=("TkDefaultFont", 10, "bold"))
    canvas.create_text(8, 14, text="Actual / Target", anchor="w", fill="#455a64", font=("TkDefaultFont", 8))
    canvas.create_line(width * 0.50, 34, width * 0.50, height - 74, fill="#cfd8dc", width=1, dash=(4, 4))
    canvas.create_text(width * 0.50 + 6, 42, text="몸 중심선", anchor="w", fill="#607d8b", font=("TkDefaultFont", 8))
    canvas.create_rectangle(width * 0.39, hip_y - 18, width * 0.61, hip_y + 12, outline=STRUCTURE_COLOR, width=2)
    canvas.create_line(*right_hip, *left_hip, fill=STRUCTURE_COLOR, width=4)
    canvas.create_line(width * 0.50, hip_y - 82, width * 0.50, hip_y - 18, fill=STRUCTURE_COLOR, width=4)
    canvas.create_text(width * 0.50, hip_y - 92, text="torso", fill=STRUCTURE_COLOR)
    canvas.create_text(right_hip[0], hip_y - 28, text="오른쪽(R)", anchor="s", fill="#263238", font=("TkDefaultFont", 9, "bold"))
    canvas.create_text(left_hip[0], hip_y - 28, text="왼쪽(L)", anchor="s", fill="#263238", font=("TkDefaultFont", 9, "bold"))

    legs = (
        ("R", right_hip, (2, 6), 1),
        ("L", left_hip, (8, 12), -1),
    )
    for side, hip, motor_ids, outward_sign in legs:
        neutral = compute_front_leg_points(hip, 0.0, 0.0, outward_sign, thigh, shank, foot)
        if not set(motor_ids).intersection(ready_ids):
            _draw_front_balance_leg(canvas, neutral, NEUTRAL_COLOR, 3, joints=True)
        target_roll = _leg_values(targets, (motor_ids[0], motor_ids[1], motor_ids[1]))
        actual_roll = _leg_values(actuals, (motor_ids[0], motor_ids[1], motor_ids[1]))
        if target_roll is not None:
            _draw_front_balance_leg(
                canvas,
                compute_front_leg_points(
                    hip,
                    to_front_kinematic_deg(motor_ids[0], target_roll[0]) or 0.0,
                    target_roll[1],
                    outward_sign,
                    thigh,
                    shank,
                    foot,
                ),
                TARGET_COLOR,
                3,
                dash=(6, 4),
            )
        if actual_roll is not None:
            _draw_front_balance_leg(
                canvas,
                compute_front_leg_points(
                    hip,
                    to_front_kinematic_deg(motor_ids[0], actual_roll[0]) or 0.0,
                    actual_roll[1],
                    outward_sign,
                    thigh,
                    shank,
                    foot,
                ),
                ACTUAL_COLOR,
                5,
                joints=True,
            )
        canvas.create_text(hip[0], hip[1] + 8, text=side, anchor="n", fill="#263238", font=("TkDefaultFont", 9, "bold"))

    _label(
        canvas,
        8,
        max(34, height - 104),
        _summary(
            actuals,
            targets,
            (
                ("R Hip R", 2),
                ("R Ankle R", 6),
                ("L Hip R", 8),
                ("L Ankle R", 12),
            ),
        ),
    )
