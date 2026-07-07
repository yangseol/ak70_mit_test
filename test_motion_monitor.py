import math
from pathlib import Path

import ak70_control_center_gui as gui
from motion_monitor import (
    BAR_DISPLAY_SIGN_BY_MOTOR,
    FRONT_KINEMATIC_SIGN_BY_MOTOR,
    LEFT_SIDE_KINEMATIC_SIGN_BY_MOTOR,
    UI_DISPLAY_SIGN_BY_MOTOR,
    _side_kinematic_values,
    build_joint_rows,
    compute_front_leg_points,
    compute_side_leg_points,
    error_state,
    draw_front_view,
    draw_side_leg_view,
    joint_error,
    to_bar_display_deg,
    to_bar_display_range,
    to_front_kinematic_deg,
    to_left_side_kinematic_deg,
    to_side_kinematic_deg,
    to_ui_display_deg,
)


def test_side_zero_pose_points_down_and_foot_forward():
    points = compute_side_leg_points((0.0, 0.0), 0.0, 0.0, 0.0, 100.0, 100.0, 40.0)

    assert points.knee[0] == pytest_approx(0.0)
    assert points.ankle[0] == pytest_approx(0.0)
    assert points.knee[1] > points.hip[1]
    assert points.ankle[1] > points.knee[1]
    assert points.toe[0] > points.ankle[0]
    assert points.toe[1] == pytest_approx(points.ankle[1])


def test_side_hip_pitch_positive_moves_leg_backward():
    points = compute_side_leg_points((0.0, 0.0), 10.0, 0.0, 0.0, 100.0, 100.0, 40.0)

    assert points.knee[0] < 0.0


def test_side_knee_positive_bends_knee():
    points = compute_side_leg_points((0.0, 0.0), 0.0, 30.0, 0.0, 100.0, 100.0, 40.0)

    assert points.ankle[0] < points.knee[0]


def test_side_ankle_pitch_positive_lifts_toe():
    neutral = compute_side_leg_points((0.0, 0.0), 0.0, 0.0, 0.0, 100.0, 100.0, 40.0)
    lifted = compute_side_leg_points((0.0, 0.0), 0.0, 0.0, 15.0, 100.0, 100.0, 40.0)

    assert lifted.toe[1] < neutral.toe[1]


def test_front_right_and_left_hip_roll_positive_move_outward():
    right = compute_front_leg_points((0.0, 0.0), 10.0, 0.0, 1, 100.0, 100.0, 40.0)
    left = compute_front_leg_points((0.0, 0.0), 10.0, 0.0, -1, 100.0, 100.0, 40.0)

    assert right.knee[0] < 0.0
    assert left.knee[0] > 0.0


def test_joint_error_and_na_state():
    assert joint_error(8.0, 10.5) == -2.5
    assert joint_error(None, 10.5) is None
    assert error_state(1.9, ready=True) == "OK"
    assert error_state(2.0, ready=True) == "CHECK"
    assert error_state(5.0, ready=True) == "LARGE ERROR"
    assert error_state(None, ready=True) == "N/A"
    assert error_state(0.0, ready=False) == "NOT READY"


def test_ready_subset_render_rows_and_id10_direction_not_reapplied():
    rows = build_joint_rows(
        gui.MOTORS,
        {7, 8, 10, 11},
        {10: 20.0},
        {10: 17.5},
    )
    by_id = {row.motor_id: row for row in rows}

    assert by_id[10].joint == "left_knee"
    assert by_id[10].target == 20.0
    assert by_id[10].actual == 17.5
    assert by_id[10].error == -2.5
    assert by_id[9].target is None
    assert by_id[9].state == "NOT READY"


def test_ui_display_sign_applies_only_to_id1():
    assert UI_DISPLAY_SIGN_BY_MOTOR[1] == -1.0
    for motor_id in range(2, 13):
        assert UI_DISPLAY_SIGN_BY_MOTOR.get(motor_id, 1.0) == 1.0
    assert to_ui_display_deg(1, 10.0) == -10.0
    assert to_ui_display_deg(1, -10.0) == 10.0
    assert to_ui_display_deg(7, 10.0) == 10.0
    assert to_ui_display_deg(1, None) is None


def test_id1_actual_and_target_display_are_both_flipped_without_changing_cache():
    app = object.__new__(gui.ControlCenterApp)
    app.ready_ids = {1, 7}
    app.status_cache = {"motors": {}}
    app.actual = {motor_id: None for motor_id in gui.ID_TO_NAME}
    app.targets = {motor_id: 0.0 for motor_id in gui.ID_TO_NAME}
    app.actual[1] = 8.0
    app.targets[1] = 10.0
    app.actual[7] = 8.0
    app.targets[7] = 10.0

    actuals, targets = app._motion_value_maps()

    assert actuals[1] == -8.0
    assert targets[1] == -10.0
    assert actuals[7] == 8.0
    assert targets[7] == 10.0
    assert app.actual[1] == 8.0
    assert app.targets[1] == 10.0
    assert abs(joint_error(app.actual[1], app.targets[1])) == abs(joint_error(actuals[1], targets[1]))


def test_id1_control_payloads_are_not_ui_flipped():
    items = gui.build_stream_target_items({1: 10.0})

    assert items == [
        {
            "motor_id": "0x001",
            "position_deg": 10.0,
            "target_deg": 10.0,
            "move_sec": 0.0,
        }
    ]


def test_id1_isaac_and_walk_targets_are_not_ui_flipped():
    targets, legacy = gui.parse_isaac_payload('{"right_hip_pitch": 10.0}')
    assert legacy is False
    assert targets[1] == 10.0

    trajectory = gui.load_walk_cycle(gui.WALK_CYCLE_PATH)
    phase_targets = gui.interpolate_walk_cycle(trajectory, trajectory.samples[0].time_sec)
    assert phase_targets[1] == trajectory.samples[0].targets_deg[1]


def test_ui_display_helpers_do_not_modify_calibration_file():
    path = Path("motor_calibration.yaml")
    before = path.read_bytes()

    assert to_ui_display_deg(1, 10.0) == -10.0
    assert gui.build_stream_target_items({1: 10.0})[0]["position_deg"] == 10.0

    assert path.read_bytes() == before


def test_side_kinematic_uses_same_value_as_ui_display_for_id1():
    display_id1 = to_ui_display_deg(1, 10.0)
    assert display_id1 == -10.0
    assert to_side_kinematic_deg(1, display_id1) == -10.0
    assert to_side_kinematic_deg(7, 10.0) == 10.0
    assert to_side_kinematic_deg(1, None) is None


def test_side_kinematic_keeps_id1_display_actual_and_target_values():
    actuals = {1: -8.0, 4: 2.0, 5: 3.0, 7: 8.0, 10: 2.0, 11: 3.0}
    targets = {1: -10.0, 4: 4.0, 5: 5.0, 7: 10.0, 10: 4.0, 11: 5.0}

    assert _side_kinematic_values(actuals, (1, 4, 5)) == (-8.0, 2.0, 3.0)
    assert _side_kinematic_values(targets, (1, 4, 5)) == (-10.0, 4.0, 5.0)
    assert _side_kinematic_values(actuals, (7, 10, 11)) == (8.0, 2.0, 3.0)
    assert _side_kinematic_values(targets, (7, 10, 11)) == (10.0, 4.0, 5.0)


def test_side_kinematic_does_not_change_bar_numbers_payload_or_front_math():
    assert to_ui_display_deg(1, 10.0) == -10.0
    assert gui.build_stream_target_items({1: 10.0})[0]["target_deg"] == 10.0

    right_before = compute_front_leg_points((0.0, 0.0), 10.0, 0.0, 1, 100.0, 100.0, 40.0)
    right_after = compute_front_leg_points((0.0, 0.0), 10.0, 0.0, 1, 100.0, 100.0, 40.0)
    assert right_before == right_after


class DummyVar:
    def __init__(self, value=""):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class DummyRoot:
    def __init__(self):
        self.after_calls = []

    def after(self, delay, callback):
        self.after_calls.append((delay, callback))
        return "after-id"


class DummyCanvas:
    def __init__(self, width=500, height=520):
        self.calls = []
        self.width = width
        self.height = height

    def winfo_width(self):
        return self.width

    def winfo_height(self):
        return self.height

    def cget(self, key):
        if key == "width":
            return self.width
        if key == "height":
            return self.height
        raise KeyError(key)

    def delete(self, *args):
        self.calls.append(("delete", args))

    def create_text(self, *args, **kwargs):
        self.calls.append(("text", args, kwargs))

    def create_line(self, *args, **kwargs):
        self.calls.append(("line", args, kwargs))

    def create_rectangle(self, *args, **kwargs):
        self.calls.append(("rectangle", args, kwargs))

    def create_oval(self, *args, **kwargs):
        self.calls.append(("oval", args, kwargs))


class DummyPanel:
    def __init__(self):
        self.visible = True
        self.grid_calls = 0
        self.grid_remove_calls = 0

    def grid(self):
        self.visible = True
        self.grid_calls += 1

    def grid_remove(self):
        self.visible = False
        self.grid_remove_calls += 1


class FakeText:
    def __init__(self, *_args, **_kwargs):
        self.content = ""

    def pack(self, *_args, **_kwargs):
        pass

    def insert(self, _where, text):
        self.content += text

    def configure(self, **_kwargs):
        pass

    def see(self, *_args):
        pass


class FakeTop:
    instances = []

    def __init__(self, _root):
        self.exists = True
        self.lift_count = 0
        FakeTop.instances.append(self)

    def title(self, *_args):
        pass

    def geometry(self, *_args):
        pass

    def protocol(self, *_args):
        pass

    def winfo_exists(self):
        return self.exists

    def deiconify(self):
        pass

    def lift(self):
        self.lift_count += 1

    def focus_force(self):
        pass

    def destroy(self):
        self.exists = False


def test_canvas_update_uses_cached_values_and_does_not_call_ipc():
    app = object.__new__(gui.ControlCenterApp)
    app._closing = False
    app.motion_monitor_after_id = None
    app.root = DummyRoot()
    app.ready_ids = {7, 8, 10, 11}
    app.status_cache = {"mode": "ARMED", "motors": {}}
    app.actual = {motor_id: None for motor_id in gui.ID_TO_NAME}
    app.targets = {motor_id: 0.0 for motor_id in gui.ID_TO_NAME}
    app.actual[10] = 17.5
    app.targets[10] = 20.0
    app.state_vars = {motor_id: DummyVar("READY" if motor_id in app.ready_ids else "NOT FOUND") for motor_id in gui.ID_TO_NAME}
    app.mode_var = DummyVar("WALK_PRESET")
    app.motion_mode_var = DummyVar()
    app.motion_ready_var = DummyVar()
    app.motion_error_var = DummyVar()
    app.motion_state_var = DummyVar()
    app.recent_status_var = DummyVar()
    app.front_canvas = DummyCanvas()
    app.right_side_canvas = DummyCanvas()
    app.left_side_canvas = DummyCanvas()

    def forbidden(*_args, **_kwargs):
        raise AssertionError("motion monitor must not send IPC")

    app.enqueue_ipc = forbidden
    app.update_motion_monitor()

    assert app.motion_monitor_after_id == "after-id"
    assert app.root.after_calls == [(50, app.update_motion_monitor)]
    assert app.motion_ready_var.get() == "READY: 4/12"
    assert "ID 10" in app.motion_error_var.get()
    assert "target +20.0°" in app.recent_status_var.get()
    assert any(call[0] == "line" for call in app.front_canvas.calls)
    assert any(call[0] == "line" for call in app.right_side_canvas.calls)
    assert any(call[0] == "line" for call in app.left_side_canvas.calls)


def test_side_view_is_split_by_leg_and_marks_front_back_direction():
    right_canvas = DummyCanvas()
    left_canvas = DummyCanvas()
    actuals = {1: 0.0, 4: 10.0, 5: -5.0, 7: 0.0, 10: -10.0, 11: 5.0}
    targets = {1: 2.0, 4: 8.0, 5: -3.0, 7: -2.0, 10: -8.0, 11: 3.0}

    draw_side_leg_view(right_canvas, actuals, targets, {1, 4, 5}, "right")
    draw_side_leg_view(left_canvas, actuals, targets, {7, 10, 11}, "left")

    right_text = " ".join(str(call[2].get("text", "")) for call in right_canvas.calls if call[0] == "text")
    left_text = " ".join(str(call[2].get("text", "")) for call in left_canvas.calls if call[0] == "text")

    assert "오른쪽 다리 측면 보기" in right_text
    assert "왼쪽 다리 측면 보기" in left_text
    assert "FRONT" in right_text
    assert "BACK" in right_text
    assert "FRONT" in left_text
    assert "BACK" in left_text
    assert "L Hip Pitch" not in right_text
    assert "R Hip Pitch" not in left_text


def test_front_view_keeps_roll_summary_without_yaw_or_detailed_foot_shape():
    canvas = DummyCanvas()
    actuals = {2: 3.0, 6: -2.0, 8: -3.0, 12: 2.0}
    targets = {2: 1.0, 6: -1.0, 8: -1.0, 12: 1.0}

    draw_front_view(canvas, actuals, targets, {2, 6, 8, 12})

    text = " ".join(str(call[2].get("text", "")) for call in canvas.calls if call[0] == "text")
    assert "몸 중심선" in text
    assert "오른쪽(R)" in text
    assert "왼쪽(L)" in text
    assert "Hip Y" not in text
    assert "R Ankle R" in text
    assert "L Ankle R" in text


def test_id2_front_kinematic_sign_only_flips_front_x_direction():
    assert FRONT_KINEMATIC_SIGN_BY_MOTOR == {2: -1.0}
    assert to_ui_display_deg(2, 10.0) == 10.0
    assert to_bar_display_deg(2, 10.0) == 10.0
    assert _side_kinematic_values({2: 10.0}, (2, 4, 5)) == (10.0, 0.0, 0.0)

    uncorrected = compute_front_leg_points((0.0, 0.0), 10.0, 0.0, 1, 100.0, 100.0, 10.0)
    actual_corrected = compute_front_leg_points((0.0, 0.0), to_front_kinematic_deg(2, 10.0) or 0.0, 0.0, 1, 100.0, 100.0, 10.0)
    target_corrected = compute_front_leg_points((0.0, 0.0), to_front_kinematic_deg(2, 20.0) or 0.0, 0.0, 1, 100.0, 100.0, 10.0)

    assert uncorrected.knee[0] < 0.0
    assert actual_corrected.knee[0] > 0.0
    assert target_corrected.knee[0] > actual_corrected.knee[0]
    assert to_front_kinematic_deg(8, 10.0) == 10.0


def test_id10_bar_sign_only_keeps_numbers_and_side_knee_kinematics():
    assert BAR_DISPLAY_SIGN_BY_MOTOR == {10: -1.0}
    assert 10 not in LEFT_SIDE_KINEMATIC_SIGN_BY_MOTOR
    assert to_ui_display_deg(10, 17.5) == 17.5
    assert to_bar_display_deg(10, 17.5) == -17.5
    assert to_bar_display_range(10, -120.0, 120.0) == (-120.0, 120.0)

    app = object.__new__(gui.ControlCenterApp)
    assert app._format_compact_display("A", 10, 17.5) == "A:+17.5"
    assert app._format_compact_display("T", 10, 20.0) == "T:+20.0"
    assert app._bar_display_deg(10, 17.5) == -17.5

    before = _side_kinematic_values({7: 0.0, 10: 25.0, 11: 0.0}, (7, 10, 11))
    after = _side_kinematic_values({7: 0.0, 10: 25.0, 11: 0.0}, (7, 10, 11), "left")
    assert before == (0.0, 25.0, 0.0)
    assert after == before


def test_id11_left_side_ankle_sign_only_flips_foot_segment():
    assert LEFT_SIDE_KINEMATIC_SIGN_BY_MOTOR == {11: -1.0}
    assert to_ui_display_deg(11, 15.0) == 15.0
    assert to_front_kinematic_deg(11, 15.0) == 15.0
    assert to_left_side_kinematic_deg(11, 15.0) == -15.0

    unchanged_right = _side_kinematic_values({1: 0.0, 4: 0.0, 5: 15.0}, (1, 4, 5), "right")
    corrected_left = _side_kinematic_values({7: 0.0, 10: 0.0, 11: 15.0}, (7, 10, 11), "left")
    uncorrected_left = _side_kinematic_values({7: 0.0, 10: 0.0, 11: 15.0}, (7, 10, 11))

    assert unchanged_right == (0.0, 0.0, 15.0)
    assert uncorrected_left == (0.0, 0.0, 15.0)
    assert corrected_left == (0.0, 0.0, -15.0)

    uncorrected_points = compute_side_leg_points((0.0, 0.0), *uncorrected_left, 100.0, 100.0, 40.0)
    corrected_points = compute_side_leg_points((0.0, 0.0), *corrected_left, 100.0, 100.0, 40.0)
    assert uncorrected_points.ankle == corrected_points.ankle
    assert uncorrected_points.toe[1] < uncorrected_points.ankle[1]
    assert corrected_points.toe[1] > corrected_points.ankle[1]


def test_other_motor_display_signs_are_unchanged():
    assert UI_DISPLAY_SIGN_BY_MOTOR == {1: -1.0}
    assert to_ui_display_deg(1, 10.0) == -10.0
    for motor_id in (3, 4, 5, 7, 8, 9):
        assert to_ui_display_deg(motor_id, 10.0) == 10.0
        assert to_bar_display_deg(motor_id, 10.0) == 10.0
        assert to_front_kinematic_deg(motor_id, 10.0) == 10.0
        assert to_left_side_kinematic_deg(motor_id, 10.0) == 10.0


def test_side_leg_views_use_canvas_center_for_leg_geometry():
    right_canvas = DummyCanvas(width=500, height=240)
    left_canvas = DummyCanvas(width=760, height=240)

    draw_side_leg_view(right_canvas, {}, {}, set(), "right")
    draw_side_leg_view(left_canvas, {}, {}, set(), "left")

    right_pelvis = next(call for call in right_canvas.calls if call[0] == "rectangle")
    left_pelvis = next(call for call in left_canvas.calls if call[0] == "rectangle")
    right_center = (right_pelvis[1][0] + right_pelvis[1][2]) / 2.0
    left_center = (left_pelvis[1][0] + left_pelvis[1][2]) / 2.0

    assert right_center == pytest_approx(250.0)
    assert left_center == pytest_approx(380.0)


def test_partial_ready_side_and_front_render_without_full_ready_set():
    front_canvas = DummyCanvas(width=500, height=300)
    right_canvas = DummyCanvas(width=500, height=240)
    left_canvas = DummyCanvas(width=500, height=240)

    draw_front_view(front_canvas, {2: 5.0}, {2: 7.0}, {2})
    draw_side_leg_view(right_canvas, {1: 0.0, 4: 5.0}, {1: 1.0, 4: 6.0}, {1, 4}, "right")
    draw_side_leg_view(left_canvas, {11: 4.0}, {11: 6.0}, {11}, "left")

    assert any(call[0] == "line" for call in front_canvas.calls)
    assert any(call[0] == "line" for call in right_canvas.calls)
    assert any(call[0] == "line" for call in left_canvas.calls)


def test_compact_ui_tab_labels_and_screen_fit():
    assert gui.TAB_LABELS == ("1. 시작 / 원점", "2. 수동 / 모니터", "3. Isaac Sim")
    assert gui.compact_window_geometry(1366, 768) == (1284, 691)
    assert gui.compact_window_geometry(1920, 1080) == (1500, 900)


def test_source_has_no_large_motion_treeview_or_main_log_frame():
    source = Path("ak70_control_center_gui.py").read_text(encoding="utf-8")

    assert "ttk.Treeview" not in source
    assert "12개 관절 Target / Actual / Error" not in source
    assert "log_frame = ttk.LabelFrame" not in source
    assert "notebook.add(motion_tab" not in source
    assert "text=\"2. 수동 조작\"" not in source
    assert "text=\"4. 모션 모니터\"" not in source


def test_manual_panel_toggle_does_not_change_isaac_source_or_stop_follow():
    app = object.__new__(gui.ControlCenterApp)
    app.manual_panel_visible = False
    app.manual_panel_container = DummyPanel()
    app.manual_toggle_var = DummyVar("수동 조작 열기")
    app.mode_var = DummyVar("ISAAC")
    app.redraw_motion_monitor_once = lambda: None
    stopped = []
    app.stop_isaac_follow = lambda: stopped.append(True)

    app.toggle_manual_panel()
    assert app.manual_panel_visible is True
    assert app.manual_toggle_var.get() == "수동 조작 닫기"
    assert app.mode_var.get() == "ISAAC"
    assert stopped == []

    app.toggle_manual_panel()
    assert app.manual_panel_visible is False
    assert app.manual_toggle_var.get() == "수동 조작 열기"
    assert stopped == []


def test_programmatic_slider_update_does_not_take_manual_takeover():
    app = object.__new__(gui.ControlCenterApp)
    app.suppress_scale = False
    app._updating_slider_programmatically = True
    app._last_slider_values = {10: 0.0}
    called = []
    app.manual_takeover = lambda _motor_id: called.append(True)

    app.on_slider(10, "12.0")

    assert called == []


def test_user_slider_value_change_takes_manual_control_once():
    app = object.__new__(gui.ControlCenterApp)
    app.suppress_scale = False
    app._updating_slider_programmatically = False
    app._last_slider_values = {10: 0.0}
    app.limits = {10: (-120.0, 120.0)}
    app.targets = {}
    app.target_vars = {10: DummyVar()}
    app.ready_ids = {10}
    app.dirty_targets = {}
    called = []
    app.manual_takeover = lambda motor_id: called.append(motor_id)

    app.on_slider(10, "12.0")
    app.on_slider(10, "12.0")

    assert called == [10]
    assert app.targets[10] == 12.0
    assert app.dirty_targets[10] == 12.0


def test_log_storage_updates_recent_status_without_main_text_widget():
    app = object.__new__(gui.ControlCenterApp)
    app.log_lines = []
    app.log_text = None
    app.recent_status_var = DummyVar()
    app.log("first")

    assert len(app.log_lines) == 1
    assert app.log_lines[0].endswith(" first")
    assert app.recent_status_var.get() == "최근 상태: first"


def test_log_window_toplevel_is_not_duplicated(monkeypatch):
    FakeTop.instances = []
    monkeypatch.setattr(gui.tk, "Toplevel", FakeTop)
    monkeypatch.setattr(gui, "Text", FakeText)
    app = object.__new__(gui.ControlCenterApp)
    app.root = object()
    app.log_lines = ["[00:00:00] first"]
    app.log_window = None
    app.log_text = None

    app.show_log_window()
    app.show_log_window()

    assert len(FakeTop.instances) == 1
    assert FakeTop.instances[0].lift_count == 1


def pytest_approx(value):
    return _Approx(value)


class _Approx:
    def __init__(self, expected):
        self.expected = expected

    def __eq__(self, actual):
        return math.isclose(actual, self.expected, abs_tol=1e-9)
