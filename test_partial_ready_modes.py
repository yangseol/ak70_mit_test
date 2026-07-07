from pathlib import Path

import ak70_control_center_gui as gui
from ak_realtime_core import ControlMode
from run_realtime_controller import RealtimeController


READY = {7, 8, 10, 11}


class DummyVar:
    def __init__(self, value=""):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


def _walk_app(ready_ids):
    app = object.__new__(gui.ControlCenterApp)
    app.ready_ids = set(ready_ids)
    app.walk_active_motor_ids = set()
    app.walk_trajectory = gui.load_walk_cycle(gui.WALK_CYCLE_PATH)
    app.walk_running = False
    app.walk_repeat = False
    app.walk_stage = "STOPPED"
    app.walk_transition_started = 0.0
    app.walk_cycle_started = 0.0
    app.walk_transition_from = {}
    app.walk_clamp_logged = set()
    app.dirty_targets = {}
    app.actual = {motor_id: 0.0 for motor_id in range(1, 13)}
    app.targets = {motor_id: 0.0 for motor_id in range(1, 13)}
    app.mode_var = DummyVar("MANUAL")
    app.walk_time_var = DummyVar()
    app.walk_cycle_var = DummyVar()
    app.walk_state_var = DummyVar()
    app.stop_walk = lambda **_kwargs: None
    app.stop_isaac_follow = lambda: None
    app.logs = []
    app.log = app.logs.append
    return app


def test_partial_ready_walk_starts_and_snapshots_active_ids():
    app = _walk_app(READY)
    app._begin_walk(repeat=True)

    assert app.walk_running is True
    assert app.walk_active_motor_ids == READY
    assert set(app.walk_transition_from) == READY
    assert "PARTIAL WALK" in app.walk_state_var.get()


def test_one_ready_motor_can_start_walk():
    app = _walk_app({10})
    app._begin_walk(repeat=False)

    assert app.walk_running is True
    assert app.walk_active_motor_ids == {10}


def test_zero_ready_motors_blocks_walk():
    app = _walk_app(set())
    app._begin_walk(repeat=False)

    assert app.walk_running is False
    assert app.walk_state_var.get() == "보행 시작 불가"


def test_walk_payload_contains_only_ready_subset_as_one_batch():
    all_targets = {motor_id: float(motor_id) for motor_id in range(1, 13)}
    filtered = gui.filter_targets_for_ids(all_targets, READY)
    items = gui.build_stream_target_items(filtered)

    assert set(filtered) == READY
    assert {item["motor_id"] for item in items} == {"0x007", "0x008", "0x00A", "0x00B"}
    assert len(items) == 4


def test_isaac_full_payload_keeps_valid_ready_joints():
    all_targets = {motor_id: float(motor_id) for motor_id in range(1, 13)}
    applied = gui.filter_targets_for_ids(all_targets, READY)

    assert applied == {7: 7.0, 8: 8.0, 10: 10.0, 11: 11.0}


def test_motor_loss_removes_only_missing_motor_and_zero_stops_subset():
    assert gui.retain_ready_ids(READY, {7, 10, 11}) == {7, 10, 11}
    assert gui.retain_ready_ids({7}, set()) == set()


def test_controller_accepts_atomic_stream_subset_for_active_owned_motors():
    calibration = Path(__file__).resolve().parent / "motor_calibration.yaml"
    controller = RealtimeController(
        "can0",
        sorted(READY),
        50.0,
        True,
        ControlMode.AK70_GUI_PERSISTENT_LEASE,
        calibration,
    )
    token = "partial-session"
    arm = controller.core.arm(owner="test", session_token=token)
    for motor_id in READY:
        raw_zero = float(controller.core.calibration_entry(motor_id)["raw_zero_pos_rad"])
        controller.core.on_feedback(motor_id, raw_zero)
    controller.start_detected_motors(token, arm["session_epoch"])

    response = controller.handle_message({
        "command": "SET_STREAM_TARGETS",
        "session_token": token,
        "session_epoch": arm["session_epoch"],
        "targets": [
            {"motor_id": gui.motor_key(motor_id), "position_deg": 1.0}
            for motor_id in sorted(READY)
        ],
    })

    assert response["ok"] is True
    assert set(controller.core.latest_targets) == READY


def test_existing_gui_session_is_reused_without_start_sequence():
    app = object.__new__(gui.ControlCenterApp)
    app.session_token = "own-token"
    app.status_cache = {
        "running": True,
        "active_session": True,
        "session_token": "own-token",
        "session_owner": "ak70_control_center_gui",
    }
    app.ready_ids = set(READY)
    app.pending_after_start = []
    app.start_busy = False
    app.logs = []
    app.log = app.logs.append
    app.apply_status = lambda _status: None
    app._apply_saved_gains_to_ready = lambda: None
    called = []

    app.start_all_motors(lambda: called.append(True))

    assert called == [True]
    assert any("기존 control session 재사용" in line for line in app.logs)


def test_other_owner_session_is_blocked():
    app = object.__new__(gui.ControlCenterApp)
    app.session_token = None
    app.status_cache = {
        "running": True,
        "active_session": True,
        "session_token": "other-token",
        "session_owner": "other_gui",
    }
    app.ready_ids = set(READY)
    app.pending_after_start = []
    app.start_busy = False
    app.logs = []
    app.log = app.logs.append

    app.start_all_motors()

    assert app.pending_after_start == []
    assert app.logs == ["다른 control owner가 사용 중"]
