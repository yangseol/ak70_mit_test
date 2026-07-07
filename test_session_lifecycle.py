import inspect
import queue
import time
from pathlib import Path

import pytest

import ak70_control_center_gui as gui
from ak_realtime_core import ControlMode, RealtimeCore
from run_realtime_controller import RealtimeController


TOKEN = "12345678abcdef"


def test_arm_success_creates_active_session():
    core = RealtimeCore([0x001], control_mode=ControlMode.AK70_GUI_PERSISTENT_LEASE)
    response = core.arm(owner="test", session_token=TOKEN)

    assert response["active_session"] is True
    assert response["session_token"] == TOKEN
    assert core.status()["active_session"] is True


def test_arm_token_is_saved_to_gui_persistent_session():
    app = object.__new__(gui.ControlCenterApp)
    app.event_queue = queue.Queue()
    app.session_token = None
    app.session_epoch = None
    app.heartbeat_seq = 99
    app._one_click_arm_complete = False
    app._heartbeat_enabled = True
    app._first_heartbeat_logged = True
    app._session_lost_logged = True

    token, epoch = app._store_arm_session(
        {"ok": True, "session_token": TOKEN, "session_epoch": 4},
        TOKEN,
        None,
    )

    assert (token, epoch) == (TOKEN, 4)
    assert app.session_token == TOKEN
    assert app.session_epoch == 4
    assert app._one_click_arm_complete is True
    assert app._heartbeat_enabled is False


def _started_dry_controller():
    calibration = Path(__file__).resolve().parent / "motor_calibration.yaml"
    controller = RealtimeController(
        "can0",
        [0x001],
        50.0,
        True,
        ControlMode.AK70_GUI_PERSISTENT_LEASE,
        calibration,
    )
    arm = controller.core.arm(owner="test", session_token=TOKEN)
    controller.core.on_feedback(0x001, 0.1)
    started = controller.start_detected_motors(TOKEN, arm["session_epoch"])
    return controller, arm, started


def test_detection_and_hold_keep_same_session_token():
    controller, arm, started = _started_dry_controller()

    assert started["detected_ids"] == ["0x001"]
    assert controller.core.session is not None
    assert controller.core.session.token == TOKEN
    assert controller.core.latest_targets[0x001].session_token == TOKEN
    assert controller.core.status()["active_target_ids"] == ["0x001"]
    assert controller.core.status()["session_epoch"] == arm["session_epoch"]


def test_first_heartbeat_succeeds_after_hold_creation():
    controller, _arm, _started = _started_dry_controller()
    now = time.monotonic()
    response = controller.core.heartbeat(TOKEN, 1, now, now=now)

    assert response["ok"] is True
    assert response["heartbeat_accepted"] is True
    assert response["status"]["active_session"] is True


def test_success_path_contains_no_release_command():
    source = inspect.getsource(gui.ControlCenterApp._start_sequence)
    assert "RELEASE_SESSION" not in source
    assert "RELEASE_IDS" not in source
    assert "ESTOP" not in source
    assert "SHUTDOWN" not in source


def test_no_active_session_cannot_complete_one_click():
    with pytest.raises(RuntimeError, match="no active session"):
        gui.ControlCenterApp._validate_one_click_status(
            {
                "running": True,
                "active_session": False,
                "session_token": None,
                "active_target_ids": ["0x001"],
            },
            TOKEN,
            ["0x001"],
        )


class DummyVar:
    def __init__(self, value=""):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


def _session_loss_app():
    app = object.__new__(gui.ControlCenterApp)
    app._session_lost_logged = False
    app._heartbeat_enabled = True
    app._one_click_arm_complete = True
    app.session_token = TOKEN
    app.session_epoch = 1
    app.dirty_targets = {1: 2.0}
    app.stream_inflight = True
    app.controller_var = DummyVar("ARMED")
    app.mode_var = DummyVar("WALK_PRESET")
    calls = {"walk": 0, "isaac": 0, "logs": []}
    app.stop_walk = lambda: calls.__setitem__("walk", calls["walk"] + 1)
    app.stop_isaac_follow = lambda: calls.__setitem__("isaac", calls["isaac"] + 1)
    app.log_ipc_error = lambda command, error: calls["logs"].append((command, error))
    return app, calls


def test_session_lost_stops_stream_walk_and_isaac():
    app, calls = _session_loss_app()
    app._handle_session_lost("no active control session")

    assert app.session_token is None
    assert app._heartbeat_enabled is False
    assert app.dirty_targets == {}
    assert app.stream_inflight is False
    assert calls["walk"] == 1
    assert calls["isaac"] == 1
    assert app.controller_var.get() == "SESSION LOST"


def test_session_lost_error_is_logged_only_once():
    app, calls = _session_loss_app()
    first = {"ok": False, "error": "no active control session"}
    second = {"ok": False, "error": "no active control session"}
    app._on_heartbeat_response(first)
    app._on_heartbeat_response(second)

    assert len(calls["logs"]) == 1
    assert first["_session_lost"] is True
    assert second["_session_lost"] is True


def test_release_session_only_in_explicit_release_and_close_paths():
    start_source = inspect.getsource(gui.ControlCenterApp._start_sequence)
    release_source = inspect.getsource(gui.ControlCenterApp.release_all)
    close_source = inspect.getsource(gui.ControlCenterApp.on_close)

    assert "RELEASE_SESSION" not in start_source
    assert "RELEASE_SESSION" in release_source
    assert "RELEASE_SESSION" in close_source
