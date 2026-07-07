import json
from pathlib import Path

import pytest

import ak70_control_center_gui as gui
from ak_realtime_core import ControlMode
from realtime_ipc import validate_message
from run_realtime_controller import RealtimeController


def _active_controller():
    calibration = Path(__file__).resolve().parent / "motor_calibration.yaml"
    controller = RealtimeController(
        "can0",
        [1, 6, 12],
        50.0,
        True,
        ControlMode.AK70_GUI_PERSISTENT_LEASE,
        calibration,
    )
    token = "gain-session"
    arm = controller.core.arm(owner="test", session_token=token)
    for motor_id in (1, 6, 12):
        raw_zero = float(controller.core.calibration_entry(motor_id)["raw_zero_pos_rad"])
        controller.core.on_feedback(motor_id, raw_zero)
    controller.start_detected_motors(token, arm["session_epoch"])
    return controller, token, arm["session_epoch"]


def test_set_gains_changes_only_gain_fields_for_ak70_and_ak45():
    controller, token, _epoch = _active_controller()
    core = controller.core
    before_generation = core.target_generation
    before = {
        motor_id: (
            core.latest_targets[motor_id].position_rad,
            core.latest_targets[motor_id].target_deg,
            core.motors[motor_id].generation,
            core.motors[motor_id].trajectory_start_monotonic,
            core.motors[motor_id].move_duration_sec,
        )
        for motor_id in (1, 6, 12)
    }

    updated = core.set_gains(
        {
            1: {"kp": 13.0, "kd": 0.5},
            6: {"kp": 18.0, "kd": 0.7},
            12: {"kp": 18.0, "kd": 0.7},
        },
        token,
    )

    assert updated["0x001"] == {"kp": 13.0, "kd": 0.5}
    assert updated["0x006"] == {"kp": 18.0, "kd": 0.7}
    assert core.target_generation == before_generation
    for motor_id in (1, 6, 12):
        assert (
            core.latest_targets[motor_id].position_rad,
            core.latest_targets[motor_id].target_deg,
            core.motors[motor_id].generation,
            core.motors[motor_id].trajectory_start_monotonic,
            core.motors[motor_id].move_duration_sec,
        ) == before[motor_id]


def test_set_gains_ipc_accepts_model_limits_and_rejects_ak70_zero_kp():
    validate_message(json.dumps({
        "command": "SET_GAINS",
        "session_token": "token",
        "updates": {
            "0x001": {"kp": 100.0, "kd": 2.0},
            "0x006": {"kp": 500.0, "kd": 5.0},
        },
    }).encode())

    with pytest.raises(ValueError):
        validate_message(json.dumps({
            "command": "SET_GAINS",
            "session_token": "token",
            "updates": {"0x001": {"kp": 0.0, "kd": 0.4}},
        }).encode())


def test_controller_set_gains_updates_active_ready_subset_atomically():
    controller, token, epoch = _active_controller()
    response = controller.handle_message({
        "command": "SET_GAINS",
        "session_token": token,
        "session_epoch": epoch,
        "updates": {
            "0x001": {"kp": 20.0, "kd": 0.6},
            "0x006": {"kp": 25.0, "kd": 0.8},
        },
    })

    assert response["ok"] is True
    assert response["updated"] == {
        "0x001": {"kp": 20.0, "kd": 0.6},
        "0x006": {"kp": 25.0, "kd": 0.8},
    }


def test_stream_items_include_latest_model_gains_for_walk_and_isaac():
    snapshot = {1: 2.0, 6: -3.0, 7: 4.0, 12: -5.0}
    gains = {
        1: (15.0, 0.5),
        6: (30.0, 0.9),
        7: (15.0, 0.5),
        12: (30.0, 0.9),
    }

    items = gui.build_stream_target_items(snapshot, gains)

    by_id = {item["motor_id"]: item for item in items}
    assert by_id["0x001"]["kp"] == 15.0
    assert by_id["0x006"]["kp"] == 30.0
    assert by_id["0x00C"]["kd"] == 0.9


def test_model_gain_mapping_uses_ak45_only_for_ids_6_and_12():
    assert [motor_id for motor_id in range(1, 13) if gui.ID_TO_MODEL[motor_id] == "AK45"] == [6, 12]


class _ValueVar:
    def __init__(self):
        self.value = ""

    def set(self, value):
        self.value = value


def test_gui_gain_buttons_store_and_clamp_without_ready_motor_ipc():
    app = object.__new__(gui.ControlCenterApp)
    app.model_gains = {
        "AK70": {"kp": 8.0, "kd": 0.4},
        "AK45": {"kp": 8.0, "kd": 0.4},
    }
    app.gain_value_vars = {
        (model, gain): _ValueVar()
        for model in ("AK70", "AK45")
        for gain in ("kp", "kd")
    }
    app.session_token = None
    app._heartbeat_enabled = False
    app.ready_ids = set()
    app.log = lambda _message: None

    app.adjust_model_gain("AK70", "kp", -5.0)
    app.adjust_model_gain("AK70", "kp", -5.0)
    app.adjust_model_gain("AK45", "kd", -0.5)

    assert app.model_gains["AK70"]["kp"] == 0.1
    assert app.gain_value_vars[("AK70", "kp")].value == "0.1"
    assert app.model_gains["AK45"]["kd"] == 0.0


def test_gui_set_gains_sends_only_ready_motors_of_selected_model():
    app = object.__new__(gui.ControlCenterApp)
    app.model_gains = {
        "AK70": {"kp": 13.0, "kd": 0.5},
        "AK45": {"kp": 18.0, "kd": 0.7},
    }
    app.session_token = "gain-session"
    app.session_epoch = 2
    app._heartbeat_enabled = True
    app.ready_ids = {1, 6, 7, 12}
    app.gain_applied_values = {}
    app.log_ipc_error = lambda _command, _error: None
    queued = []

    def enqueue(payload, callback):
        queued.append(payload)
        callback({"ok": True})

    app.enqueue_ipc = enqueue
    app._apply_saved_gains_to_ready("AK45")

    assert len(queued) == 1
    assert queued[0]["command"] == "SET_GAINS"
    assert queued[0]["updates"] == {
        "0x006": {"kp": 18.0, "kd": 0.7},
        "0x00C": {"kp": 18.0, "kd": 0.7},
    }
    assert 1 not in app.gain_applied_values
    assert app.gain_applied_values[6] == (18.0, 0.7)
