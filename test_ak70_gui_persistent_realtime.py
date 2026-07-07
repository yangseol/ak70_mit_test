import inspect
import json
import math
import time

import pytest

import ak70_control_center_gui as gui
from ak_realtime_core import (
    AK70_KP_MAX,
    AK70_KP_MIN,
    FEEDBACK_TIMEOUT_SEC,
    FOLLOWING_ERROR_CONSECUTIVE_LIMIT,
    FOLLOWING_ERROR_GRACE_SEC,
    GUI_LEASE_TIMEOUT_SEC,
    STOP_TIMEOUT_SEC,
    ControlMode,
    ControllerMode,
    FaultScope,
    MotorState,
    RealtimeCore,
    RealtimeTarget,
    TrajectoryWaypoint,
    joint_deg_to_raw_rad,
    quintic_smoothstep,
    targets_from_ipc,
)
from homing_state import HomingState
from realtime_ipc import validate_message
from run_realtime_controller import RealtimeController


def write_calibration(tmp_path):
    path = tmp_path / "motor_calibration.yaml"
    path.write_text(
        """
motors:
  "0x001":
    raw_zero_pos_rad: 1.0
    direction_sign: 1
  "0x002":
    raw_zero_pos_rad: 1.5
    direction_sign: 1
  "0x003":
    raw_zero_pos_rad: -0.5
    direction_sign: 1
""",
        encoding="utf-8",
    )
    return path


def arm_persistent_core(tmp_path):
    core = RealtimeCore([0x001, 0x002, 0x003], control_mode=ControlMode.AK70_GUI_PERSISTENT_LEASE, calibration_path=write_calibration(tmp_path))
    session = core.arm(owner="test", session_token="session-a", now=10.0)
    core.heartbeat("session-a", 1, generated_monotonic=10.0, now=10.0)
    return core, session


def feed(core, motor_id, raw_position, now=10.0):
    core.on_feedback(motor_id, raw_position, now=now)


def feed_many(core, values, now=10.0):
    for motor_id, raw_position in values.items():
        feed(core, motor_id, raw_position, now)


def gui_app_stub():
    app = object.__new__(gui.ControlCenterApp)
    app.ready_ids = {1, 6, 12}
    app.model_gains = {"AK70": {"kp": 8.0, "kd": 0.4}, "AK45": {"kp": 8.0, "kd": 0.4}}
    return app


def test_persistent_mode_keeps_target_after_default_stop_timeout(tmp_path):
    core, session = arm_persistent_core(tmp_path)
    feed(core, 0x001, 1.25, now=10.0)
    target = RealtimeTarget(
        0x001,
        joint_deg_to_raw_rad(0x001, 15.0, core.calibration),
        8.0,
        0.4,
        target_deg=15.0,
        session_token="session-a",
        session_epoch=session["session_epoch"],
    )
    core.set_latest_targets([target], now=10.0)
    feed(core, 0x001, 1.25, now=10.9)
    core.heartbeat("session-a", 2, generated_monotonic=10.9, now=10.9)
    commands = core.compute_cycle_commands(now=10.0 + STOP_TIMEOUT_SEC + 0.2)
    assert 0x001 in commands
    assert core.mode == ControllerMode.ARMED
    assert 0x001 in core.latest_targets


def test_default_mode_still_times_out():
    core = RealtimeCore([0x001])
    core.arm()
    core.set_latest_targets([RealtimeTarget(0x001, 0.1, 8.0, 0.4)], now=1.0)
    assert core.compute_cycle_commands(now=1.0 + STOP_TIMEOUT_SEC) == {}
    assert core.mode == ControllerMode.DISARMED


def test_first_target_requires_fresh_feedback_and_starts_from_feedback(tmp_path):
    core, session = arm_persistent_core(tmp_path)
    raw_target = joint_deg_to_raw_rad(0x001, 10.0, core.calibration)
    target = RealtimeTarget(
        0x001,
        raw_target,
        8.0,
        0.4,
        target_deg=10.0,
        move_sec=2.0,
        session_token="session-a",
        session_epoch=session["session_epoch"],
    )
    core.motors[0x001].command_position_rad = 0.0
    with pytest.raises(RuntimeError, match="FEEDBACK_REQUIRED"):
        core.set_latest_targets([target], now=10.0)
    feed(core, 0x001, 1.25, now=10.0)
    core.set_latest_targets([target], now=10.0)
    command = core.compute_cycle_commands(now=10.0)[0x001]
    assert command.position_rad == pytest.approx(1.25)
    assert core.motors[0x001].start_position_rad == pytest.approx(1.25)
    assert core.motors[0x001].command_position_rad != pytest.approx(0.0)


def test_previous_session_stale_commanded_position_is_not_used(tmp_path):
    core, session = arm_persistent_core(tmp_path)
    runtime = core.motors[0x001]
    runtime.command_initialized = True
    runtime.command_position_rad = 2.0
    runtime.owner_session_token = "old-session"
    runtime.state = MotorState.MOVING
    target = RealtimeTarget(
        0x001,
        joint_deg_to_raw_rad(0x001, 5.0, core.calibration),
        8.0,
        0.4,
        target_deg=5.0,
        session_token="session-a",
        session_epoch=session["session_epoch"],
    )
    with pytest.raises(RuntimeError, match="FEEDBACK_REQUIRED"):
        core.set_latest_targets([target], now=10.0)


def test_exact_move_sec_quintic_interpolation_and_hold(tmp_path):
    core, session = arm_persistent_core(tmp_path)
    start = 1.2
    target_raw = 1.8
    feed(core, 0x001, start, now=10.0)
    core.set_latest_targets(
        [
            RealtimeTarget(
                0x001,
                target_raw,
                8.0,
                0.4,
                target_deg=20.0,
                move_sec=2.0,
                session_token="session-a",
                session_epoch=session["session_epoch"],
            )
        ],
        now=10.0,
    )
    assert core.compute_cycle_commands(now=10.0)[0x001].position_rad == pytest.approx(start)
    core.heartbeat("session-a", 2, generated_monotonic=11.0, now=11.0)
    feed(core, 0x001, start, now=11.0)
    mid = core.compute_cycle_commands(now=11.0)[0x001].position_rad
    assert mid == pytest.approx(start + (target_raw - start) * quintic_smoothstep(0.5))
    core.heartbeat("session-a", 3, generated_monotonic=12.0, now=12.0)
    feed(core, 0x001, mid, now=12.0)
    assert core.compute_cycle_commands(now=12.0)[0x001].position_rad == pytest.approx(target_raw)
    core.heartbeat("session-a", 4, generated_monotonic=13.0, now=13.0)
    feed(core, 0x001, target_raw, now=13.0)
    assert core.compute_cycle_commands(now=13.0)[0x001].position_rad == pytest.approx(target_raw)


def test_multi_target_uses_same_normalized_time(tmp_path):
    core, session = arm_persistent_core(tmp_path)
    feed_many(core, {0x001: 1.0, 0x002: 1.5}, now=10.0)
    targets = [
        RealtimeTarget(0x001, 2.0, 8.0, 0.4, target_deg=30.0, move_sec=4.0, control_group_id="g", session_token="session-a", session_epoch=session["session_epoch"]),
        RealtimeTarget(0x002, 2.5, 8.0, 0.4, target_deg=30.0, move_sec=4.0, control_group_id="g", session_token="session-a", session_epoch=session["session_epoch"]),
    ]
    core.set_latest_targets(targets, now=10.0)
    core.heartbeat("session-a", 2, generated_monotonic=12.0, now=12.0)
    feed_many(core, {0x001: 1.0, 0x002: 1.5}, now=12.0)
    commands = core.compute_cycle_commands(now=12.0)
    assert commands[0x001].position_rad == pytest.approx(1.0 + (2.0 - 1.0) * quintic_smoothstep(0.5))
    assert commands[0x002].position_rad == pytest.approx(1.5 + (2.5 - 1.5) * quintic_smoothstep(0.5))


def test_feedback_timeout_faults_single_and_group(tmp_path):
    core, session = arm_persistent_core(tmp_path)
    feed_many(core, {0x001: 1.0, 0x002: 1.5}, now=10.0)
    targets = [
        RealtimeTarget(0x001, 1.1, 8.0, 0.4, target_deg=5.0, control_group_id="g", session_token="session-a", session_epoch=session["session_epoch"]),
        RealtimeTarget(0x002, 1.6, 8.0, 0.4, target_deg=5.0, control_group_id="g", session_token="session-a", session_epoch=session["session_epoch"]),
    ]
    core.set_latest_targets(targets, now=10.0)
    assert core.compute_cycle_commands(now=10.0 + FEEDBACK_TIMEOUT_SEC + 0.01) == {}
    events = core.consume_release_events()
    assert events[-1][1] == [0x001, 0x002]
    assert "g" in core.faulted_groups


def test_following_error_requires_consecutive_limit_and_faults_group(tmp_path):
    core, session = arm_persistent_core(tmp_path)
    feed_many(core, {0x001: 1.0, 0x002: 1.5}, now=10.0)
    targets = [
        RealtimeTarget(0x001, 2.0, 8.0, 0.4, target_deg=10.0, move_sec=1.0, max_following_error_deg=1.0, control_group_id="g", session_token="session-a", session_epoch=session["session_epoch"]),
        RealtimeTarget(0x002, 2.5, 8.0, 0.4, target_deg=10.0, move_sec=1.0, max_following_error_deg=1.0, control_group_id="g", session_token="session-a", session_epoch=session["session_epoch"]),
    ]
    core.set_latest_targets(targets, now=10.0)
    core.compute_cycle_commands(now=10.0 + FOLLOWING_ERROR_GRACE_SEC + 0.01)
    feed_many(core, {0x001: 1.0, 0x002: 1.5}, now=10.0 + FOLLOWING_ERROR_GRACE_SEC + 0.01)
    for i in range(FOLLOWING_ERROR_CONSECUTIVE_LIMIT - 1):
        assert core.compute_cycle_commands(now=10.3 + i * 0.01)
        feed_many(core, {0x001: 1.0, 0x002: 1.5}, now=10.3 + i * 0.01)
    core.compute_cycle_commands(now=10.5)
    assert core.latest_targets == {}
    assert core.consume_release_events()[-1][1] == [0x001, 0x002]


def test_trajectory_visits_each_waypoint_and_holds_last(tmp_path):
    core, session = arm_persistent_core(tmp_path)
    feed(core, 0x001, 1.0, now=10.0)
    wp1 = joint_deg_to_raw_rad(0x001, 10.0, core.calibration)
    wp2 = joint_deg_to_raw_rad(0x001, -5.0, core.calibration)
    core.set_trajectory(
        motor_id=0x001,
        waypoints=[TrajectoryWaypoint(10.0, 1.0), TrajectoryWaypoint(-5.0, 2.0)],
        kp=8.0,
        kd=0.4,
        max_following_error_deg=60.0,
        control_group_id="traj",
        plan_id="plan-a",
        session_token="session-a",
        session_epoch=session["session_epoch"],
        now=10.0,
    )
    assert core.compute_cycle_commands(now=10.0)[0x001].position_rad == pytest.approx(1.0)
    core.heartbeat("session-a", 2, generated_monotonic=11.0, now=11.0)
    feed(core, 0x001, 1.0, now=11.0)
    assert core.compute_cycle_commands(now=11.0)[0x001].position_rad == pytest.approx(wp1)
    core.heartbeat("session-a", 3, generated_monotonic=12.0, now=12.0)
    feed(core, 0x001, wp1, now=12.0)
    mid2 = core.compute_cycle_commands(now=12.0)[0x001].position_rad
    assert mid2 == pytest.approx(wp1 + (wp2 - wp1) * quintic_smoothstep(0.5))
    core.heartbeat("session-a", 4, generated_monotonic=13.0, now=13.0)
    feed(core, 0x001, mid2, now=13.0)
    assert core.compute_cycle_commands(now=13.0)[0x001].position_rad == pytest.approx(wp2)
    core.heartbeat("session-a", 5, generated_monotonic=14.0, now=14.0)
    feed(core, 0x001, wp2, now=14.0)
    assert core.compute_cycle_commands(now=14.0)[0x001].position_rad == pytest.approx(wp2)


def test_torque_ff_defaults_and_commands_follow_target(tmp_path):
    core, session = arm_persistent_core(tmp_path)
    assert RealtimeTarget(0x001, 0.1, 8.0, 0.4).torque_ff_nm == pytest.approx(0.0)
    feed(core, 0x001, 1.0, now=10.0)
    core.set_latest_targets(
        [
            RealtimeTarget(
                0x001,
                joint_deg_to_raw_rad(0x001, 5.0, core.calibration),
                8.0,
                0.4,
                target_deg=5.0,
                session_token="session-a",
                session_epoch=session["session_epoch"],
                torque_ff_nm=1.0,
            )
        ],
        now=10.0,
    )
    command = core.compute_cycle_commands(now=10.0)[0x001]
    assert command.torque_ff_nm == pytest.approx(1.0)
    assert core.status(now=10.0)["motors"]["0x001"]["torque_ff_nm"] == pytest.approx(1.0)


@pytest.mark.parametrize("value", [-25.0, 25.0])
def test_torque_ff_boundaries_allowed(tmp_path, value):
    core, session = arm_persistent_core(tmp_path)
    feed(core, 0x001, 1.0, now=10.0)
    core.set_latest_targets(
        [
            RealtimeTarget(
                0x001,
                1.1,
                8.0,
                0.4,
                target_deg=5.0,
                session_token="session-a",
                session_epoch=session["session_epoch"],
                torque_ff_nm=value,
            )
        ],
        now=10.0,
    )
    assert core.compute_cycle_commands(now=10.0)[0x001].torque_ff_nm == pytest.approx(value)


@pytest.mark.parametrize("value", [-25.1, 25.1, math.nan, math.inf, -math.inf])
def test_torque_ff_invalid_values_rejected(tmp_path, value):
    core, session = arm_persistent_core(tmp_path)
    feed(core, 0x001, 1.0, now=10.0)
    with pytest.raises(ValueError):
        core.set_latest_targets(
            [
                RealtimeTarget(
                    0x001,
                    1.1,
                    8.0,
                    0.4,
                    target_deg=5.0,
                    session_token="session-a",
                    session_epoch=session["session_epoch"],
                    torque_ff_nm=value,
                )
            ],
            now=10.0,
        )


def test_set_torque_ff_moving_preserves_motion_identity(tmp_path):
    core, session = arm_persistent_core(tmp_path)
    feed(core, 0x001, 1.0, now=10.0)
    core.set_latest_targets(
        [
            RealtimeTarget(
                0x001,
                1.5,
                8.0,
                0.4,
                target_deg=5.0,
                move_sec=2.0,
                control_group_id="g",
                plan_id="p",
                session_token="session-a",
                session_epoch=session["session_epoch"],
            )
        ],
        now=10.0,
    )
    runtime = core.motors[0x001]
    before = (
        runtime.target_position_rad,
        runtime.start_position_rad,
        runtime.trajectory_start_monotonic,
        runtime.move_duration_sec,
        runtime.plan_id,
        runtime.control_group_id,
        runtime.generation,
    )
    updated = core.set_torque_ff({"0x001": 3.0}, session_token="session-a")
    after = (
        runtime.target_position_rad,
        runtime.start_position_rad,
        runtime.trajectory_start_monotonic,
        runtime.move_duration_sec,
        runtime.plan_id,
        runtime.control_group_id,
        runtime.generation,
    )
    assert updated == {"0x001": 3.0}
    assert after == before
    assert core.compute_cycle_commands(now=10.5)[0x001].torque_ff_nm == pytest.approx(3.0)


def test_set_torque_ff_trajectory_preserves_plan_progress(tmp_path):
    core, session = arm_persistent_core(tmp_path)
    feed(core, 0x001, 1.0, now=10.0)
    core.set_trajectory(
        motor_id=0x001,
        waypoints=[TrajectoryWaypoint(10.0, 1.0), TrajectoryWaypoint(20.0, 1.0)],
        kp=8.0,
        kd=0.4,
        max_following_error_deg=60.0,
        control_group_id="traj",
        plan_id="plan-a",
        session_token="session-a",
        session_epoch=session["session_epoch"],
        now=10.0,
    )
    plan = core.plans["plan-a"]
    before = (
        plan.current_waypoint_index,
        plan.segment_start_monotonic,
        plan.segment_duration_sec,
        dict(plan.segment_start_position_rad),
        dict(plan.segment_target_position_rad),
        dict(plan.generation),
        core.motors[0x001].trajectory_start_monotonic,
        core.motors[0x001].generation,
    )
    core.set_torque_ff({"0x001": -2.0}, session_token="session-a")
    after = (
        plan.current_waypoint_index,
        plan.segment_start_monotonic,
        plan.segment_duration_sec,
        dict(plan.segment_start_position_rad),
        dict(plan.segment_target_position_rad),
        dict(plan.generation),
        core.motors[0x001].trajectory_start_monotonic,
        core.motors[0x001].generation,
    )
    assert after == before
    assert core.compute_cycle_commands(now=10.25)[0x001].torque_ff_nm == pytest.approx(-2.0)
    assert "plan-a" in core.plans


def test_set_torque_ff_atomic_rejects_bad_batch(tmp_path):
    core, session = arm_persistent_core(tmp_path)
    feed_many(core, {0x001: 1.0, 0x002: 1.5}, now=10.0)
    core.set_latest_targets(
        [
            RealtimeTarget(0x001, 1.1, 8.0, 0.4, target_deg=5.0, session_token="session-a", session_epoch=session["session_epoch"], torque_ff_nm=1.0),
            RealtimeTarget(0x002, 1.6, 8.0, 0.4, target_deg=5.0, session_token="session-a", session_epoch=session["session_epoch"], torque_ff_nm=2.0),
        ],
        now=10.0,
    )
    with pytest.raises(ValueError):
        core.set_torque_ff({"0x001": 3.0, "0x002": 26.0}, session_token="session-a")
    assert core.latest_targets[0x001].torque_ff_nm == pytest.approx(1.0)
    assert core.latest_targets[0x002].torque_ff_nm == pytest.approx(2.0)


def test_controller_set_torque_ff_ack_and_session_ownership(tmp_path):
    controller = RealtimeController("can0", [0x001], 50.0, True, ControlMode.AK70_GUI_PERSISTENT_LEASE, write_calibration(tmp_path))
    session = controller.core.arm(owner="test", session_token="session-a", now=10.0)
    feed(controller.core, 0x001, 1.0, now=10.0)
    controller.core.set_latest_targets(
        [
            RealtimeTarget(
                0x001,
                1.1,
                8.0,
                0.4,
                target_deg=5.0,
                session_token="session-a",
                session_epoch=session["session_epoch"],
            )
        ],
        now=10.0,
    )
    response = controller.handle_message({"command": "SET_TORQUE_FF", "session_token": "session-a", "updates": {"0x001": 4.0}})
    assert response["ok"] is True
    assert response["updated"] == {"0x001": 4.0}
    assert controller.core.latest_targets[0x001].torque_ff_nm == pytest.approx(4.0)
    with pytest.raises(RuntimeError):
        controller.handle_message({"command": "SET_TORQUE_FF", "session_token": "other", "updates": {"0x001": 5.0}})


def test_controller_mit_command_uses_latest_torque_ff(tmp_path, monkeypatch):
    controller = RealtimeController("can0", [0x001], 50.0, True, ControlMode.AK70_GUI_PERSISTENT_LEASE, write_calibration(tmp_path))
    now = time.monotonic()
    session = controller.core.arm(owner="test", session_token="session-a", now=now)
    feed(controller.core, 0x001, 1.0, now=now)
    controller.core.set_latest_targets(
        [
            RealtimeTarget(
                0x001,
                1.1,
                8.0,
                0.4,
                target_deg=5.0,
                session_token="session-a",
                session_epoch=session["session_epoch"],
                torque_ff_nm=-1.0,
            )
        ],
        now=now,
    )
    captured = []

    def fake_pack_checked_commands(commands):
        captured.extend(commands)
        return {0x001: b"\x00" * 8}

    class FakeBus:
        def send(self, _message):
            return None

    monkeypatch.setattr("run_realtime_controller.pack_checked_commands", fake_pack_checked_commands)
    controller.bus = FakeBus()
    controller.control_cycle()
    assert captured[-1].torque == pytest.approx(-1.0)


def test_release_invalidates_plan_and_prevents_reactivation(tmp_path):
    core, session = arm_persistent_core(tmp_path)
    feed(core, 0x001, 1.0, now=10.0)
    core.set_trajectory(
        motor_id=0x001,
        waypoints=[TrajectoryWaypoint(10.0, 1.0), TrajectoryWaypoint(20.0, 1.0)],
        kp=8.0,
        kd=0.4,
        max_following_error_deg=60.0,
        control_group_id="traj",
        plan_id="plan-a",
        session_token="session-a",
        session_epoch=session["session_epoch"],
        now=10.0,
    )
    core.release_ids(["0x001"], session_token="session-a")
    assert core.plans == {}
    assert core.compute_cycle_commands(now=11.0) == {}


def test_controller_release_sends_zero_torque_even_when_already_released(tmp_path):
    controller = RealtimeController("can0", [0x001], 50.0, True, ControlMode.AK70_GUI_PERSISTENT_LEASE, write_calibration(tmp_path))
    controller.core.arm(owner="test", session_token="session-a", now=10.0)
    response = controller.handle_message({"command": "RELEASE_IDS", "session_token": "session-a", "motor_ids": ["0x001"]})
    assert response["ok"] is True
    assert response["already_released_ids"] == ["0x001"]
    assert response["zero_torque_sent_ids"] == ["0x001"]
    assert response["zero_torque_failed_ids"] == []


def test_controller_exception_exit_snapshots_active_release(tmp_path):
    controller = RealtimeController("can0", [0x001], 50.0, True, ControlMode.AK70_GUI_PERSISTENT_LEASE, write_calibration(tmp_path))
    session = controller.core.arm(owner="test", session_token="session-a", now=10.0)
    feed(controller.core, 0x001, 1.0, now=10.0)
    controller.core.set_latest_targets(
        [
            RealtimeTarget(
                0x001,
                1.2,
                8.0,
                0.4,
                target_deg=5.0,
                session_token="session-a",
                session_epoch=session["session_epoch"],
            )
        ],
        now=10.0,
    )
    controller.release_active_on_exit("test exception")
    sent, failed = controller.drain_release_events()
    assert sent == ["0x001"]
    assert failed == []
    assert controller.core.latest_targets == {}


def test_heartbeat_duplicate_and_stale_do_not_refresh_lease(tmp_path):
    core, _session = arm_persistent_core(tmp_path)
    accepted = core.heartbeat("session-a", 2, generated_monotonic=10.1, now=10.1)
    duplicate = core.heartbeat("session-a", 2, generated_monotonic=10.2, now=10.2)
    stale = core.heartbeat("session-a", 3, generated_monotonic=10.0, now=11.0)
    assert accepted["heartbeat_accepted"] is True
    assert duplicate["heartbeat_accepted"] is False
    assert duplicate["reason"] == "duplicate_or_old_seq"
    assert stale["heartbeat_accepted"] is False
    assert stale["reason"] == "stale_heartbeat"


def test_lease_loss_releases_all_active_and_disarms(tmp_path):
    core, session = arm_persistent_core(tmp_path)
    feed(core, 0x001, 1.1, now=10.0)
    target = RealtimeTarget(
        0x001,
        0.1,
        8.0,
        0.4,
        target_deg=5.0,
        session_token="session-a",
        session_epoch=session["session_epoch"],
    )
    core.set_latest_targets([target], now=10.0)
    assert core.compute_cycle_commands(now=10.0 + GUI_LEASE_TIMEOUT_SEC + 0.01) == {}
    assert core.mode == ControllerMode.DISARMED
    assert core.latest_targets == {}
    events = core.consume_release_events()
    assert events[-1][0].value == "lease loss"
    assert events[-1][1] == [0x001]


def test_release_ids_is_idempotent_and_increments_generation(tmp_path):
    core, session = arm_persistent_core(tmp_path)
    feed(core, 0x001, 1.1, now=10.0)
    core.set_latest_targets(
        [
            RealtimeTarget(
                0x001,
                0.1,
                8.0,
                0.4,
                target_deg=5.0,
                session_token="session-a",
                session_epoch=session["session_epoch"],
            )
        ],
        now=10.0,
    )
    before = core.motors[0x001].generation
    first = core.release_ids(["0x001"], session_token="session-a")
    second = core.release_ids(["0x001"], session_token="session-a")
    assert first["released_ids"] == ["0x001"]
    assert second["already_released_ids"] == ["0x001"]
    assert core.motors[0x001].generation == before + 2
    assert 0x001 not in core.latest_targets


def test_group_fault_releases_whole_group(tmp_path):
    core, session = arm_persistent_core(tmp_path)
    feed_many(core, {0x001: 1.1, 0x002: 1.6}, now=10.0)
    group = "pose-a"
    targets = [
        RealtimeTarget(mid, 0.1, 8.0, 0.4, target_deg=5.0, control_group_id=group, session_token="session-a", session_epoch=session["session_epoch"])
        for mid in (0x001, 0x002)
    ]
    core.set_latest_targets(targets, now=10.0)
    affected = core.apply_group_fault(group, "following error", FaultScope.GROUP)
    assert affected == [0x001, 0x002]
    assert core.latest_targets == {}
    assert group in core.faulted_groups


def test_ak70_target_limits_allow_boundaries_and_reject_outside(tmp_path):
    core, session = arm_persistent_core(tmp_path)
    feed(core, 0x001, 1.0, now=10.0)
    for target_deg in (-120.0, 120.0):
        target = RealtimeTarget(
            0x001,
            joint_deg_to_raw_rad(0x001, target_deg, core.calibration),
            8.0,
            0.4,
            target_deg=target_deg,
            session_token="session-a",
            session_epoch=session["session_epoch"],
        )
        core.set_latest_targets([target], now=10.0)
    with pytest.raises(ValueError):
        targets_from_ipc(
            {
                "command": "SET_TARGETS",
                "session_token": "session-a",
                "session_epoch": session["session_epoch"],
                "targets": [{"motor_id": "0x001", "position_deg": 120.1, "kp": 8.0, "kd": 0.4}],
            },
            core,
        )[0] and core.set_latest_targets(
            targets_from_ipc(
                {
                    "command": "SET_TARGETS",
                    "session_token": "session-a",
                    "session_epoch": session["session_epoch"],
                    "targets": [{"motor_id": "0x001", "position_deg": 120.1, "kp": 8.0, "kd": 0.4}],
                },
                core,
            )
        )


@pytest.mark.parametrize(
    ("kp", "allowed"),
    [
        (0.0, False),
        (-1.0, False),
        (10.0, True),
        (30.0, True),
        (50.0, True),
        (80.0, True),
        (100.0, True),
        (100.1, False),
        (101.0, False),
    ],
)
def test_ak70_realtime_kp_limit_matrix(tmp_path, kp, allowed):
    core, session = arm_persistent_core(tmp_path)
    feed(core, 0x001, 1.0, now=10.0)
    target = RealtimeTarget(
        0x001,
        joint_deg_to_raw_rad(0x001, 10.0, core.calibration),
        kp,
        0.4,
        target_deg=10.0,
        session_token="session-a",
        session_epoch=session["session_epoch"],
    )
    if allowed:
        core.set_latest_targets([target], now=10.0)
        assert core.latest_targets[0x001].kp == pytest.approx(kp)
    else:
        with pytest.raises(ValueError):
            core.set_latest_targets([target], now=10.0)


def test_ak70_single_home_trajectory_and_batch_allow_kp_100(tmp_path):
    core, session = arm_persistent_core(tmp_path)
    feed_many(core, {0x001: 1.0, 0x002: 1.5}, now=10.0)
    single = RealtimeTarget(
        0x001,
        joint_deg_to_raw_rad(0x001, 10.0, core.calibration),
        AK70_KP_MAX,
        0.4,
        target_deg=10.0,
        session_token="session-a",
        session_epoch=session["session_epoch"],
    )
    core.set_latest_targets([single], now=10.0)
    assert core.latest_targets[0x001].kp == pytest.approx(AK70_KP_MAX)

    home = RealtimeTarget(
        0x001,
        joint_deg_to_raw_rad(0x001, 0.0, core.calibration),
        AK70_KP_MAX,
        0.4,
        target_deg=0.0,
        display_mode="home",
        session_token="session-a",
        session_epoch=session["session_epoch"],
    )
    core.set_latest_targets([home], now=10.1)
    assert core.latest_targets[0x001].display_mode == "home"
    assert core.latest_targets[0x001].kp == pytest.approx(AK70_KP_MAX)

    core.set_trajectory(
        motor_id=0x001,
        waypoints=[TrajectoryWaypoint(5.0, 1.0), TrajectoryWaypoint(0.0, 1.0)],
        kp=AK70_KP_MAX,
        kd=0.4,
        max_following_error_deg=60.0,
        control_group_id="traj",
        plan_id="kp100-traj",
        session_token="session-a",
        session_epoch=session["session_epoch"],
        now=10.2,
    )
    assert core.plans["kp100-traj"].kp[0x001] == pytest.approx(AK70_KP_MAX)

    batch = [
        RealtimeTarget(
            0x001,
            joint_deg_to_raw_rad(0x001, 3.0, core.calibration),
            AK70_KP_MAX,
            0.4,
            target_deg=3.0,
            control_group_id="batch",
            session_token="session-a",
            session_epoch=session["session_epoch"],
        ),
        RealtimeTarget(
            0x002,
            joint_deg_to_raw_rad(0x002, -3.0, core.calibration),
            AK70_KP_MAX,
            0.4,
            target_deg=-3.0,
            control_group_id="batch",
            session_token="session-a",
            session_epoch=session["session_epoch"],
        ),
    ]
    core.set_latest_targets(batch, now=10.3)
    assert {target.kp for target in core.latest_targets.values()} == {AK70_KP_MAX}


def test_ipc_ak70_kp_100_allowed_and_over_limit_rejected():
    base = {
        "command": "SET_TARGETS",
        "targets": [{"motor_id": "0x001", "position_deg": 1.0, "kp": AK70_KP_MAX, "kd": 0.4}],
    }
    validate_message(json.dumps(base).encode())
    for bad_kp in (AK70_KP_MIN, -1.0, AK70_KP_MAX + 0.1, 101.0):
        message = {
            "command": "SET_TARGETS",
            "targets": [{"motor_id": "0x001", "position_deg": 1.0, "kp": bad_kp, "kd": 0.4}],
        }
        with pytest.raises(ValueError):
            validate_message(json.dumps(message).encode())
    validate_message(
        json.dumps(
            {
                "command": "SET_TRAJECTORY",
                "motor_id": "0x001",
                "waypoints": [{"target_deg": 1.0, "duration": 0.5}],
                "kp": AK70_KP_MAX,
                "kd": 0.4,
            }
        ).encode()
    )
    with pytest.raises(ValueError):
        validate_message(
            json.dumps(
                {
                    "command": "SET_TRAJECTORY",
                    "motor_id": "0x001",
                    "waypoints": [{"target_deg": 1.0, "duration": 0.5}],
                    "kp": 101.0,
                    "kd": 0.4,
                }
            ).encode()
        )


def test_controller_accepts_ak70_kp_100_and_rejects_over_limit(tmp_path):
    controller = RealtimeController("can0", [0x001], 50.0, True, ControlMode.AK70_GUI_PERSISTENT_LEASE, write_calibration(tmp_path))
    session = controller.core.arm(owner="test", session_token="session-a", now=10.0)
    feed(controller.core, 0x001, 1.0, now=10.0)
    ok = controller.handle_message(
        {
            "command": "SET_TARGETS",
            "session_token": "session-a",
            "session_epoch": session["session_epoch"],
            "targets": [{"motor_id": "0x001", "position_deg": 1.0, "kp": AK70_KP_MAX, "kd": 0.4}],
        }
    )
    assert ok["ok"] is True
    with pytest.raises(ValueError):
        controller.handle_message(
            {
                "command": "SET_TARGETS",
                "session_token": "session-a",
                "session_epoch": session["session_epoch"],
                "targets": [{"motor_id": "0x001", "position_deg": 1.0, "kp": 101.0, "kd": 0.4}],
            }
        )


def test_gui_gain_limits_match_current_motor_models():
    assert gui.gain_limits("AK70", "kp") == (0.1, AK70_KP_MAX)
    assert gui.gain_limits("AK70", "kd") == (0.0, 2.0)
    assert gui.gain_limits("AK45", "kp") == (0.0, 500.0)
    assert gui.gain_limits("AK45", "kd") == (0.0, 5.0)


def test_gui_stream_items_can_carry_model_gains_without_torque_ff_ui():
    app = gui_app_stub()
    items = gui.build_stream_target_items(
        {1: 5.0, 6: -1.0, 12: 1.0},
        {motor_id: app._gain_for_motor(motor_id) for motor_id in (1, 6, 12)},
    )

    by_id = {item["motor_id"]: item for item in items}
    assert by_id["0x001"]["kp"] == pytest.approx(8.0)
    assert by_id["0x006"]["kp"] == pytest.approx(8.0)
    assert "torque_ff_nm" not in by_id["0x001"]


def test_ak45_core_profile_kp_validation_regression():
    core = RealtimeCore([0x006])
    core.arm()
    core.motors[0x006].homing.state = HomingState.HOMED
    core.set_latest_targets([RealtimeTarget(0x006, 0.0, 100.0, 0.4)], now=10.0)
    assert core.latest_targets[0x006].kp == pytest.approx(100.0)


def test_ipc_accepts_new_commands():
    validate_message(
        json.dumps(
            {
                "command": "HEARTBEAT",
                "request_id": "r1",
                "session_token": "s",
                "heartbeat_seq": 1,
                "generated_monotonic": 1.0,
            }
        ).encode()
    )
    validate_message(json.dumps({"command": "RELEASE_IDS", "request_id": "r2", "session_token": "s", "motor_ids": ["0x001"]}).encode())
    validate_message(
        json.dumps(
            {
                "command": "SET_TRAJECTORY",
                "request_id": "r3",
                "session_token": "s",
                "session_epoch": 1,
                "motor_id": "0x001",
                "waypoints": [{"target_deg": 1.0, "duration": 0.5}],
            }
        ).encode()
    )
    validate_message(
        json.dumps(
            {
                "command": "SET_TORQUE_FF",
                "request_id": "r4",
                "session_token": "s",
                "updates": {"0x001": 1.0, "0x003": -1.0},
            }
        ).encode()
    )


@pytest.mark.parametrize("value", [-25.1, 25.1, math.nan, math.inf, -math.inf])
def test_ipc_rejects_invalid_torque_ff(value):
    with pytest.raises(ValueError):
        validate_message(
            json.dumps(
                {
                    "command": "SET_TORQUE_FF",
                    "request_id": "r",
                    "session_token": "s",
                    "updates": {"0x001": value},
                },
                allow_nan=True,
            ).encode()
        )
    with pytest.raises(ValueError):
        validate_message(
            json.dumps(
                {
                    "command": "SET_TARGETS",
                    "targets": [{"motor_id": "0x001", "position_deg": 1.0, "kp": 8.0, "kd": 0.4, "torque_ff_nm": value}],
                },
                allow_nan=True,
            ).encode()
        )


def test_gui_current_control_center_does_not_spawn_legacy_finite_scripts():
    source = inspect.getsource(gui.ControlCenterApp)
    for text in ("script_path(\"move_once\")", "script_path(\"hold\")", "script_path(\"trajectory\")", "script_path(\"multi_pose\")"):
        assert text not in source


def test_close_path_releases_or_shutdowns_before_destroy():
    on_close = inspect.getsource(gui.ControlCenterApp.on_close)
    finish = inspect.getsource(gui.ControlCenterApp.finish_close)
    assert "SHUTDOWN" in on_close
    assert "RELEASE_SESSION" in on_close
    assert "root.destroy" not in on_close
    assert "root.destroy" in finish
