import json
from pathlib import Path

import pytest

import realtime_ipc
from ak_realtime_core import ControllerMode, RealtimeCore, RealtimeTarget, STOP_TIMEOUT_SEC
from realtime_ipc import (
    ControllerAlreadyRunning,
    acquire_controller_lock,
    bind_controller_socket,
    validate_message,
)
from run_mixed_joint_pose import parse_targets, run_once, validate_static_inputs


def test_ak45_power_verified_blocks_actual_not_dry_run():
    targets = parse_targets(["0x00B,0"], "deg")
    errors = validate_static_inputs(targets, [], 8.0, 0.4, 2.0, 0.0, 50.0, 5.0, 60.0, False, False, False)
    assert any("--ak45-power-verified" in error for error in errors)
    dry_errors = validate_static_inputs(targets, [], 8.0, 0.4, 2.0, 0.0, 50.0, 5.0, 60.0, False, False, True)
    assert not any("--ak45-power-verified" in error for error in dry_errors)


def test_home_to_zero_requires_exact_zero():
    targets = parse_targets(["0x001,1"], "deg")
    errors = validate_static_inputs(targets, [], 8.0, 0.4, 2.0, 0.0, 50.0, 5.0, 60.0, True, False, True)
    assert any("exactly 0" in error for error in errors)


def test_mixed_dry_run_no_can_open_for_calibrated_ak70_only():
    targets = parse_targets(["0x001,0"], "deg")
    assert run_once("can0", targets, [], 8.0, 0.4, 2.0, 0.0, 50.0, 5.0, 60.0, False, False, False, True) == 0


def test_latest_target_only_and_watchdog():
    core = RealtimeCore([0x001], rate_hz=50.0)
    core.arm()
    core.set_latest_targets([RealtimeTarget(0x001, 1.0, 8.0, 0.4)], now=10.0)
    first_generation = core.target_generation
    core.set_latest_targets([RealtimeTarget(0x001, 2.0, 8.0, 0.4)], now=10.1)
    assert core.target_generation == first_generation + 1
    assert core.latest_targets[0x001].position_rad == 2.0
    assert core.compute_cycle_commands(now=10.2)
    assert core.compute_cycle_commands(now=10.1 + STOP_TIMEOUT_SEC) == {}
    assert core.mode == ControllerMode.DISARMED
    assert core.requires_rearm is True


def test_ak45_unhomed_blocks_realtime_target():
    core = RealtimeCore([0x00B])
    core.arm()
    with pytest.raises(RuntimeError):
        core.set_latest_targets([RealtimeTarget(0x00B, 0.0, 8.0, 0.4)])


def test_bus_off_does_not_auto_resume():
    core = RealtimeCore([0x001, 0x00B])
    core.arm()
    core.on_bus_off_or_reconnect()
    assert core.mode == ControllerMode.FAULT
    assert core.latest_targets == {}
    assert core.requires_rearm is True
    assert core.status()["motors"]["0x00B"]["homing"] == "FAULT"


def test_ipc_message_validation():
    validate_message(json.dumps({"command": "PING"}).encode())
    validate_message(json.dumps({"command": "SET_TARGETS", "targets": [{"motor_id": "0x001", "position_deg": 1.0}]}).encode())
    with pytest.raises(ValueError):
        validate_message(b"{bad json")
    with pytest.raises(ValueError):
        validate_message(json.dumps({"command": "NOPE"}).encode())
    with pytest.raises(ValueError):
        validate_message(json.dumps({"command": "SET_TARGETS", "targets": [{"motor_id": "0x001", "position_deg": 1.0}, {"motor_id": "0x001", "position_deg": 2.0}]}).encode())


def test_flock_duplicate_and_persistent_file(tmp_path):
    lock_path = tmp_path / "controller.lock"
    first = acquire_controller_lock(lock_path)
    assert lock_path.exists()
    with pytest.raises(ControllerAlreadyRunning):
        acquire_controller_lock(lock_path)
    first.close()
    second = acquire_controller_lock(lock_path)
    second.close()
    assert lock_path.exists()


class FakeDatagramSocket:
    def bind(self, path):
        self.path = Path(path)
        self.path.write_text("fake socket", encoding="utf-8")

    def close(self):
        pass


def test_stale_socket_cleanup_and_own_inode_cleanup(tmp_path, monkeypatch):
    socket_path = tmp_path / "controller.sock"
    socket_path.write_text("stale", encoding="utf-8")
    monkeypatch.setattr(realtime_ipc, "probe_datagram_socket", lambda path: False)
    monkeypatch.setattr(realtime_ipc.stat, "S_ISSOCK", lambda mode: True)
    monkeypatch.setattr(realtime_ipc.socket, "socket", lambda *args, **kwargs: FakeDatagramSocket())
    bound = bind_controller_socket(socket_path)
    st = socket_path.stat()
    assert st.st_ino == bound.st_ino
    bound.close_and_cleanup()
    assert not socket_path.exists()


def test_live_socket_protected(tmp_path, monkeypatch):
    socket_path = tmp_path / "live.sock"
    socket_path.write_text("live", encoding="utf-8")
    monkeypatch.setattr(realtime_ipc, "probe_datagram_socket", lambda path: True)
    with pytest.raises(Exception):
        bind_controller_socket(socket_path)
