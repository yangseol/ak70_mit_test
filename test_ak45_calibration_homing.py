import math

import pytest
import yaml

from ak45_calibration import (
    default_calibration,
    joint_to_raw_position,
    joint_to_raw_velocity,
    load_ak45_calibration,
    raw_to_joint_position,
    save_software_zero,
)
from homing_state import HomingMachine, HomingState, encoder_ambiguity_summary, sensor_window_does_not_home


def test_ak45_calibration_load_save_atomic(tmp_path):
    path = tmp_path / "ak45.yaml"
    data = default_calibration()
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    save_software_zero("0x00B", 1.25, path=path)
    loaded = load_ak45_calibration(path)
    assert loaded["motors"]["0x00B"]["raw_zero_pos_rad"] == pytest.approx(1.25)
    assert loaded["motors"]["0x00B"]["captured_at"]
    assert any(tmp_path.glob("ak45_*.yaml.bak"))


def test_direction_sign_transformations():
    assert raw_to_joint_position(1.5, 1.0, 1) == pytest.approx(0.5)
    assert raw_to_joint_position(1.5, 1.0, -1) == pytest.approx(-0.5)
    assert joint_to_raw_position(0.5, 1.0, -1) == pytest.approx(0.5)
    assert joint_to_raw_velocity(0.2, -1) == pytest.approx(-0.2)


def test_calibration_missing_blocks_home():
    calibration = default_calibration()
    machine = HomingMachine(0x00B)
    assert machine.state == HomingState.UNHOMED
    assert machine.confirm_horizontal_pose(0.0, calibration) == HomingState.HOMING_REQUIRED


def test_user_confirmation_transitions_to_homed_for_session_only():
    calibration = default_calibration()
    calibration["motors"]["0x00B"]["raw_zero_pos_rad"] = 1.0
    machine = HomingMachine(0x00B)
    assert machine.confirm_horizontal_pose(1.0, calibration) == HomingState.HOMED
    restarted = HomingMachine(0x00B)
    assert restarted.state == HomingState.UNHOMED


def test_bus_off_and_calibration_change_clear_homing():
    calibration = default_calibration()
    calibration["motors"]["0x00B"]["raw_zero_pos_rad"] = 1.0
    machine = HomingMachine(0x00B)
    machine.confirm_horizontal_pose(1.0, calibration)
    assert machine.on_bus_off() == HomingState.FAULT
    machine.confirm_horizontal_pose(1.0, calibration)
    assert machine.on_calibration_changed() == HomingState.HOMING_REQUIRED


def test_sensor_window_does_not_auto_home():
    assert sensor_window_does_not_home() is True
    summary = encoder_ambiguity_summary(0x00B)
    assert summary["encoder_output_period_deg"] == pytest.approx(10.0)
    assert summary["encoder_output_half_period_deg"] == pytest.approx(5.0)

