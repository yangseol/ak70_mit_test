import pytest

from calibration import apply_software_offset, get_motor_offset_rad, load_motor_calibration


def test_motor_0x00a_offset_is_loaded_from_calibration():
    calibration = load_motor_calibration()

    assert get_motor_offset_rad(0x00A, calibration) == pytest.approx(0.523194)


def test_apply_software_offset_returns_zero_at_calibrated_position():
    calibration = load_motor_calibration()

    joint_position_rad = apply_software_offset(0.523194, 0x00A, calibration)

    assert joint_position_rad == pytest.approx(0.0)


def test_apply_software_offset_subtracts_calibrated_zero():
    calibration = load_motor_calibration()

    joint_position_rad = apply_software_offset(1.660, 0x00A, calibration)

    assert joint_position_rad == pytest.approx(1.136806)


def test_missing_motor_id_raises_key_error():
    calibration = load_motor_calibration()

    with pytest.raises(KeyError):
        apply_software_offset(0.0, 0x00B, calibration)
