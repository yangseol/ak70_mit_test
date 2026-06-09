import pytest

from calibration import (
    apply_software_offset,
    get_motor_offset_rad,
    load_motor_calibration,
)


def test_get_motor_offset_rad_reads_mocked_calibration():
    calibration = {
        "motors": {
            "0x00A": {
                "raw_zero_pos_rad": 0.523194,
            }
        }
    }

    assert get_motor_offset_rad(0x00A, calibration) == pytest.approx(0.523194)


def test_apply_software_offset_returns_zero_at_calibrated_position():
    calibration = {
        "motors": {
            "0x00A": {
                "raw_zero_pos_rad": 0.523194,
            }
        }
    }

    joint_position_rad = apply_software_offset(0.523194, 0x00A, calibration)

    assert joint_position_rad == pytest.approx(0.0)


def test_apply_software_offset_subtracts_calibrated_zero():
    calibration = {
        "motors": {
            "0x00A": {
                "raw_zero_pos_rad": 0.523194,
            }
        }
    }

    joint_position_rad = apply_software_offset(1.660, 0x00A, calibration)

    assert joint_position_rad == pytest.approx(1.136806)


def test_missing_motor_id_raises_key_error():
    calibration = {
        "motors": {
            "0x00A": {
                "raw_zero_pos_rad": 0.523194,
            }
        }
    }

    with pytest.raises(KeyError):
        get_motor_offset_rad(0x001, calibration)


def test_motor_calibration_yaml_has_valid_motor_entries():
    calibration = load_motor_calibration()

    assert "motors" in calibration
    assert isinstance(calibration["motors"], dict)

    for required_motor_id in ("0x00A", "0x007", "0x005"):
        assert required_motor_id in calibration["motors"]

    for motor_id, motor_data in calibration["motors"].items():
        assert motor_id.startswith("0x")
        assert "raw_zero_p_uint" in motor_data
        assert "raw_zero_pos_rad" in motor_data
        assert "zero_command_persistent" in motor_data
        assert "direction_sign" in motor_data

        assert isinstance(motor_data["raw_zero_p_uint"], str)
        assert isinstance(motor_data["raw_zero_pos_rad"], float)
        assert isinstance(motor_data["zero_command_persistent"], bool)
        assert motor_data["direction_sign"] in (-1, 1)
