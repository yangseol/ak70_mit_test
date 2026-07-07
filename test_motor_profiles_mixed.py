import math

import pytest

from mit_packet import pack_mit_command as pack_ak70_command
from mit_packet import float_to_uint
from mixed_mit_packet import MitCommand, pack_checked_commands, pack_mit_command, unpack_mit_feedback
from motor_profiles import (
    AK45_36_KV80_PROFILE,
    AK70_10_PROFILE,
    encoder_output_half_period_deg,
    encoder_output_period_deg,
    get_motor_profile,
)


def test_motor_profile_id_selection():
    assert get_motor_profile("0x001").model == "AK70-10"
    assert get_motor_profile("0x00A").model == "AK70-10"
    assert get_motor_profile("0x006").model == "AK45-36-KV80"
    assert get_motor_profile("0x00B").model == "AK70-10"
    assert get_motor_profile("0x00C").model == "AK45-36-KV80"
    with pytest.raises(ValueError):
        get_motor_profile("0x00D")
    with pytest.raises(ValueError):
        get_motor_profile("0x00E")


def test_ak45_profile_official_mit_ranges():
    profile = AK45_36_KV80_PROFILE
    assert (profile.p_min, profile.p_max) == (-12.5, 12.5)
    assert (profile.v_min, profile.v_max) == (-6.0, 6.0)
    assert (profile.kp_min, profile.kp_max) == (0.0, 500.0)
    assert (profile.kd_min, profile.kd_max) == (0.0, 5.0)
    assert (profile.protocol_torque_min, profile.protocol_torque_max) == (-34.0, 34.0)
    assert profile.rated_voltage == 24.0
    assert (profile.allowable_voltage_min, profile.allowable_voltage_max) == (16.0, 28.0)


def test_ak70_packet_regression_zero_torque():
    assert pack_mit_command("0x001", 0.0, 0.0, 0.0, 0.0, 0.0) == pack_ak70_command(0.0, 0.0, 0.0, 0.0, 0.0)


def test_ak45_pack_unpack_round_trip_uses_ak45_ranges():
    profile = AK45_36_KV80_PROFILE
    p_int = float_to_uint(1.25, profile.p_min, profile.p_max, 16)
    v_int = float_to_uint(-3.0, profile.v_min, profile.v_max, 12)
    t_int = float_to_uint(5.0, profile.protocol_torque_min, profile.protocol_torque_max, 12)
    feedback_like = bytes(
        [
                0x06,
            p_int >> 8,
            p_int & 0xFF,
            v_int >> 4,
            ((v_int & 0x0F) << 4) | (t_int >> 8),
            t_int & 0xFF,
            25,
            0,
        ]
    )
    decoded = unpack_mit_feedback("0x006", feedback_like)
    assert decoded.motor_id == 0x006
    assert decoded.position == pytest.approx(1.25, abs=0.001)
    assert decoded.velocity == pytest.approx(-3.0, abs=0.002)
    assert decoded.torque == pytest.approx(5.0, abs=0.02)


def test_strict_range_and_finite_checks():
    with pytest.raises(ValueError):
        pack_mit_command("0x006", 0.0, 45.0, 0.0, 0.0, 0.0)
    with pytest.raises(ValueError):
        pack_mit_command("0x006", math.nan, 0.0, 0.0, 0.0, 0.0)
    with pytest.raises(ValueError):
        pack_checked_commands([MitCommand(0x006, 0.0, 0.0, 0.0, 0.0), MitCommand(0x006, 0.0, 0.0, 0.0, 0.0)])


def test_encoder_output_periods():
    assert encoder_output_period_deg(AK45_36_KV80_PROFILE) == pytest.approx(10.0)
    assert encoder_output_half_period_deg(AK45_36_KV80_PROFILE) == pytest.approx(5.0)
    assert encoder_output_period_deg(AK70_10_PROFILE) == pytest.approx(36.0)
    assert encoder_output_half_period_deg(AK70_10_PROFILE) == pytest.approx(18.0)
