import pytest

# 공식 예제 확보 전까지 의도적인 임의 골든 패킷(Golden Packet) 바이너리 테스트는 보류함.
from mit_packet import (
    AK70_10_LIMIT,
    analyze_feedback_candidate,
    float_to_uint,
    pack_mit_command,
    quantization_error,
    unpack_mit_command,
    unpack_mit_feedback,
    uint_to_float,
)


def clamp(x, x_min, x_max):
    return min(max(x, x_min), x_max)


def assert_close_quantized(expected, actual, x_min, x_max, bits):
    max_error = quantization_error(x_min, x_max, bits)
    assert abs(expected - actual) <= max_error


def assert_command_matches(expected):
    limit = AK70_10_LIMIT
    packet = pack_mit_command(
        p=expected["position"],
        v=expected["velocity"],
        kp=expected["kp"],
        kd=expected["kd"],
        t=expected["torque"],
    )
    unpacked = unpack_mit_command(packet)

    assert_close_quantized(expected["position"], unpacked["position"], limit.p_min, limit.p_max, 16)
    assert_close_quantized(expected["velocity"], unpacked["velocity"], limit.v_min, limit.v_max, 12)
    assert_close_quantized(expected["kp"], unpacked["kp"], limit.kp_min, limit.kp_max, 12)
    assert_close_quantized(expected["kd"], unpacked["kd"], limit.kd_min, limit.kd_max, 12)
    assert_close_quantized(expected["torque"], unpacked["torque"], limit.t_min, limit.t_max, 12)


def test_pack_mit_command_returns_bytes():
    packet = pack_mit_command(p=0.0, v=0.0, kp=0.0, kd=0.0, t=0.0)
    assert isinstance(packet, bytes)


def test_pack_mit_command_returns_exactly_8_bytes():
    packet = pack_mit_command(p=0.0, v=0.0, kp=0.0, kd=0.0, t=0.0)
    assert len(packet) == 8


def test_zero_torque_command_golden_packet_verification():
    limit = AK70_10_LIMIT
    packet = pack_mit_command(p=0.0, v=0.0, kp=0.0, kd=0.0, t=0.0)
    expected_bytes = bytes([0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x00])

    assert isinstance(packet, bytes)
    assert len(packet) == 8
    assert packet == expected_bytes

    unpacked = unpack_mit_command(packet)
    assert_close_quantized(0.0, unpacked["position"], limit.p_min, limit.p_max, 16)
    assert_close_quantized(0.0, unpacked["velocity"], limit.v_min, limit.v_max, 12)
    assert_close_quantized(0.0, unpacked["kp"], limit.kp_min, limit.kp_max, 12)
    assert_close_quantized(0.0, unpacked["kd"], limit.kd_min, limit.kd_max, 12)
    assert_close_quantized(0.0, unpacked["torque"], limit.t_min, limit.t_max, 12)


def test_min_values_round_trip_within_quantization_error():
    limit = AK70_10_LIMIT
    assert_command_matches(
        {
            "position": limit.p_min,
            "velocity": limit.v_min,
            "kp": limit.kp_min,
            "kd": limit.kd_min,
            "torque": limit.t_min,
        }
    )


def test_max_values_round_trip_within_quantization_error():
    limit = AK70_10_LIMIT
    assert_command_matches(
        {
            "position": limit.p_max,
            "velocity": limit.v_max,
            "kp": limit.kp_max,
            "kd": limit.kd_max,
            "torque": limit.t_max,
        }
    )


def test_zero_or_near_zero_values_round_trip_within_quantization_error():
    assert_command_matches(
        {
            "position": 0.0,
            "velocity": 0.0,
            "kp": 0.0,
            "kd": 0.001,
            "torque": 0.0,
        }
    )


def test_mid_values_round_trip_within_quantization_error():
    assert_command_matches(
        {
            "position": 3.25,
            "velocity": -12.75,
            "kp": 125.5,
            "kd": 2.5,
            "torque": 8.25,
        }
    )


def test_out_of_range_inputs_are_clamped_before_packing():
    limit = AK70_10_LIMIT
    packet = pack_mit_command(p=999.0, v=-999.0, kp=999.0, kd=-999.0, t=999.0)
    unpacked = unpack_mit_command(packet)

    assert_close_quantized(limit.p_max, unpacked["position"], limit.p_min, limit.p_max, 16)
    assert_close_quantized(limit.v_min, unpacked["velocity"], limit.v_min, limit.v_max, 12)
    assert_close_quantized(limit.kp_max, unpacked["kp"], limit.kp_min, limit.kp_max, 12)
    assert_close_quantized(limit.kd_min, unpacked["kd"], limit.kd_min, limit.kd_max, 12)
    assert_close_quantized(limit.t_max, unpacked["torque"], limit.t_min, limit.t_max, 12)


@pytest.mark.parametrize(
    ("x", "x_min", "x_max", "bits"),
    [
        (-12.5, -12.5, 12.5, 16),
        (12.5, -12.5, 12.5, 16),
        (0.0, -45.0, 45.0, 12),
        (250.0, 0.0, 500.0, 12),
        (-999.0, -24.0, 24.0, 12),
        (999.0, -24.0, 24.0, 12),
    ],
)
def test_float_uint_round_trip_within_quantization_error(x, x_min, x_max, bits):
    x_clamped = clamp(x, x_min, x_max)
    x_int = float_to_uint(x, x_min, x_max, bits)
    restored = uint_to_float(x_int, x_min, x_max, bits)

    max_int = (2**bits) - 1
    assert 0 <= x_int <= max_int
    assert_close_quantized(x_clamped, restored, x_min, x_max, bits)


def test_unpack_mit_command_rejects_non_8_byte_packet():
    with pytest.raises(ValueError):
        unpack_mit_command(b"\x00" * 7)


def test_unpack_mit_feedback_is_not_implemented_until_verified():
    with pytest.raises(NotImplementedError) as excinfo:
        unpack_mit_feedback(b"\x00" * 8)

    assert "실제 candump 로그" in str(excinfo.value)


def test_analyze_feedback_candidate_splits_observed_raw_sample():
    raw_sample = bytes([0x0A, 0x82, 0x9D, 0x80, 0x07, 0xFF, 0xD6, 0x00])

    res = analyze_feedback_candidate(raw_sample)

    assert res is not None
    assert res["motor_id"] == 0x0A
    assert res["p_uint"] == 0x829D
    assert res["v_uint"] == 0x800
    assert res["t_uint"] == 0x7FF
    assert res["status_byte6"] == 0xD6
    assert res["status_byte7"] == 0x00
    assert "candidate_position_rad" in res
    assert "candidate_velocity_rads" in res
    assert "candidate_effort_from_torque_limit" in res


def test_analyze_feedback_candidate_returns_none_for_echo_command_frame():
    echo_cmd = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])

    assert analyze_feedback_candidate(echo_cmd) is None


def test_analyze_feedback_candidate_rejects_invalid_input():
    with pytest.raises(ValueError):
        analyze_feedback_candidate(b"\x00" * 7)

    with pytest.raises(ValueError):
        analyze_feedback_candidate([0x00] * 8)
