# WARNING:
# - AK70-10 상수는 공식 매뉴얼로 반드시 재확인 필요.
# - 현재 단일 AK70-10 실험에서는 명령 arbitration ID와 응답 arbitration ID가 모두 0x00A로 관찰됨.
# - 과거 0x02E 피드백 ID 가정은 현재 실험 결과와 맞지 않으므로 확정값처럼 사용하지 않는다.
# - unpack_mit_feedback()의 피드백 패킷 구조는 실제 매뉴얼/로그로 추가 검증 필요.

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MotorLimit:
    p_min: float
    p_max: float
    v_min: float
    v_max: float
    kp_min: float
    kp_max: float
    kd_min: float
    kd_max: float
    t_min: float
    t_max: float


AK70_10_LIMIT = MotorLimit(
    p_min=-12.5,
    p_max=12.5,
    v_min=-45.0,
    v_max=45.0,
    kp_min=0.0,
    kp_max=500.0,
    kd_min=0.0,
    kd_max=5.0,
    t_min=-24.0,
    t_max=24.0,
)

MOTOR_COMMAND_ID = 0x00A
# Latest single-motor observation only: response arbitration ID matched 0x00A.
# Do not generalize this to all motor IDs until the manual/logs are verified.
OBSERVED_SINGLE_MOTOR_RESPONSE_ID = 0x00A


def _clamp(x: float, x_min: float, x_max: float) -> float:
    return min(max(x, x_min), x_max)


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    x_clamped = _clamp(x, x_min, x_max)
    max_int = (1 << bits) - 1

    # T-Motor 공식 예제의 반올림/버림 방식과 일치 여부는 추후 확인 필요
    normalized = (x_clamped - x_min) / (x_max - x_min)
    scaled = normalized * max_int
    x_int = int(scaled + 0.5)

    return min(max(x_int, 0), max_int)


def uint_to_float(x_int: int, x_min: float, x_max: float, bits: int) -> float:
    span = x_max - x_min
    max_int = (2**bits) - 1
    return float(x_int) * span / max_int + x_min


def quantization_error(x_min: float, x_max: float, bits: int) -> float:
    step = (x_max - x_min) / ((2**bits) - 1)
    max_error = step / 2 + 1e-9
    return max_error


def pack_mit_command(p: float, v: float, kp: float, kd: float, t: float) -> bytes:
    limit = AK70_10_LIMIT

    p_int = float_to_uint(p, limit.p_min, limit.p_max, 16)
    v_int = float_to_uint(v, limit.v_min, limit.v_max, 12)
    kp_int = float_to_uint(kp, limit.kp_min, limit.kp_max, 12)
    kd_int = float_to_uint(kd, limit.kd_min, limit.kd_max, 12)
    t_int = float_to_uint(t, limit.t_min, limit.t_max, 12)

    data = bytearray(8)
    data[0] = p_int >> 8
    data[1] = p_int & 0xFF
    data[2] = v_int >> 4
    data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8)
    data[4] = kp_int & 0xFF
    data[5] = kd_int >> 4
    data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8)
    data[7] = t_int & 0xFF
    return bytes(data)


def unpack_mit_command(packet_8bytes: bytes) -> dict[str, float]:
    """Decode a packed command for tests only.

    This function is NOT for decoding real motor feedback. It is a command
    packet test helper designed exclusively to verify the mathematical
    round-trip accuracy of pack_mit_command().
    """
    if len(packet_8bytes) != 8:
        raise ValueError("MIT command packet must be exactly 8 bytes")

    data = packet_8bytes
    p_int = (data[0] << 8) | data[1]
    v_int = (data[2] << 4) | (data[3] >> 4)
    kp_int = ((data[3] & 0xF) << 8) | data[4]
    kd_int = (data[5] << 4) | (data[6] >> 4)
    t_int = ((data[6] & 0xF) << 8) | data[7]

    limit = AK70_10_LIMIT
    return {
        "position": uint_to_float(p_int, limit.p_min, limit.p_max, 16),
        "velocity": uint_to_float(v_int, limit.v_min, limit.v_max, 12),
        "kp": uint_to_float(kp_int, limit.kp_min, limit.kp_max, 12),
        "kd": uint_to_float(kd_int, limit.kd_min, limit.kd_max, 12),
        "torque": uint_to_float(t_int, limit.t_min, limit.t_max, 12),
    }


def analyze_feedback_candidate(packet_8bytes: bytes) -> dict | None:
    """Analyze stored raw feedback bytes for offline research only.

    This helper applies a tentative MIT-style bit split to saved raw bytes. It
    is not a verified feedback decoder and must not be used in a real control
    loop.
    """
    if not isinstance(packet_8bytes, (bytes, bytearray)):
        raise ValueError("feedback candidate packet must be bytes or bytearray")
    if len(packet_8bytes) != 8:
        raise ValueError("feedback candidate packet must be exactly 8 bytes")
    if packet_8bytes[0] == 0xFF:
        return None

    data = packet_8bytes
    motor_id_candidate = data[0]
    p_uint_candidate = (data[1] << 8) | data[2]
    v_uint_candidate = (data[3] << 4) | (data[4] >> 4)
    t_uint_candidate = ((data[4] & 0x0F) << 8) | data[5]
    status_byte6_candidate = data[6]
    status_byte7_candidate = data[7]

    limit = AK70_10_LIMIT
    position_candidate = uint_to_float(p_uint_candidate, limit.p_min, limit.p_max, 16)
    velocity_candidate = uint_to_float(v_uint_candidate, limit.v_min, limit.v_max, 12)
    effort_candidate = uint_to_float(t_uint_candidate, limit.t_min, limit.t_max, 12)

    return {
        "motor_id": motor_id_candidate,
        "p_uint": p_uint_candidate,
        "v_uint": v_uint_candidate,
        "t_uint": t_uint_candidate,
        "status_byte6": status_byte6_candidate,
        "status_byte7": status_byte7_candidate,
        "candidate_position_rad": position_candidate,
        "candidate_velocity_rads": velocity_candidate,
        "candidate_effort_from_torque_limit": effort_candidate,
    }


def unpack_mit_feedback(packet_8bytes: bytes) -> dict[str, float | int]:
    """Decode an MIT feedback packet.

    피드백 패킷 구조 공식 확인 필요. AK70-10 실제 매뉴얼/로그로
    추가 검증하기 전까지 명령 패킷 unpack 검증과 섞어 사용하지 않는다.

    Latest raw feedback-like sample:
    00A#0A829D8007FFD600

    ECHO warning: can0 may receive locally echoed command frames such as
    FF FF FF FF FF FF FF FC. A future decoder must not mistake command/echo
    frames, especially frames with data[0] == 0xFF, for real motor feedback.
    """
    if len(packet_8bytes) != 8:
        raise ValueError("MIT feedback packet must be exactly 8 bytes")
    raise NotImplementedError("실제 candump 로그와 공식 매뉴얼로 피드백 구조 검증 후 구현할 것")
