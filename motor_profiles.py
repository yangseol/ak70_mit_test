"""Motor model profiles for mixed AK70/AK45 MIT control."""

from __future__ import annotations

from dataclasses import dataclass

from mit_packet import AK70_10_LIMIT

AK70_KP_MIN = 0.0
AK70_KP_MAX = 100.0
AK70_KD_MIN = 0.0
AK70_KD_MAX = 2.0


@dataclass(frozen=True)
class MotorProfile:
    model: str

    id_min: int
    id_max: int

    p_min: float
    p_max: float

    v_min: float
    v_max: float

    kp_min: float
    kp_max: float

    kd_min: float
    kd_max: float

    protocol_torque_min: float
    protocol_torque_max: float

    rated_voltage: float
    allowable_voltage_min: float
    allowable_voltage_max: float

    gear_ratio: float
    encoder_type: str
    has_output_encoder: bool

    default_kp: float
    default_kd: float

    bench_target_limit_deg: float
    bench_delta_limit_deg: float


# Bench limits are pre-assembly validation limits only. After ankle assembly,
# replace them with CAD and real collision based soft limits.
AK70_10_PROFILE = MotorProfile(
    model="AK70-10",
    id_min=0x001,
    id_max=0x00C,
    p_min=AK70_10_LIMIT.p_min,
    p_max=AK70_10_LIMIT.p_max,
    v_min=AK70_10_LIMIT.v_min,
    v_max=AK70_10_LIMIT.v_max,
    kp_min=AK70_10_LIMIT.kp_min,
    kp_max=AK70_10_LIMIT.kp_max,
    kd_min=AK70_10_LIMIT.kd_min,
    kd_max=AK70_10_LIMIT.kd_max,
    protocol_torque_min=AK70_10_LIMIT.t_min,
    protocol_torque_max=AK70_10_LIMIT.t_max,
    rated_voltage=48.0,
    allowable_voltage_min=24.0,
    allowable_voltage_max=48.0,
    gear_ratio=10.0,
    encoder_type="single-turn internal encoder",
    has_output_encoder=False,
    default_kp=8.0,
    default_kd=0.4,
    bench_target_limit_deg=120.0,
    bench_delta_limit_deg=120.0,
)

AK45_36_KV80_PROFILE = MotorProfile(
    model="AK45-36-KV80",
    id_min=0x006,
    id_max=0x00C,
    p_min=-12.5,
    p_max=12.5,
    v_min=-6.0,
    v_max=6.0,
    kp_min=0.0,
    kp_max=500.0,
    kd_min=0.0,
    kd_max=5.0,
    protocol_torque_min=-34.0,
    protocol_torque_max=34.0,
    rated_voltage=24.0,
    allowable_voltage_min=16.0,
    allowable_voltage_max=28.0,
    gear_ratio=36.0,
    encoder_type="14bit single-turn",
    has_output_encoder=False,
    default_kp=8.0,
    default_kd=0.4,
    bench_target_limit_deg=120.0,
    bench_delta_limit_deg=120.0,
)


def normalize_motor_id(motor_id: int | str) -> int:
    if isinstance(motor_id, int):
        value = motor_id
    elif isinstance(motor_id, str):
        text = motor_id.strip()
        if not text:
            raise ValueError("motor ID must not be empty")
        value = int(text, 0) if text.lower().startswith("0x") else int(text, 10)
    else:
        raise TypeError("motor ID must be int or str")

    if not 0 <= value <= 0x7FF:
        raise ValueError(f"motor ID out of 11-bit CAN range: {value!r}")
    return value


def format_motor_id(motor_id: int | str) -> str:
    return f"0x{normalize_motor_id(motor_id):03X}"


def get_motor_profile(motor_id: int | str) -> MotorProfile:
    value = normalize_motor_id(motor_id)
    # The lower-body wiring is fixed: only ankle-roll IDs 6 and 12 are AK45.
    # A range check is not sufficient because the two models share the same bus.
    if value in (0x006, 0x00C):
        return AK45_36_KV80_PROFILE
    if 0x001 <= value <= 0x00C:
        return AK70_10_PROFILE
    raise ValueError(f"unsupported motor ID: {format_motor_id(value)}")


def encoder_output_period_deg(profile: MotorProfile) -> float:
    return 360.0 / profile.gear_ratio


def encoder_output_half_period_deg(profile: MotorProfile) -> float:
    return encoder_output_period_deg(profile) / 2.0


def is_ak45(motor_id: int | str) -> bool:
    return get_motor_profile(motor_id).model == AK45_36_KV80_PROFILE.model
