"""Model-aware MIT packet helpers for mixed AK70-10 and AK45-36-KV80 control."""

from __future__ import annotations

import math
from dataclasses import dataclass

from mit_packet import float_to_uint, pack_mit_command as pack_ak70_mit_command, uint_to_float
from motor_profiles import MotorProfile, format_motor_id, get_motor_profile, normalize_motor_id


@dataclass(frozen=True)
class MitCommand:
    motor_id: int
    position: float
    velocity: float
    kp: float
    kd: float
    torque: float = 0.0


@dataclass(frozen=True)
class MitFeedback:
    motor_id: int
    position: float
    velocity: float
    torque: float
    temperature: int
    error_code: int


def _require_finite(value: float, label: str) -> None:
    if not math.isfinite(value):
        raise ValueError(f"{label} must be finite")


def _require_range(value: float, low: float, high: float, label: str) -> None:
    _require_finite(value, label)
    if not low <= value <= high:
        raise ValueError(f"{label}={value!r} outside range [{low}, {high}]")


def validate_command_values(profile: MotorProfile, position: float, velocity: float, kp: float, kd: float, torque: float) -> None:
    _require_range(position, profile.p_min, profile.p_max, "position")
    _require_range(velocity, profile.v_min, profile.v_max, "velocity")
    _require_range(kp, profile.kp_min, profile.kp_max, "kp")
    _require_range(kd, profile.kd_min, profile.kd_max, "kd")
    _require_range(torque, profile.protocol_torque_min, profile.protocol_torque_max, "protocol torque")


def pack_mit_command_for_profile(profile: MotorProfile, position: float, velocity: float, kp: float, kd: float, torque: float = 0.0) -> bytes:
    validate_command_values(profile, position, velocity, kp, kd, torque)
    if profile.model == "AK70-10":
        return pack_ak70_mit_command(position, velocity, kp, kd, torque)

    p_int = float_to_uint(position, profile.p_min, profile.p_max, 16)
    v_int = float_to_uint(velocity, profile.v_min, profile.v_max, 12)
    kp_int = float_to_uint(kp, profile.kp_min, profile.kp_max, 12)
    kd_int = float_to_uint(kd, profile.kd_min, profile.kd_max, 12)
    t_int = float_to_uint(torque, profile.protocol_torque_min, profile.protocol_torque_max, 12)
    return bytes(
        [
            p_int >> 8,
            p_int & 0xFF,
            v_int >> 4,
            ((v_int & 0x0F) << 4) | (kp_int >> 8),
            kp_int & 0xFF,
            kd_int >> 4,
            ((kd_int & 0x0F) << 4) | (t_int >> 8),
            t_int & 0xFF,
        ]
    )


def pack_mit_command(motor_id: int | str, position: float, velocity: float, kp: float, kd: float, torque: float = 0.0) -> bytes:
    profile = get_motor_profile(motor_id)
    return pack_mit_command_for_profile(profile, position, velocity, kp, kd, torque)


def pack_checked_commands(commands: list[MitCommand]) -> dict[int, bytes]:
    packets: dict[int, bytes] = {}
    seen: set[int] = set()
    for command in commands:
        motor_id = normalize_motor_id(command.motor_id)
        if motor_id in seen:
            raise ValueError(f"duplicate command motor ID: {format_motor_id(motor_id)}")
        seen.add(motor_id)
        profile = get_motor_profile(motor_id)
        packets[motor_id] = pack_mit_command_for_profile(
            profile,
            command.position,
            command.velocity,
            command.kp,
            command.kd,
            command.torque,
        )
    return {motor_id: packets[motor_id] for motor_id in sorted(packets)}


def unpack_mit_feedback(motor_id: int | str, packet_8bytes: bytes) -> MitFeedback:
    motor_id_int = normalize_motor_id(motor_id)
    profile = get_motor_profile(motor_id_int)
    if not isinstance(packet_8bytes, (bytes, bytearray)):
        raise ValueError("feedback packet must be bytes")
    if len(packet_8bytes) != 8:
        raise ValueError("MIT feedback packet must be exactly 8 bytes")
    data = bytes(packet_8bytes)
    if data[0] == 0xFF:
        raise ValueError("command echo frame is not feedback")
    if data[0] != (motor_id_int & 0xFF):
        raise ValueError(f"feedback payload ID mismatch for {format_motor_id(motor_id_int)}")

    p_int = (data[1] << 8) | data[2]
    v_int = (data[3] << 4) | (data[4] >> 4)
    t_int = ((data[4] & 0x0F) << 8) | data[5]
    return MitFeedback(
        motor_id=motor_id_int,
        position=uint_to_float(p_int, profile.p_min, profile.p_max, 16),
        velocity=uint_to_float(v_int, profile.v_min, profile.v_max, 12),
        torque=uint_to_float(t_int, profile.protocol_torque_min, profile.protocol_torque_max, 12),
        temperature=data[6],
        error_code=data[7],
    )

