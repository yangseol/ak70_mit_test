"""AK45 software-zero calibration file helpers."""

from __future__ import annotations

import copy
import math
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from motor_profiles import format_motor_id, get_motor_profile, normalize_motor_id


DEFAULT_AK45_CALIBRATION_PATH = Path(__file__).resolve().parent / "ak45_motor_calibration.yaml"
AK45_IDS = (0x00B, 0x00C, 0x00D)

DEFAULT_AK45_CALIBRATION: dict[str, Any] = {
    "motors": {
        "0x00B": {
            "name": "left_ankle_roll",
            "model": "AK45-36-KV80",
            "raw_zero_pos_rad": None,
            "direction_sign": 1,
            "captured_at": None,
            "power_cycle_verified": False,
            "notes": "",
        },
        "0x00C": {
            "name": "right_ankle_roll",
            "model": "AK45-36-KV80",
            "raw_zero_pos_rad": None,
            "direction_sign": 1,
            "captured_at": None,
            "power_cycle_verified": False,
            "notes": "",
        },
        "0x00D": {
            "name": "spare_ak45",
            "model": "AK45-36-KV80",
            "raw_zero_pos_rad": None,
            "direction_sign": 1,
            "captured_at": None,
            "power_cycle_verified": False,
            "notes": "",
        },
    }
}


def utc_timestamp() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def _path(path: str | Path | None = None) -> Path:
    return Path(path) if path is not None else DEFAULT_AK45_CALIBRATION_PATH


def validate_ak45_id(motor_id: int | str) -> int:
    value = normalize_motor_id(motor_id)
    profile = get_motor_profile(value)
    if profile.model != "AK45-36-KV80":
        raise ValueError(f"{format_motor_id(value)} is not an AK45-36-KV80 ID")
    return value


def default_calibration() -> dict[str, Any]:
    return copy.deepcopy(DEFAULT_AK45_CALIBRATION)


def validate_calibration(data: dict[str, Any]) -> None:
    if not isinstance(data, dict) or not isinstance(data.get("motors"), dict):
        raise ValueError("invalid AK45 calibration: missing motors mapping")
    for motor_id in AK45_IDS:
        key = format_motor_id(motor_id)
        if key not in data["motors"]:
            raise ValueError(f"invalid AK45 calibration: missing {key}")
        entry = data["motors"][key]
        if not isinstance(entry, dict):
            raise ValueError(f"invalid AK45 calibration entry for {key}")
        if entry.get("model") != "AK45-36-KV80":
            raise ValueError(f"invalid AK45 model for {key}")
        raw_zero = entry.get("raw_zero_pos_rad")
        if raw_zero is not None and not math.isfinite(float(raw_zero)):
            raise ValueError(f"raw_zero_pos_rad for {key} must be finite or null")
        direction_sign = int(entry.get("direction_sign", 1))
        if direction_sign not in (-1, 1):
            raise ValueError(f"direction_sign for {key} must be 1 or -1")
        if not isinstance(entry.get("power_cycle_verified", False), bool):
            raise ValueError(f"power_cycle_verified for {key} must be boolean")


def load_ak45_calibration(path: str | Path | None = None) -> dict[str, Any]:
    calibration_path = _path(path)
    if not calibration_path.exists():
        data = default_calibration()
        validate_calibration(data)
        return data
    with calibration_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    validate_calibration(data)
    return data


def atomic_save_ak45_calibration(data: dict[str, Any], path: str | Path | None = None) -> None:
    validate_calibration(data)
    calibration_path = _path(path)
    calibration_path.parent.mkdir(parents=True, exist_ok=True)

    if calibration_path.exists():
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_path = calibration_path.with_name(f"{calibration_path.stem}_{timestamp}{calibration_path.suffix}.bak")
        backup_path.write_bytes(calibration_path.read_bytes())

    temp_path = calibration_path.with_name(f".{calibration_path.name}.tmp")
    with temp_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)
        f.flush()
        os.fsync(f.fileno())
    os.replace(temp_path, calibration_path)

    loaded = load_ak45_calibration(calibration_path)
    if loaded != data:
        raise RuntimeError("AK45 calibration verification failed after save")


def get_ak45_entry(motor_id: int | str, calibration: dict[str, Any] | None = None) -> dict[str, Any]:
    value = validate_ak45_id(motor_id)
    data = calibration if calibration is not None else load_ak45_calibration()
    key = format_motor_id(value)
    try:
        entry = data["motors"][key]
    except KeyError as exc:
        raise KeyError(f"AK45 calibration missing for {key}") from exc
    return entry


def calibration_exists(motor_id: int | str, calibration: dict[str, Any] | None = None) -> bool:
    entry = get_ak45_entry(motor_id, calibration)
    raw_zero = entry.get("raw_zero_pos_rad")
    return raw_zero is not None and math.isfinite(float(raw_zero))


def set_direction_sign(motor_id: int | str, direction_sign: int, path: str | Path | None = None) -> None:
    if direction_sign not in (-1, 1):
        raise ValueError("direction_sign must be 1 or -1")
    data = load_ak45_calibration(path)
    data["motors"][format_motor_id(validate_ak45_id(motor_id))]["direction_sign"] = direction_sign
    atomic_save_ak45_calibration(data, path)


def save_software_zero(
    motor_id: int | str,
    raw_pos_rad: float,
    path: str | Path | None = None,
    notes: str | None = None,
) -> None:
    value = validate_ak45_id(motor_id)
    if not math.isfinite(raw_pos_rad):
        raise ValueError("raw_pos_rad must be finite")
    data = load_ak45_calibration(path)
    entry = data["motors"][format_motor_id(value)]
    entry["raw_zero_pos_rad"] = float(raw_pos_rad)
    entry["captured_at"] = utc_timestamp()
    entry["power_cycle_verified"] = False
    if notes is not None:
        entry["notes"] = notes
    atomic_save_ak45_calibration(data, path)


def record_power_cycle_verified(motor_id: int | str, path: str | Path | None = None, verified: bool = True) -> None:
    data = load_ak45_calibration(path)
    entry = data["motors"][format_motor_id(validate_ak45_id(motor_id))]
    entry["power_cycle_verified"] = bool(verified)
    atomic_save_ak45_calibration(data, path)


def raw_to_joint_position(raw_pos_rad: float, raw_zero_pos_rad: float, direction_sign: int) -> float:
    if direction_sign not in (-1, 1):
        raise ValueError("direction_sign must be 1 or -1")
    if not math.isfinite(raw_pos_rad) or not math.isfinite(raw_zero_pos_rad):
        raise ValueError("position values must be finite")
    return direction_sign * (raw_pos_rad - raw_zero_pos_rad)


def joint_to_raw_position(joint_target_rad: float, raw_zero_pos_rad: float, direction_sign: int) -> float:
    if direction_sign not in (-1, 1):
        raise ValueError("direction_sign must be 1 or -1")
    if not math.isfinite(joint_target_rad) or not math.isfinite(raw_zero_pos_rad):
        raise ValueError("position values must be finite")
    return raw_zero_pos_rad + direction_sign * joint_target_rad


def joint_to_raw_velocity(joint_velocity_rad_s: float, direction_sign: int) -> float:
    if direction_sign not in (-1, 1):
        raise ValueError("direction_sign must be 1 or -1")
    if not math.isfinite(joint_velocity_rad_s):
        raise ValueError("velocity must be finite")
    return direction_sign * joint_velocity_rad_s


def joint_position_from_calibration(motor_id: int | str, raw_pos_rad: float, calibration: dict[str, Any] | None = None) -> float:
    entry = get_ak45_entry(motor_id, calibration)
    if entry.get("raw_zero_pos_rad") is None:
        raise ValueError(f"AK45 calibration missing for {format_motor_id(motor_id)}")
    return raw_to_joint_position(raw_pos_rad, float(entry["raw_zero_pos_rad"]), int(entry["direction_sign"]))

