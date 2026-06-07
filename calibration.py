from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any


DEFAULT_CALIBRATION_PATH = "motor_calibration.yaml"
FALLBACK_CALIBRATION = {
    "motors": {
        "0x00A": {
            "name": "test_motor_10",
            "raw_zero_p_uint": "0x855B",
            "raw_zero_pos_rad": 0.523194,
            "zero_command_persistent": False,
            "notes": (
                "Set-zero works during active session but was not retained after power cycle in this test; "
                "software offset required."
            ),
        }
    }
}


def _motor_key(motor_id: int) -> str:
    return f"0x{motor_id:03X}"


def load_motor_calibration(path: str = DEFAULT_CALIBRATION_PATH) -> dict[str, Any]:
    calibration_path = Path(path)
    try:
        import yaml
    except ImportError:
        if calibration_path.name == DEFAULT_CALIBRATION_PATH:
            return deepcopy(FALLBACK_CALIBRATION)
        raise RuntimeError("PyYAML is required to load non-default calibration files") from None

    with calibration_path.open("r", encoding="utf-8") as f:
        calibration = yaml.safe_load(f)

    if not isinstance(calibration, dict) or "motors" not in calibration:
        raise ValueError(f"Invalid motor calibration file: {path}")
    return calibration


def get_motor_offset_rad(motor_id: int, calibration: dict) -> float:
    key = _motor_key(motor_id)
    try:
        motor_calibration = calibration["motors"][key]
        return float(motor_calibration["raw_zero_pos_rad"])
    except KeyError as exc:
        raise KeyError(f"No calibration found for motor_id={key}") from exc


def apply_software_offset(raw_position_rad: float, motor_id: int, calibration: dict) -> float:
    return raw_position_rad - get_motor_offset_rad(motor_id, calibration)
