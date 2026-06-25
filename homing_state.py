"""Session-only homing state machine for single-encoder motors."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

from ak45_calibration import calibration_exists, get_ak45_entry
from motor_profiles import MotorProfile, encoder_output_half_period_deg, encoder_output_period_deg, format_motor_id, get_motor_profile


class HomingState(Enum):
    UNHOMED = "UNHOMED"
    HOMING_REQUIRED = "HOMING_REQUIRED"
    HOMED = "HOMED"
    AMBIGUOUS_STARTUP = "AMBIGUOUS_STARTUP"
    FAULT = "FAULT"


class HomingPolicy(Enum):
    REQUIRED_ON_COLD_START = "REQUIRED_ON_COLD_START"
    WARNING_ONLY = "WARNING_ONLY"


AK45_CLEAR_REASONS = {
    "feedback_timeout",
    "bus_off",
    "motor_reboot_suspected",
    "feedback_discontinuity",
    "abnormal_status",
    "can_reconnect",
    "controller_restart",
    "calibration_changed",
    "emergency_stop",
}


def default_policy_for_profile(profile: MotorProfile) -> HomingPolicy:
    if profile.model == "AK45-36-KV80":
        return HomingPolicy.REQUIRED_ON_COLD_START
    return HomingPolicy.WARNING_ONLY


@dataclass
class HomingMachine:
    motor_id: int
    policy: HomingPolicy | None = None
    state: HomingState = field(init=False)
    last_clear_reason: str | None = None

    def __post_init__(self) -> None:
        profile = get_motor_profile(self.motor_id)
        if self.policy is None:
            self.policy = default_policy_for_profile(profile)
        self.state = HomingState.UNHOMED

    @property
    def command_allowed(self) -> bool:
        if self.policy == HomingPolicy.WARNING_ONLY:
            return self.state in {HomingState.UNHOMED, HomingState.HOMED}
        return self.state == HomingState.HOMED

    def require_homing(self) -> None:
        self.state = HomingState.HOMING_REQUIRED

    def mark_ambiguous_startup(self) -> None:
        self.state = HomingState.AMBIGUOUS_STARTUP

    def confirm_horizontal_pose(self, raw_pos_rad: float, calibration: dict) -> HomingState:
        if not math.isfinite(raw_pos_rad):
            self.state = HomingState.FAULT
            return self.state
        if not calibration_exists(self.motor_id, calibration):
            self.state = HomingState.HOMING_REQUIRED
            return self.state
        entry = get_ak45_entry(self.motor_id, calibration)
        raw_zero = entry.get("raw_zero_pos_rad")
        if raw_zero is None or not math.isfinite(float(raw_zero)):
            self.state = HomingState.HOMING_REQUIRED
            return self.state
        self.state = HomingState.HOMED
        self.last_clear_reason = None
        return self.state

    def clear_homing(self, reason: str, fault: bool = False) -> HomingState:
        self.last_clear_reason = reason
        self.state = HomingState.FAULT if fault or reason == "bus_off" else HomingState.HOMING_REQUIRED
        return self.state

    def on_bus_off(self) -> HomingState:
        return self.clear_homing("bus_off", fault=True)

    def on_controller_restart(self) -> HomingState:
        self.last_clear_reason = "controller_restart"
        self.state = HomingState.UNHOMED
        return self.state

    def on_calibration_changed(self) -> HomingState:
        return self.clear_homing("calibration_changed")


def startup_state_for_motor(motor_id: int) -> HomingState:
    profile = get_motor_profile(motor_id)
    if default_policy_for_profile(profile) == HomingPolicy.REQUIRED_ON_COLD_START:
        return HomingState.UNHOMED
    return HomingState.UNHOMED


def encoder_ambiguity_summary(motor_id: int) -> dict[str, float | str]:
    profile = get_motor_profile(motor_id)
    return {
        "motor_id": format_motor_id(motor_id),
        "model": profile.model,
        "encoder_output_period_deg": encoder_output_period_deg(profile),
        "encoder_output_half_period_deg": encoder_output_half_period_deg(profile),
    }


def sensor_window_does_not_home() -> bool:
    """Deliberate policy guard: startup sensor proximity is never auto-HOMED."""
    return True

