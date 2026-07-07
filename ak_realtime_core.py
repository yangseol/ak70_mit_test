"""Realtime target/session core for mixed AK motors.

The core is testable without opening CAN.  It separates the historical
standalone/default watchdog behavior from the AK70 GUI persistent lease mode.
"""

from __future__ import annotations

import math
import time
import uuid
from dataclasses import dataclass, field, replace
from enum import Enum
from pathlib import Path
from typing import Any

from ak45_calibration import load_ak45_calibration
from calibration import load_motor_calibration
from homing_state import HomingMachine, HomingState
from motor_profiles import (
    AK70_KD_MAX,
    AK70_KD_MIN,
    AK70_KP_MAX,
    AK70_KP_MIN,
    format_motor_id,
    get_motor_profile,
    is_ak45,
    normalize_motor_id,
)


DEFAULT_RATE_HZ = 50.0
DEFAULT_MAX_VELOCITY_DEG_S = 60.0
HOLD_TIMEOUT_SEC = 0.3
STOP_TIMEOUT_SEC = 1.0
GUI_HEARTBEAT_INTERVAL_MS = 250
GUI_HEARTBEAT_MAX_AGE_SEC = 0.75
GUI_LEASE_TIMEOUT_SEC = 1.0
AK70_TARGET_LIMIT_DEG = 120.0
AK70_TORQUE_FF_MIN_NM = -25.0
AK70_TORQUE_FF_MAX_NM = 25.0
FEEDBACK_TIMEOUT_SEC = 0.5
MAX_CONSECUTIVE_FEEDBACK_MISSES = 10
FOLLOWING_ERROR_CONSECUTIVE_LIMIT = 3
FOLLOWING_ERROR_GRACE_SEC = 0.2
BASE_DIR = Path(__file__).resolve().parent
DEFAULT_CALIBRATION_PATH = BASE_DIR / "motor_calibration.yaml"


class ControllerMode(Enum):
    DISARMED = "DISARMED"
    ARMED = "ARMED"
    FAULT = "FAULT"
    ESTOPPED = "ESTOPPED"


class ControlMode(Enum):
    DEFAULT = "default"
    AK70_GUI_PERSISTENT_LEASE = "ak70-gui-persistent-lease"


class MotorState(Enum):
    RELEASED = "RELEASED"
    MOVING = "MOVING"
    HOLDING = "HOLDING"
    HOME_MOVING = "HOME_MOVING"
    HOME_HOLDING = "HOME_HOLDING"
    RELEASING = "RELEASING"
    GROUP_FAULTED = "GROUP_FAULTED"
    ESTOPPED = "ESTOPPED"
    LEASE_LOST = "LEASE_LOST"
    COMM_ERROR = "COMM_ERROR"


class ReleaseReason(Enum):
    RELEASE_IDS = "RELEASE_IDS"
    RELEASE_SESSION = "RELEASE_SESSION"
    ESTOP = "ESTOP"
    SHUTDOWN = "SHUTDOWN"
    LEASE_LOSS = "lease loss"
    CAN_FAULT = "CAN/BUS-OFF"
    GROUP_FAULT = "feedback/following error fault"
    DIRECT_FALLBACK = "direct fallback"
    LEGACY_FINITE_CLI = "legacy finite CLI"


class FaultScope(Enum):
    GROUP = "group"
    GLOBAL = "global"


@dataclass(frozen=True)
class RealtimeTarget:
    motor_id: int
    position_rad: float
    kp: float
    kd: float
    target_deg: float | None = None
    move_sec: float | None = None
    max_following_error_deg: float = 60.0
    control_group_id: str | None = None
    generation: int | None = None
    display_mode: str = "target"
    session_token: str | None = None
    session_epoch: int | None = None
    plan_id: str | None = None
    torque_ff_nm: float = 0.0


@dataclass
class TrajectoryWaypoint:
    target_deg: float
    duration: float
    target_position_rad: float | None = None


@dataclass
class TrajectoryPlan:
    plan_id: str
    control_group_id: str
    motor_ids: list[int]
    waypoints: dict[int, list[TrajectoryWaypoint]]
    kp: dict[int, float]
    kd: dict[int, float]
    torque_ff_nm: dict[int, float]
    max_following_error_deg: dict[int, float]
    current_waypoint_index: int
    segment_start_position_rad: dict[int, float]
    segment_target_position_rad: dict[int, float]
    segment_start_monotonic: float
    segment_duration_sec: float
    final_target_position_rad: dict[int, float]
    generation: dict[int, int]
    display_mode: str = "target"
    final_hold: bool = True
    session_token: str | None = None
    session_epoch: int | None = None


@dataclass
class MotorRuntime:
    motor_id: int
    command_position_rad: float = 0.0
    feedback_position_rad: float | None = None
    feedback_monotonic: float | None = None
    feedback_valid: bool = False
    consecutive_feedback_misses: int = 0
    homing: HomingMachine = field(init=False)
    enabled: bool = False
    mit_entered: bool = False
    command_initialized: bool = False
    generation: int = 0
    state: MotorState = MotorState.RELEASED
    control_group_id: str | None = None
    plan_id: str | None = None
    target_deg: float | None = None
    kp: float | None = None
    kd: float | None = None
    torque_ff_nm: float = 0.0
    owner_session_token: str | None = None
    following_error_deg: float | None = None
    following_error_exceed_count: int = 0
    fault: str | None = None
    start_position_rad: float | None = None
    target_position_rad: float | None = None
    trajectory_start_monotonic: float | None = None
    move_duration_sec: float = 0.0

    def __post_init__(self) -> None:
        self.homing = HomingMachine(self.motor_id)


@dataclass
class ControlSession:
    token: str
    epoch: int
    owner: str
    created_monotonic: float
    last_heartbeat_monotonic: float | None = None
    last_heartbeat_seq: int = -1
    active: bool = True


def _now(now: float | None = None) -> float:
    return time.monotonic() if now is None else now


def _clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def quintic_smoothstep(u: float) -> float:
    u = _clamp(u, 0.0, 1.0)
    return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5


def _target_limit_for(motor_id: int) -> float:
    if is_ak45(motor_id):
        return get_motor_profile(motor_id).bench_target_limit_deg
    return AK70_TARGET_LIMIT_DEG


def validate_ak70_torque_ff_nm(value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("AK70 torque_ff_nm must be finite")
    if not AK70_TORQUE_FF_MIN_NM <= value <= AK70_TORQUE_FF_MAX_NM:
        raise ValueError(
            f"AK70 torque_ff_nm must be {AK70_TORQUE_FF_MIN_NM:g}..{AK70_TORQUE_FF_MAX_NM:g} Nm"
        )
    return value


def _ak70_calibration_entry(calibration: dict[str, Any], motor_id: int) -> dict[str, Any]:
    key = format_motor_id(motor_id)
    motors = calibration.get("motors", {})
    if key not in motors:
        raise ValueError(f"{key} calibration is missing")
    entry = motors[key]
    if not isinstance(entry, dict):
        raise ValueError(f"{key} calibration entry is invalid")
    return entry


def _entry_for_motor(
    motor_id: int,
    calibration: dict[str, Any] | None,
    ak45_calibration: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Return the configured software-zero entry without changing either file.

    Older installations stored ID 6 in the AK70 file and ID 11 in the AK45
    file.  Those entries remain usable as an in-memory compatibility fallback
    after the fixed 6/12 AK45 wiring map is applied.
    """

    key = format_motor_id(motor_id)
    primary = ak45_calibration if is_ak45(motor_id) else calibration
    secondary = calibration if is_ak45(motor_id) else ak45_calibration
    for data in (primary, secondary):
        motors = data.get("motors", {}) if isinstance(data, dict) else {}
        entry = motors.get(key)
        if isinstance(entry, dict) and entry.get("raw_zero_pos_rad") is not None:
            return entry
    raise ValueError(f"{key} calibration is missing")


def joint_deg_to_raw_rad(
    motor_id: int,
    target_deg: float,
    calibration: dict[str, Any] | None,
    ak45_calibration: dict[str, Any] | None = None,
) -> float:
    """Convert a joint degree target to the model-specific protocol position."""

    motor_id = normalize_motor_id(motor_id)
    joint_rad = math.radians(target_deg)
    entry = _entry_for_motor(motor_id, calibration, ak45_calibration)
    raw_zero_pos_rad = float(entry["raw_zero_pos_rad"])
    direction_sign = float(entry.get("direction_sign", 1.0))
    raw_target_rad = raw_zero_pos_rad + direction_sign * joint_rad
    profile = get_motor_profile(motor_id)
    if not profile.p_min <= raw_target_rad <= profile.p_max:
        raise ValueError(f"{format_motor_id(motor_id)} raw target outside protocol range")
    return raw_target_rad


def raw_rad_to_joint_deg(
    motor_id: int,
    raw_pos_rad: float,
    calibration: dict[str, Any] | None,
    ak45_calibration: dict[str, Any] | None = None,
) -> float:
    motor_id = normalize_motor_id(motor_id)
    try:
        entry = _entry_for_motor(motor_id, calibration, ak45_calibration)
    except ValueError:
        return math.degrees(raw_pos_rad)
    raw_zero_pos_rad = float(entry["raw_zero_pos_rad"])
    direction_sign = float(entry.get("direction_sign", 1.0))
    return math.degrees(direction_sign * (raw_pos_rad - raw_zero_pos_rad))


@dataclass
class RealtimeCore:
    motor_ids: list[int]
    rate_hz: float = DEFAULT_RATE_HZ
    default_max_velocity_deg_s: float = DEFAULT_MAX_VELOCITY_DEG_S
    control_mode: ControlMode = ControlMode.DEFAULT
    calibration_path: str | Path = DEFAULT_CALIBRATION_PATH
    mode: ControllerMode = ControllerMode.DISARMED
    target_generation: int = 0
    latest_targets: dict[int, RealtimeTarget] = field(default_factory=dict)
    last_target_time: float | None = None
    requires_rearm: bool = False
    bus_fault: bool = False
    session: ControlSession | None = None
    session_epoch_counter: int = 0
    global_epoch: int = 0
    plans: dict[str, TrajectoryPlan] = field(default_factory=dict)
    faulted_groups: set[str] = field(default_factory=set)
    release_events: list[tuple[ReleaseReason, list[int]]] = field(default_factory=list)

    def __post_init__(self) -> None:
        if not self.motor_ids:
            raise ValueError("at least one motor ID is required")
        self.motor_ids = sorted(normalize_motor_id(motor_id) for motor_id in self.motor_ids)
        for motor_id in self.motor_ids:
            get_motor_profile(motor_id)
        self.motors = {motor_id: MotorRuntime(motor_id) for motor_id in self.motor_ids}
        self.calibration: dict[str, Any] | None = None
        self.ak45_calibration: dict[str, Any] | None = None
        self.reload_calibrations()

    def reload_calibrations(self) -> None:
        self.calibration = load_motor_calibration(str(self.calibration_path))
        try:
            self.ak45_calibration = load_ak45_calibration()
        except Exception:
            # Keep startup compatible with legacy files; per-motor conversion
            # still requires a valid entry before a non-zero joint target.
            self.ak45_calibration = {"motors": {}}

    def calibration_entry(self, motor_id: int | str) -> dict[str, Any]:
        return _entry_for_motor(normalize_motor_id(motor_id), self.calibration, self.ak45_calibration)

    def gains_for_motor(self, motor_id: int | str) -> tuple[float, float]:
        motor_id = normalize_motor_id(motor_id)
        profile = get_motor_profile(motor_id)
        try:
            entry = self.calibration_entry(motor_id)
        except ValueError:
            entry = {}
        return float(entry.get("kp", profile.default_kp)), float(entry.get("kd", profile.default_kd))

    def direction_sign_for_motor(self, motor_id: int | str) -> int:
        try:
            value = int(self.calibration_entry(motor_id).get("direction_sign", 1))
        except (ValueError, TypeError):
            value = 1
        return value if value in (-1, 1) else 1

    def joint_limits_for_motor(self, motor_id: int | str) -> tuple[float, float]:
        motor_id = normalize_motor_id(motor_id)
        fallback = get_motor_profile(motor_id).bench_target_limit_deg
        try:
            entry = self.calibration_entry(motor_id)
        except ValueError:
            entry = {}
        low = float(entry.get("joint_min_deg", entry.get("min_deg", -fallback)))
        high = float(entry.get("joint_max_deg", entry.get("max_deg", fallback)))
        if not math.isfinite(low) or not math.isfinite(high) or low >= high:
            return -fallback, fallback
        return low, high

    @property
    def max_step_rad(self) -> float:
        return math.radians(self.default_max_velocity_deg_s) / self.rate_hz

    def arm(self, owner: str = "standalone", session_token: str | None = None, now: float | None = None) -> dict[str, Any]:
        if self.bus_fault:
            raise RuntimeError("cannot arm while bus fault is latched")
        current = _now(now)
        if self.control_mode == ControlMode.AK70_GUI_PERSISTENT_LEASE:
            if self.session is not None and self.session.active:
                if session_token != self.session.token:
                    raise RuntimeError("another control session is active")
            else:
                self.session_epoch_counter += 1
                self.session = ControlSession(
                    token=session_token or uuid.uuid4().hex,
                    epoch=self.session_epoch_counter,
                    owner=owner,
                    created_monotonic=current,
                    last_heartbeat_monotonic=current,
                )
        self.mode = ControllerMode.ARMED
        self.requires_rearm = False
        return self.session_info(current)

    def disarm(self) -> None:
        for motor_id in self.latest_targets:
            self.motors[motor_id].torque_ff_nm = 0.0
        self.latest_targets.clear()
        self.plans.clear()
        self.mode = ControllerMode.DISARMED

    def confirm_home(self, motor_id: int, raw_pos_rad: float = 0.0, calibration: dict[str, Any] | None = None) -> HomingState:
        motor_id = normalize_motor_id(motor_id)
        machine = self.motors[motor_id].homing
        if calibration is None:
            if is_ak45(motor_id):
                raise ValueError("AK45 confirm_home requires calibration")
            machine.state = HomingState.HOMED
            return machine.state
        return machine.confirm_horizontal_pose(raw_pos_rad, calibration)

    def _require_session(self, session_token: str | None, session_epoch: int | None = None) -> ControlSession:
        if self.control_mode != ControlMode.AK70_GUI_PERSISTENT_LEASE:
            raise RuntimeError("session is only required in persistent lease mode")
        if self.session is None or not self.session.active:
            raise RuntimeError("no active control session")
        if session_token != self.session.token:
            raise RuntimeError("session token mismatch")
        if session_epoch is not None and session_epoch != self.session.epoch:
            raise RuntimeError("stale session_epoch")
        return self.session

    def heartbeat(
        self,
        session_token: str,
        heartbeat_seq: int,
        generated_monotonic: float,
        request_id: str | None = None,
        now: float | None = None,
    ) -> dict[str, Any]:
        current = _now(now)
        session = self._require_session(session_token)
        age = current - float(generated_monotonic)
        accepted = False
        reason = ""
        if heartbeat_seq <= session.last_heartbeat_seq:
            reason = "duplicate_or_old_seq"
        elif age < 0 or age > GUI_HEARTBEAT_MAX_AGE_SEC:
            reason = "stale_heartbeat"
        else:
            session.last_heartbeat_seq = int(heartbeat_seq)
            session.last_heartbeat_monotonic = current
            accepted = True
        return {
            "ok": True,
            "request_id": request_id,
            "command": "HEARTBEAT",
            "heartbeat_accepted": accepted,
            "reason": reason,
            "status": self.status(current),
        }

    def refresh_session_activity(
        self,
        session_token: str,
        session_epoch: int | None = None,
        now: float | None = None,
    ) -> None:
        """Keep a validated session alive while a synchronous startup step runs."""

        session = self._require_session(session_token, session_epoch)
        session.last_heartbeat_monotonic = _now(now)

    def validate_target(self, target: RealtimeTarget) -> None:
        motor_id = normalize_motor_id(target.motor_id)
        if motor_id not in self.motors:
            raise ValueError(f"untargeted controller motor {format_motor_id(motor_id)}")
        profile = get_motor_profile(motor_id)
        if not math.isfinite(target.position_rad) or not profile.p_min <= target.position_rad <= profile.p_max:
            raise ValueError("target position outside profile range")
        if not math.isfinite(target.kp) or not profile.kp_min <= target.kp <= profile.kp_max:
            raise ValueError("Kp outside profile range")
        if not math.isfinite(target.kd) or not profile.kd_min <= target.kd <= profile.kd_max:
            raise ValueError("Kd outside profile range")
        target_deg = target.target_deg
        if target_deg is None:
            target_deg = raw_rad_to_joint_deg(
                motor_id, target.position_rad, self.calibration, self.ak45_calibration
            )
        low, high = self.joint_limits_for_motor(motor_id)
        if not low <= target_deg <= high:
            raise ValueError(f"target_deg outside {low:g}..{high:g}")
        if not is_ak45(motor_id):
            validate_ak70_torque_ff_nm(target.torque_ff_nm)
            if not -AK70_TARGET_LIMIT_DEG <= target_deg <= AK70_TARGET_LIMIT_DEG:
                raise ValueError("AK70 target_deg outside -120..+120")
            if not AK70_KP_MIN < target.kp <= AK70_KP_MAX:
                raise ValueError("AK70 GUI Kp outside range")
            if not AK70_KD_MIN <= target.kd <= AK70_KD_MAX:
                raise ValueError("AK70 GUI Kd outside range")

    def has_fresh_feedback(self, motor_id: int | str, now: float | None = None) -> bool:
        motor_id = normalize_motor_id(motor_id)
        current = _now(now)
        runtime = self.motors[motor_id]
        profile = get_motor_profile(motor_id)
        if runtime.feedback_position_rad is None or runtime.feedback_monotonic is None:
            return False
        if not runtime.feedback_valid or not math.isfinite(runtime.feedback_position_rad):
            return False
        if not profile.p_min <= runtime.feedback_position_rad <= profile.p_max:
            return False
        return current - runtime.feedback_monotonic <= FEEDBACK_TIMEOUT_SEC

    def _current_session_command_is_valid(self, runtime: MotorRuntime, session_token: str | None) -> bool:
        return (
            runtime.command_initialized
            and runtime.owner_session_token == session_token
            and runtime.state in {MotorState.MOVING, MotorState.HOLDING, MotorState.HOME_MOVING, MotorState.HOME_HOLDING}
        )

    def needs_fresh_feedback_for_target(self, motor_id: int | str, session_token: str | None) -> bool:
        runtime = self.motors[normalize_motor_id(motor_id)]
        return not self._current_session_command_is_valid(runtime, session_token)

    def _select_start_position(self, motor_id: int, session_token: str | None, now: float) -> float:
        runtime = self.motors[motor_id]
        if self._current_session_command_is_valid(runtime, session_token):
            return runtime.command_position_rad
        if self.has_fresh_feedback(motor_id, now):
            assert runtime.feedback_position_rad is not None
            runtime.command_position_rad = runtime.feedback_position_rad
            runtime.command_initialized = True
            return runtime.feedback_position_rad
        raise RuntimeError(f"FEEDBACK_REQUIRED: {format_motor_id(motor_id)} fresh feedback required before target")

    def set_latest_targets(self, targets: list[RealtimeTarget], now: float | None = None) -> None:
        current = _now(now)
        if self.mode != ControllerMode.ARMED:
            raise RuntimeError("controller is not armed")
        seen: set[int] = set()
        next_targets = dict(self.latest_targets) if self.control_mode == ControlMode.AK70_GUI_PERSISTENT_LEASE else {}
        start_positions: dict[int, float] = {}
        for target in targets:
            motor_id = normalize_motor_id(target.motor_id)
            if motor_id in seen:
                raise ValueError(f"duplicate target {format_motor_id(motor_id)}")
            seen.add(motor_id)
            if self.control_mode == ControlMode.AK70_GUI_PERSISTENT_LEASE:
                self._require_session(target.session_token, target.session_epoch)
                if target.generation is not None and target.generation < self.motors[motor_id].generation:
                    raise RuntimeError(f"{format_motor_id(motor_id)} stale generation")
            self.validate_target(target)
            if self.control_mode == ControlMode.AK70_GUI_PERSISTENT_LEASE:
                start_positions[motor_id] = self._select_start_position(motor_id, target.session_token, current)
            else:
                start_positions[motor_id] = self.motors[motor_id].command_position_rad
        for target in targets:
            motor_id = normalize_motor_id(target.motor_id)
            runtime = self.motors[motor_id]
            runtime.generation += 1
            generation = runtime.generation
            target = RealtimeTarget(
                motor_id=motor_id,
                position_rad=target.position_rad,
                kp=target.kp,
                kd=target.kd,
                target_deg=target.target_deg,
                move_sec=target.move_sec,
                max_following_error_deg=target.max_following_error_deg,
                control_group_id=target.control_group_id or f"single-{format_motor_id(motor_id)}-{generation}",
                generation=generation,
                display_mode=target.display_mode,
                session_token=target.session_token,
                session_epoch=target.session_epoch,
                plan_id=target.plan_id,
                torque_ff_nm=target.torque_ff_nm,
            )
            runtime.control_group_id = target.control_group_id
            runtime.plan_id = target.plan_id
            runtime.target_deg = target.target_deg
            runtime.kp = target.kp
            runtime.kd = target.kd
            runtime.torque_ff_nm = target.torque_ff_nm
            runtime.owner_session_token = target.session_token
            runtime.state = MotorState.HOME_MOVING if target.display_mode == "home" else MotorState.MOVING
            runtime.enabled = True
            runtime.command_initialized = True
            runtime.start_position_rad = start_positions[motor_id]
            runtime.target_position_rad = target.position_rad
            runtime.trajectory_start_monotonic = current
            runtime.move_duration_sec = max(float(target.move_sec or 0.0), 1e-6)
            runtime.following_error_exceed_count = 0
            next_targets[motor_id] = target
            self._invalidate_plans_for_ids([motor_id])
        self.latest_targets = next_targets
        self.target_generation += 1
        self.last_target_time = current

    def clear_targets(self) -> None:
        self.latest_targets.clear()
        self.plans.clear()

    def _invalidate_plans_for_ids(self, motor_ids: list[int]) -> None:
        ids = {normalize_motor_id(motor_id) for motor_id in motor_ids}
        for plan_id, plan in list(self.plans.items()):
            if ids.intersection(plan.motor_ids):
                self.plans.pop(plan_id, None)

    def set_trajectory(
        self,
        *,
        motor_id: int | str,
        waypoints: list[TrajectoryWaypoint],
        kp: float,
        kd: float,
        max_following_error_deg: float,
        control_group_id: str,
        plan_id: str,
        session_token: str | None,
        session_epoch: int | None,
        display_mode: str = "target",
        torque_ff_nm: float = 0.0,
        now: float | None = None,
    ) -> None:
        current = _now(now)
        if self.mode != ControllerMode.ARMED:
            raise RuntimeError("controller is not armed")
        motor_id = normalize_motor_id(motor_id)
        if not waypoints:
            raise ValueError("trajectory requires at least one waypoint")
        if self.control_mode == ControlMode.AK70_GUI_PERSISTENT_LEASE:
            self._require_session(session_token, session_epoch)
        if not is_ak45(motor_id):
            torque_ff_nm = validate_ak70_torque_ff_nm(torque_ff_nm)
        raw_waypoints: list[TrajectoryWaypoint] = []
        for waypoint in waypoints:
            duration = float(waypoint.duration)
            if not math.isfinite(duration) or duration <= 0.0:
                raise ValueError("waypoint duration_sec must be positive")
            raw_target = joint_deg_to_raw_rad(motor_id, float(waypoint.target_deg), self.calibration, self.ak45_calibration)
            target = RealtimeTarget(
                motor_id=motor_id,
                position_rad=raw_target,
                kp=kp,
                kd=kd,
                target_deg=float(waypoint.target_deg),
                max_following_error_deg=max_following_error_deg,
                control_group_id=control_group_id,
                display_mode=display_mode,
                session_token=session_token,
                session_epoch=session_epoch,
                plan_id=plan_id,
                torque_ff_nm=torque_ff_nm,
            )
            self.validate_target(target)
            raw_waypoints.append(TrajectoryWaypoint(float(waypoint.target_deg), duration, raw_target))
        start = self._select_start_position(motor_id, session_token, current)
        runtime = self.motors[motor_id]
        runtime.generation += 1
        runtime.control_group_id = control_group_id
        runtime.plan_id = plan_id
        runtime.target_deg = raw_waypoints[-1].target_deg
        runtime.kp = kp
        runtime.kd = kd
        runtime.torque_ff_nm = torque_ff_nm
        runtime.owner_session_token = session_token
        runtime.state = MotorState.HOME_MOVING if display_mode == "home" else MotorState.MOVING
        runtime.enabled = True
        runtime.command_initialized = True
        runtime.start_position_rad = start
        runtime.target_position_rad = raw_waypoints[-1].target_position_rad
        runtime.trajectory_start_monotonic = current
        runtime.move_duration_sec = raw_waypoints[0].duration
        runtime.following_error_exceed_count = 0
        self.latest_targets[motor_id] = RealtimeTarget(
            motor_id=motor_id,
            position_rad=raw_waypoints[-1].target_position_rad if raw_waypoints[-1].target_position_rad is not None else 0.0,
            kp=kp,
            kd=kd,
            target_deg=raw_waypoints[-1].target_deg,
            move_sec=sum(w.duration for w in raw_waypoints),
            max_following_error_deg=max_following_error_deg,
            control_group_id=control_group_id,
            generation=runtime.generation,
            display_mode=display_mode,
            session_token=session_token,
            session_epoch=session_epoch,
            plan_id=plan_id,
            torque_ff_nm=torque_ff_nm,
        )
        self._invalidate_plans_for_ids([motor_id])
        assert raw_waypoints[0].target_position_rad is not None
        self.plans[plan_id] = TrajectoryPlan(
            plan_id=plan_id,
            control_group_id=control_group_id,
            motor_ids=[motor_id],
            waypoints={motor_id: raw_waypoints},
            kp={motor_id: kp},
            kd={motor_id: kd},
            torque_ff_nm={motor_id: torque_ff_nm},
            max_following_error_deg={motor_id: max_following_error_deg},
            current_waypoint_index=0,
            segment_start_position_rad={motor_id: start},
            segment_target_position_rad={motor_id: raw_waypoints[0].target_position_rad},
            segment_start_monotonic=current,
            segment_duration_sec=raw_waypoints[0].duration,
            final_target_position_rad={motor_id: raw_waypoints[-1].target_position_rad if raw_waypoints[-1].target_position_rad is not None else 0.0},
            generation={motor_id: runtime.generation},
            display_mode=display_mode,
            session_token=session_token,
            session_epoch=session_epoch,
        )
        self.target_generation += 1
        self.last_target_time = current

    def set_torque_ff(self, updates: dict[int | str, float], session_token: str | None) -> dict[str, float]:
        if self.control_mode == ControlMode.AK70_GUI_PERSISTENT_LEASE:
            session = self._require_session(session_token)
        else:
            session = None
        if not updates:
            raise ValueError("SET_TORQUE_FF requires updates")

        normalized: dict[int, float] = {}
        for raw_motor_id, raw_value in updates.items():
            motor_id = normalize_motor_id(raw_motor_id)
            motor_text = format_motor_id(motor_id)
            if motor_id in normalized:
                raise ValueError(f"duplicate torque FF update {motor_text}")
            if motor_id not in self.motors or is_ak45(motor_id):
                raise ValueError("SET_TORQUE_FF is AK70-only and must target controller-managed IDs")
            runtime = self.motors[motor_id]
            if motor_id not in self.latest_targets or runtime.owner_session_token != (session.token if session else session_token):
                raise RuntimeError(f"{motor_text} is not owned by current session")
            normalized[motor_id] = validate_ak70_torque_ff_nm(raw_value)

        for motor_id, value in normalized.items():
            runtime = self.motors[motor_id]
            runtime.torque_ff_nm = value
            target = self.latest_targets[motor_id]
            self.latest_targets[motor_id] = RealtimeTarget(
                motor_id=target.motor_id,
                position_rad=target.position_rad,
                kp=target.kp,
                kd=target.kd,
                target_deg=target.target_deg,
                move_sec=target.move_sec,
                max_following_error_deg=target.max_following_error_deg,
                control_group_id=target.control_group_id,
                generation=target.generation,
                display_mode=target.display_mode,
                session_token=target.session_token,
                session_epoch=target.session_epoch,
                plan_id=target.plan_id,
                torque_ff_nm=value,
            )
            for plan in self.plans.values():
                if motor_id in plan.torque_ff_nm:
                    plan.torque_ff_nm[motor_id] = value
        return {format_motor_id(motor_id): value for motor_id, value in sorted(normalized.items())}

    def set_gains(
        self,
        updates: dict[int | str, dict[str, float]],
        session_token: str | None,
    ) -> dict[str, dict[str, float]]:
        if self.control_mode == ControlMode.AK70_GUI_PERSISTENT_LEASE:
            session = self._require_session(session_token)
        else:
            session = None
        if not updates:
            raise ValueError("SET_GAINS requires updates")

        candidates: dict[int, RealtimeTarget] = {}
        for raw_motor_id, raw_gains in updates.items():
            motor_id = normalize_motor_id(raw_motor_id)
            motor_text = format_motor_id(motor_id)
            if motor_id in candidates:
                raise ValueError(f"duplicate gain update {motor_text}")
            if motor_id not in self.motors or not isinstance(raw_gains, dict):
                raise ValueError(f"invalid gain update for {motor_text}")
            runtime = self.motors[motor_id]
            expected_token = session.token if session else session_token
            if motor_id not in self.latest_targets or runtime.owner_session_token != expected_token:
                raise RuntimeError(f"{motor_text} is not owned by current session")
            target = self.latest_targets[motor_id]
            kp = float(raw_gains.get("kp", target.kp))
            kd = float(raw_gains.get("kd", target.kd))
            candidate = replace(target, kp=kp, kd=kd)
            self.validate_target(candidate)
            candidates[motor_id] = candidate

        for motor_id, candidate in candidates.items():
            runtime = self.motors[motor_id]
            runtime.kp = candidate.kp
            runtime.kd = candidate.kd
            self.latest_targets[motor_id] = candidate
            for plan in self.plans.values():
                if motor_id in plan.kp:
                    plan.kp[motor_id] = candidate.kp
                    plan.kd[motor_id] = candidate.kd
        return {
            format_motor_id(motor_id): {"kp": target.kp, "kd": target.kd}
            for motor_id, target in sorted(candidates.items())
        }

    def release_ids(
        self,
        motor_ids: list[int | str],
        session_token: str | None = None,
        expected_generations: dict[str, int] | None = None,
        reason: ReleaseReason = ReleaseReason.RELEASE_IDS,
    ) -> dict[str, Any]:
        if self.control_mode == ControlMode.AK70_GUI_PERSISTENT_LEASE:
            self._require_session(session_token)
        released: list[str] = []
        already: list[str] = []
        zero_torque_required: list[str] = []
        errors: dict[str, str] = {}
        for raw in motor_ids:
            try:
                motor_id = normalize_motor_id(raw)
                motor_text = format_motor_id(motor_id)
                if motor_id not in self.motors:
                    raise ValueError("RELEASE_IDS must target controller-managed IDs")
                runtime = self.motors[motor_id]
                expected = None if expected_generations is None else expected_generations.get(motor_text)
                if expected is not None and expected != runtime.generation:
                    raise RuntimeError("generation mismatch")
                self._invalidate_plans_for_ids([motor_id])
                existed = motor_id in self.latest_targets or runtime.state != MotorState.RELEASED
                self.latest_targets.pop(motor_id, None)
                runtime.generation += 1
                runtime.enabled = False
                runtime.state = MotorState.RELEASED
                runtime.control_group_id = None
                runtime.plan_id = None
                runtime.target_deg = None
                runtime.owner_session_token = None
                runtime.torque_ff_nm = 0.0
                runtime.command_initialized = False
                runtime.start_position_rad = None
                runtime.target_position_rad = None
                runtime.trajectory_start_monotonic = None
                (released if existed else already).append(motor_text)
                zero_torque_required.append(motor_text)
            except Exception as exc:
                errors[str(raw)] = str(exc)
        affected = [normalize_motor_id(m) for m in zero_torque_required]
        self.release_events.append((reason, affected))
        return {
            "released_ids": released,
            "already_released_ids": already,
            "zero_torque_required_ids": zero_torque_required,
            "errors": errors,
        }

    def release_session(self, session_token: str, reason: ReleaseReason = ReleaseReason.RELEASE_SESSION) -> dict[str, Any]:
        session = self._require_session(session_token)
        owned = [motor_id for motor_id, rt in self.motors.items() if rt.owner_session_token == session.token]
        result = self.release_ids(owned, session_token=session.token, reason=reason)
        self.plans.clear()
        session.active = False
        self.session = None
        self.mode = ControllerMode.DISARMED
        return result

    def estop(self) -> dict[str, Any]:
        self.global_epoch += 1
        active = list(self.latest_targets)
        self.latest_targets.clear()
        self.plans.clear()
        self.mode = ControllerMode.ESTOPPED
        self.requires_rearm = True
        for runtime in self.motors.values():
            runtime.generation += 1
            runtime.enabled = False
            runtime.state = MotorState.ESTOPPED
            runtime.torque_ff_nm = 0.0
        if self.session is not None:
            self.session.active = False
        self.release_events.append((ReleaseReason.ESTOP, active))
        return {"released_ids": [format_motor_id(m) for m in active], "global_epoch": self.global_epoch}

    def shutdown(self) -> dict[str, Any]:
        active = list(self.latest_targets)
        self.global_epoch += 1
        self.latest_targets.clear()
        self.plans.clear()
        self.mode = ControllerMode.DISARMED
        if self.session is not None:
            self.session.active = False
            self.session = None
        for motor_id in active:
            self.motors[motor_id].torque_ff_nm = 0.0
        self.release_events.append((ReleaseReason.SHUTDOWN, active))
        return {"released_ids": [format_motor_id(m) for m in active], "global_epoch": self.global_epoch}

    def apply_group_fault(self, control_group_id: str, fault: str, scope: FaultScope = FaultScope.GROUP) -> list[int]:
        if scope == FaultScope.GLOBAL:
            affected = list(self.latest_targets)
            self.latest_targets.clear()
            self.plans.clear()
            self.mode = ControllerMode.FAULT
            self.requires_rearm = True
            reason = ReleaseReason.CAN_FAULT
        else:
            affected = [
                motor_id
                for motor_id, target in self.latest_targets.items()
                if target.control_group_id == control_group_id
            ]
            for motor_id in affected:
                self.latest_targets.pop(motor_id, None)
            self._invalidate_plans_for_ids(affected)
            self.faulted_groups.add(control_group_id)
            reason = ReleaseReason.GROUP_FAULT
        for motor_id in affected:
            runtime = self.motors[motor_id]
            runtime.generation += 1
            runtime.enabled = False
            runtime.state = MotorState.GROUP_FAULTED
            runtime.fault = fault
            runtime.command_initialized = False
            runtime.owner_session_token = None
            runtime.torque_ff_nm = 0.0
            runtime.torque_ff_nm = 0.0
        self.release_events.append((reason, affected))
        return affected

    def on_feedback(self, motor_id: int | str, raw_position_rad: float, now: float | None = None) -> None:
        motor_id = normalize_motor_id(motor_id)
        runtime = self.motors[motor_id]
        profile = get_motor_profile(motor_id)
        if not math.isfinite(raw_position_rad) or not profile.p_min <= raw_position_rad <= profile.p_max:
            runtime.feedback_valid = False
            return
        runtime.feedback_position_rad = raw_position_rad
        runtime.feedback_monotonic = _now(now)
        runtime.feedback_valid = True
        runtime.consecutive_feedback_misses = 0
        if motor_id in self.latest_targets:
            runtime.following_error_deg = math.degrees(abs(runtime.command_position_rad - raw_position_rad))

    def on_bus_off_or_reconnect(self) -> None:
        active = list(self.latest_targets)
        self.latest_targets.clear()
        self.plans.clear()
        self.mode = ControllerMode.FAULT
        self.bus_fault = True
        self.requires_rearm = True
        for runtime in self.motors.values():
            runtime.homing.on_bus_off()
            runtime.state = MotorState.COMM_ERROR
            runtime.torque_ff_nm = 0.0
        self.release_events.append((ReleaseReason.CAN_FAULT, active))

    def on_controller_restart(self) -> None:
        self.latest_targets.clear()
        self.plans.clear()
        self.mode = ControllerMode.DISARMED
        for runtime in self.motors.values():
            runtime.homing.on_controller_restart()

    def _check_persistent_lease(self, now: float) -> None:
        if self.control_mode != ControlMode.AK70_GUI_PERSISTENT_LEASE:
            return
        if self.session is None or not self.session.active:
            return
        last = self.session.last_heartbeat_monotonic
        if last is not None and now - last <= GUI_LEASE_TIMEOUT_SEC:
            return
        active = list(self.latest_targets)
        self.latest_targets.clear()
        self.plans.clear()
        self.mode = ControllerMode.DISARMED
        self.requires_rearm = True
        self.session.active = False
        self.session = None
        for motor_id in active:
            runtime = self.motors[motor_id]
            runtime.generation += 1
            runtime.enabled = False
            runtime.state = MotorState.LEASE_LOST
            runtime.torque_ff_nm = 0.0
        self.release_events.append((ReleaseReason.LEASE_LOSS, active))

    def _check_active_feedback_and_following_error(self, now: float) -> None:
        checked_groups: set[str] = set()
        for motor_id, target in list(self.latest_targets.items()):
            runtime = self.motors[motor_id]
            group = target.control_group_id or runtime.control_group_id or f"single-{format_motor_id(motor_id)}"
            if group in checked_groups and group in self.faulted_groups:
                continue
            if not self.has_fresh_feedback(motor_id, now):
                runtime.consecutive_feedback_misses += 1
                feedback_age = None if runtime.feedback_monotonic is None else now - runtime.feedback_monotonic
                if runtime.consecutive_feedback_misses >= MAX_CONSECUTIVE_FEEDBACK_MISSES or (
                    feedback_age is not None and feedback_age > FEEDBACK_TIMEOUT_SEC
                ):
                    self.apply_group_fault(group, "feedback timeout", FaultScope.GROUP)
                    checked_groups.add(group)
                continue
            runtime.consecutive_feedback_misses = 0
            if runtime.state not in {MotorState.MOVING, MotorState.HOLDING, MotorState.HOME_MOVING, MotorState.HOME_HOLDING}:
                continue
            if not runtime.command_initialized or runtime.feedback_position_rad is None:
                continue
            start = runtime.trajectory_start_monotonic
            if start is not None and now - start < FOLLOWING_ERROR_GRACE_SEC:
                runtime.following_error_exceed_count = 0
                continue
            following_error_rad = abs(runtime.command_position_rad - runtime.feedback_position_rad)
            runtime.following_error_deg = math.degrees(following_error_rad)
            limit_rad = math.radians(float(target.max_following_error_deg))
            if following_error_rad > limit_rad:
                runtime.following_error_exceed_count += 1
                if runtime.following_error_exceed_count >= FOLLOWING_ERROR_CONSECUTIVE_LIMIT:
                    self.apply_group_fault(group, "following error", FaultScope.GROUP)
                    checked_groups.add(group)
            else:
                runtime.following_error_exceed_count = 0

    def _command_for_target(self, motor_id: int, target: RealtimeTarget, now: float) -> RealtimeTarget:
        runtime = self.motors[motor_id]
        if runtime.start_position_rad is None or runtime.target_position_rad is None or runtime.trajectory_start_monotonic is None:
            runtime.command_position_rad = target.position_rad
        else:
            duration = max(runtime.move_duration_sec, 1e-6)
            elapsed = now - runtime.trajectory_start_monotonic
            u = _clamp(elapsed / duration, 0.0, 1.0)
            s = quintic_smoothstep(u)
            runtime.command_position_rad = runtime.start_position_rad + (runtime.target_position_rad - runtime.start_position_rad) * s
            if u >= 1.0:
                runtime.command_position_rad = runtime.target_position_rad
                runtime.state = MotorState.HOME_HOLDING if target.display_mode == "home" else MotorState.HOLDING
        return RealtimeTarget(
            motor_id=motor_id,
            position_rad=runtime.command_position_rad,
            kp=target.kp,
            kd=target.kd,
            target_deg=target.target_deg,
            control_group_id=target.control_group_id,
            generation=target.generation,
            display_mode=target.display_mode,
            session_token=target.session_token,
            session_epoch=target.session_epoch,
            plan_id=target.plan_id,
            torque_ff_nm=runtime.torque_ff_nm,
        )

    def _command_for_plan(self, plan: TrajectoryPlan, now: float) -> dict[int, RealtimeTarget]:
        commands: dict[int, RealtimeTarget] = {}
        motor_id = plan.motor_ids[0]
        if motor_id not in self.latest_targets:
            self.plans.pop(plan.plan_id, None)
            return commands
        runtime = self.motors[motor_id]
        target = self.latest_targets[motor_id]
        elapsed = now - plan.segment_start_monotonic
        u = _clamp(elapsed / max(plan.segment_duration_sec, 1e-6), 0.0, 1.0)
        s = quintic_smoothstep(u)
        start = plan.segment_start_position_rad[motor_id]
        end = plan.segment_target_position_rad[motor_id]
        runtime.command_position_rad = start + (end - start) * s
        if u >= 1.0:
            runtime.command_position_rad = end
        commands[motor_id] = RealtimeTarget(
            motor_id=motor_id,
            position_rad=runtime.command_position_rad,
            kp=plan.kp[motor_id],
            kd=plan.kd[motor_id],
            target_deg=target.target_deg,
            control_group_id=plan.control_group_id,
            generation=plan.generation[motor_id],
            display_mode=plan.display_mode,
            session_token=plan.session_token,
            session_epoch=plan.session_epoch,
            plan_id=plan.plan_id,
            torque_ff_nm=runtime.torque_ff_nm,
        )
        if u >= 1.0:
            next_index = plan.current_waypoint_index + 1
            waypoints = plan.waypoints[motor_id]
            if next_index >= len(waypoints):
                runtime.command_position_rad = plan.final_target_position_rad[motor_id]
                runtime.state = MotorState.HOME_HOLDING if plan.display_mode == "home" else MotorState.HOLDING
                runtime.start_position_rad = runtime.command_position_rad
                runtime.target_position_rad = runtime.command_position_rad
                runtime.trajectory_start_monotonic = now
                runtime.move_duration_sec = 1e-6
                self.plans.pop(plan.plan_id, None)
            else:
                next_wp = waypoints[next_index]
                assert next_wp.target_position_rad is not None
                plan.current_waypoint_index = next_index
                plan.segment_start_position_rad[motor_id] = end
                plan.segment_target_position_rad[motor_id] = next_wp.target_position_rad
                plan.segment_start_monotonic = now
                plan.segment_duration_sec = next_wp.duration
                runtime.start_position_rad = end
                runtime.target_position_rad = next_wp.target_position_rad
                runtime.trajectory_start_monotonic = now
                runtime.move_duration_sec = next_wp.duration
        return commands

    def compute_cycle_commands(self, now: float | None = None) -> dict[int, RealtimeTarget]:
        current_time = _now(now)
        self._check_persistent_lease(current_time)
        if self.mode != ControllerMode.ARMED or self.last_target_time is None:
            return {}
        self._check_active_feedback_and_following_error(current_time)
        age = current_time - self.last_target_time
        if self.control_mode == ControlMode.DEFAULT and age >= STOP_TIMEOUT_SEC:
            self.latest_targets.clear()
            self.mode = ControllerMode.DISARMED
            self.requires_rearm = True
            return {}
        commands: dict[int, RealtimeTarget] = {}
        planned_ids: set[int] = set()
        for plan_id, plan in list(self.plans.items()):
            if plan_id not in self.plans:
                continue
            plan_commands = self._command_for_plan(plan, current_time)
            commands.update(plan_commands)
            planned_ids.update(plan_commands)
        for motor_id, target in list(self.latest_targets.items()):
            if motor_id in planned_ids:
                continue
            if self.control_mode == ControlMode.DEFAULT and age >= HOLD_TIMEOUT_SEC:
                self.motors[motor_id].command_position_rad = target.position_rad
            commands[motor_id] = self._command_for_target(motor_id, target, current_time)
        return commands

    def consume_release_events(self) -> list[tuple[ReleaseReason, list[int]]]:
        events = list(self.release_events)
        self.release_events.clear()
        return events

    def session_info(self, now: float | None = None) -> dict[str, Any]:
        current = _now(now)
        if self.session is None:
            return {
                "active_session": False,
                "session_token": None,
                "session_epoch": None,
                "session_owner": None,
                "heartbeat_seq": None,
                "heartbeat_age": None,
                "lease_ok": False,
                "lease_remaining_sec": 0.0,
            }
        last = self.session.last_heartbeat_monotonic
        heartbeat_age = None if last is None else current - last
        active = bool(self.session.active)
        lease_remaining = (
            0.0
            if heartbeat_age is None or not active
            else max(0.0, GUI_LEASE_TIMEOUT_SEC - heartbeat_age)
        )
        return {
            "active_session": active,
            "session_token": self.session.token,
            "session_epoch": self.session.epoch,
            "session_owner": self.session.owner,
            "heartbeat_seq": self.session.last_heartbeat_seq,
            "heartbeat_age": heartbeat_age,
            "lease_ok": heartbeat_age is not None and heartbeat_age <= GUI_LEASE_TIMEOUT_SEC and self.session.active,
            "lease_remaining_sec": lease_remaining,
        }

    def status(self, now: float | None = None) -> dict[str, Any]:
        current = _now(now)
        session = self.session_info(current)
        return {
            "running": True,
            "mode": self.mode.value,
            "control_mode": self.control_mode.value,
            "target_generation": self.target_generation,
            "latest_target_count": len(self.latest_targets),
            "requires_rearm": self.requires_rearm,
            "bus_fault": self.bus_fault,
            "global_epoch": self.global_epoch,
            "session": session,
            "session_token": session["session_token"],
            "session_epoch": session["session_epoch"],
            "session_owner": session["session_owner"],
            "active_session": session["active_session"],
            "heartbeat_seq": session["heartbeat_seq"],
            "heartbeat_age": session["heartbeat_age"],
            "lease_ok": session["lease_ok"],
            "lease_remaining_sec": session["lease_remaining_sec"],
            "active_owned_ids": [format_motor_id(m) for m in sorted(self.latest_targets)],
            "owned_motor_ids": [format_motor_id(m) for m in sorted(self.latest_targets)],
            "active_target_ids": [format_motor_id(m) for m in sorted(self.latest_targets)],
            "motors": {
                format_motor_id(motor_id): {
                    "model": get_motor_profile(motor_id).model,
                    "homing": runtime.homing.state.value,
                    "command_position_rad": runtime.command_position_rad,
                    "commanded_deg": raw_rad_to_joint_deg(
                        motor_id, runtime.command_position_rad, self.calibration, self.ak45_calibration
                    ),
                    "actual_deg": None
                    if runtime.feedback_position_rad is None
                    else raw_rad_to_joint_deg(
                        motor_id, runtime.feedback_position_rad, self.calibration, self.ak45_calibration
                    ),
                    "raw_position_rad": runtime.feedback_position_rad,
                    "joint_limit_min_deg": self.joint_limits_for_motor(motor_id)[0],
                    "joint_limit_max_deg": self.joint_limits_for_motor(motor_id)[1],
                    "direction_sign": self.direction_sign_for_motor(motor_id),
                    "feedback_age": None if runtime.feedback_monotonic is None else current - runtime.feedback_monotonic,
                    "state": runtime.state.value,
                    "control_group_id": runtime.control_group_id,
                    "generation": runtime.generation,
                    "plan_id": runtime.plan_id,
                    "target_deg": runtime.target_deg,
                    "following_error_deg": runtime.following_error_deg,
                    "feedback_valid": runtime.feedback_valid,
                    "consecutive_feedback_misses": runtime.consecutive_feedback_misses,
                    "command_initialized": runtime.command_initialized,
                    "start_position_rad": runtime.start_position_rad,
                    "target_position_rad": runtime.target_position_rad,
                    "trajectory_start_monotonic": runtime.trajectory_start_monotonic,
                    "move_duration_sec": runtime.move_duration_sec,
                    "kp": runtime.kp,
                    "kd": runtime.kd,
                    "torque_ff_nm": runtime.torque_ff_nm,
                    "fault": runtime.fault,
                    "owned": motor_id in self.latest_targets,
                }
                for motor_id, runtime in self.motors.items()
            },
        }


def targets_from_ipc(message: dict[str, Any], core: RealtimeCore | None = None) -> list[RealtimeTarget]:
    targets = []
    control_group_id = message.get("control_group_id")
    session_token = message.get("session_token")
    session_epoch = message.get("session_epoch")
    for item in message.get("targets", []):
        motor_id = normalize_motor_id(str(item["motor_id"]))
        target_deg: float | None = None
        if "position_rad" in item:
            position_rad = float(item["position_rad"])
            if "target_deg" in item:
                target_deg = float(item["target_deg"])
        else:
            target_deg = float(item.get("target_deg", item["position_deg"]))
            if core is not None:
                position_rad = joint_deg_to_raw_rad(
                    motor_id, target_deg, core.calibration, core.ak45_calibration
                )
            else:
                position_rad = math.radians(target_deg)
        profile = get_motor_profile(motor_id)
        default_kp, default_kd = (
            core.gains_for_motor(motor_id) if core is not None else (profile.default_kp, profile.default_kd)
        )
        targets.append(
            RealtimeTarget(
                motor_id=motor_id,
                position_rad=position_rad,
                kp=float(item.get("kp", default_kp)),
                kd=float(item.get("kd", default_kd)),
                target_deg=target_deg,
                move_sec=None if item.get("move_sec") is None else float(item["move_sec"]),
                max_following_error_deg=float(item.get("max_following_error_deg", 60.0)),
                control_group_id=str(item.get("control_group_id", control_group_id or "")) or None,
                generation=None if item.get("generation") is None else int(item["generation"]),
                display_mode=str(item.get("display_mode", "target")),
                session_token=str(item.get("session_token", session_token)) if item.get("session_token", session_token) else None,
                session_epoch=None if item.get("session_epoch", session_epoch) is None else int(item.get("session_epoch", session_epoch)),
                plan_id=None if item.get("plan_id") is None else str(item["plan_id"]),
                torque_ff_nm=float(item.get("torque_ff_nm", 0.0)),
            )
        )
    return targets
