"""Tunable defaults for the bring-up tool. CLI flags override a subset of these.

`JOINTS` is the one table to edit by hand: which servo ID drives which joint,
and each joint's allowed travel. README section "Joint map and travel limits"
describes how to measure the values.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .models import MODELS, SCS0009, HLS3606, ServoModel

WCH_USB_VID = 0x1A86  # the URT-2's own USB-serial chip
# ESP32-S3 bridge (firmware/urt_bridge): Espressif VID plus the USB product name its firmware sets.
ESP32_USB_VID = 0x303A
BRIDGE_PRODUCT = "Dexhand URT Bridge"

# Servo supply is ~6 V; diagnose warnings and the motion supervisor share this window.
SUPPLY_VOLTAGE_WINDOW = (5.5, 6.5)

# Conservative bring-up default; raise per joint after `verify-motion` measured the speed units.
# Native steps/s of the joint's servo: ~44 deg/s on the SCS0009, ~13 deg/s on the HLS3606.
DEFAULT_MAX_SPEED = 150  # steps/s for generated trajectories
MAX_SPEED_LIMIT = 1000  # SCSCL goal-speed register range
HLS_CURRENT_LIMIT_MA_MAX = round(2047 * 6.5)  # register 28/44 range x 6.5 mA
HLS_POSITION_OFFSET_MAX = 4095


@dataclass(frozen=True)
class JointConfig:
    """One servo-driven joint. Positions are raw servo steps as shown by `diagnose` / `monitor`.

    `min_position` / `max_position` bound the travel any motion command may request.
    Fill them only after measuring the joint (README), already including the safety
    margin; leave both None until then. A joint without limits can never be armed.

    `max_speed` caps generated trajectories (steps/s). Measure it per joint with
    `verify-motion --speed-sweep` (the SCS0009 speed register is load-dependent, so the
    servo cannot be trusted to limit speed itself). Streamed goals are additionally
    limited per packet to `max_speed * MotionConfig.goal_step_window_s` steps.

    `model` is the servo type ("scs0009" or "hls3606"); positions, limits and `max_speed`
    are in that servo's own steps (SCS0009 ~0.293 deg, HLS3606 ~0.088 deg). The model is
    checked against the servo before every arming. HLS3606 only:
    - `current_limit_ma` caps the goal current (register 44); None uses the servo's own
      protection current (register 28).
    - `position_offset` is the EEPROM position offset (register 31) the travel limits were
      measured with; arming is refused when the servo reports another offset.
    """

    name: str
    min_position: int | None = None
    max_position: int | None = None
    max_speed: int = DEFAULT_MAX_SPEED
    model: str = SCS0009.key
    current_limit_ma: int | None = None
    position_offset: int = 0

    @property
    def has_travel_limits(self) -> bool:
        return self.min_position is not None and self.max_position is not None

    @property
    def servo_model(self) -> ServoModel:
        return MODELS[self.model]


# --------------------------------------------------------------------------- edit here
# Key = servo ID (1..253). Example of a filled entry:
#     3: JointConfig("index_mcp", min_position=212, max_position=688),
# 关节命名法
# 类别	缩写
# 侧别	left / right，或 LH / RH
# 手指	thumb、index、middle、ring、little/pinky、triple（三指合一）
# 关节	CMC 腕掌、MCP 掌指、PIP 近端指间、DIP 远端指间、IP 拇指指间
# 运动	flex 屈伸、abduct 外展、adduct 内收、opp 对掌、rot 旋转
# 方向	FE 屈伸、AA 外展/内收、OPP 对掌、ROT 旋转
JOINTS: dict[int, JointConfig] = {
    1: JointConfig("triple_pip", min_position=350, max_position=2000, model=HLS3606.key, max_speed=900),
    2: JointConfig("triple_dip", min_position=600, max_position=890),
    # 3, 4, 7: speed-sweep result 10 was force-limited creep, not a speed; default until mechanics are fixed
    3: JointConfig("index_pip", min_position=250, max_position=600),
    4: JointConfig("index_dip", min_position=250, max_position=590),
    5: JointConfig("index_mcp", min_position=435, max_position=600),
    6: JointConfig("thumb_mcp_rot", min_position=300, max_position=700),
    7: JointConfig("thumb_mcp", min_position=650, max_position=900),
    8: JointConfig("thumb_ip", min_position=20, max_position=280),
    # 9: not backdrivable, so not measured with `monitor`. Max = servo EEPROM max angle (1003; it rested
    # at 1006), min = 400 below; narrow/widen by trial
    9: JointConfig("thumb_cmc", min_position=503, max_position=1003, max_speed=800)
}
# ---------------------------------------------------------------------------------------

JOINT_MAP: dict[int, str] = {servo_id: joint.name for servo_id, joint in JOINTS.items()}


def validate_joints(joints: dict[int, JointConfig]) -> list[str]:
    """Return human-readable problems; an empty list means the table is consistent."""
    problems: list[str] = []
    names: dict[str, int] = {}
    for servo_id, joint in joints.items():
        where = f"JOINTS[{servo_id}]"
        if not 1 <= servo_id <= 253:
            problems.append(f"{where}: servo ID must be within 1..253")
        if not joint.name.strip():
            problems.append(f"{where}: name is empty (use 'TBD' until known)")
        elif joint.name != "TBD":
            if joint.name in names:
                problems.append(f"{where}: name '{joint.name}' already used by ID {names[joint.name]}")
            names[joint.name] = servo_id
        model = MODELS.get(joint.model)
        if model is None:
            problems.append(f"{where}: unknown model '{joint.model}' (known: {', '.join(MODELS)})")
            continue
        speed_limit = math.floor(model.steps_per_s(model.max_speed_register))
        if not (isinstance(joint.max_speed, int) and 1 <= joint.max_speed <= speed_limit):
            problems.append(f"{where}: max_speed must be an integer within 1..{speed_limit} ({model.label} steps/s)")
        if model.is_hls:
            limit = joint.current_limit_ma
            if limit is not None and not (isinstance(limit, int) and 7 <= limit <= HLS_CURRENT_LIMIT_MA_MAX):
                problems.append(f"{where}: current_limit_ma must be None or an integer within "
                                f"7..{HLS_CURRENT_LIMIT_MA_MAX}")
            offset = joint.position_offset
            if not (isinstance(offset, int) and abs(offset) <= HLS_POSITION_OFFSET_MAX):
                problems.append(f"{where}: position_offset must be an integer within "
                                f"-{HLS_POSITION_OFFSET_MAX}..{HLS_POSITION_OFFSET_MAX}")
        elif joint.current_limit_ma is not None or joint.position_offset != 0:
            problems.append(f"{where}: current_limit_ma / position_offset apply to HLS servos only")
        lo, hi = joint.min_position, joint.max_position
        if (lo is None) != (hi is None):
            problems.append(f"{where}: set both min_position and max_position, or neither")
            continue
        if lo is None or hi is None:
            continue
        if not all(isinstance(v, int) and not isinstance(v, bool) for v in (lo, hi)):
            problems.append(f"{where}: limits must be integers (raw steps)")
            continue
        if not model.position_min <= lo < hi <= model.position_max:
            problems.append(
                f"{where}: need {model.position_min} <= min_position < max_position <= {model.position_max} "
                f"({model.label}), got {lo}..{hi}"
            )
    return problems


@dataclass(frozen=True)
class BringupConfig:
    baudrate: int = 1_000_000
    # Replaces the SDK's hard-coded 50 ms LATENCY_TIMER; keeps a full 0-253 scan short.
    latency_ms: float = 20.0

    idle_listen_s: float = 0.2
    echo_probe_id: int = 253

    scan_ids: tuple[int, ...] = tuple(range(254))
    scan_retries: int = 2

    ping_trials: int = 100
    read_trials: int = 50
    id_register_trials: int = 5
    # Stop hammering a conflicted ID early: several servos driving the line at once is electrical contention.
    max_corrupt_frames: int = 5
    position_spread_max: int = 10
    healthy_success_rate: float = 0.99

    single_servo_read_trials: int = 50

    supply_voltage_window: tuple[float, float] = SUPPLY_VOLTAGE_WINDOW
    temperature_max_c: int = 50

    joints: dict[int, JointConfig] = field(default_factory=lambda: dict(JOINTS))

    @property
    def expected_ids(self) -> tuple[int, ...]:
        return tuple(self.joints)


@dataclass(frozen=True)
class MotionConfig:
    """Control loop, motion generation and safety-supervisor settings for the GUI driver.

    Step and steps/s values are in SCS0009 units (~0.293 deg per step) and are converted per joint
    to the joint's own servo (`ServoModel.scs_steps`, `ServoModel.native_speed`); on the SCS0009 the
    speed register is ~steps/s, so "register" below reads as SCS0009 steps/s for other models.
    """

    cycle_hz: float = 50.0
    # Reply timeout margin on the control bus: one dead servo must not eat the cycle budget.
    control_latency_ms: float = 4.0

    acceleration: float = 600.0  # steps/s^2 for every generated setpoint
    # Speed register sent with each streamed goal. On freely moving joints the register is close to
    # steps/s (verify-motion --speed-sweep: ~1:1 up to ~300, saturating at 400-550).
    #   "dynamic": register = max(floor, gain * |setpoint velocity|), capped at stream_speed_register.
    #              The servo's internal reference then glides between our 1/cycle_hz goal updates
    #              instead of jumping to each new goal and waiting (stair-step / stop-go).
    #   "fixed":   always stream_speed_register (previous behaviour; kept for A/B comparison).
    # Speed is never the safety limit: the trajectory and the per-packet goal jump limit are.
    stream_speed_mode: str = "dynamic"
    stream_speed_gain: float = 1.3
    stream_speed_floor: int = 50  # lowest register while streaming (final settling, holding corrections)
    stream_speed_register: int = 1000  # cap in dynamic mode, the value in fixed mode
    # Largest goal change one packet may carry = max_speed * window + 2 steps (window >= worst cycle dt).
    goal_step_window_s: float = 0.1
    arm_speed: int = 50  # speed register while arming (goal = present, so no motion expected)
    arm_hold_tolerance: int = 5  # steps between goal and present after arming
    jog_steps: tuple[int, ...] = (1, 5, 10)
    min_move_duration_s: float = 0.3

    load_warn_pct: float = 60.0
    load_trip_pct: float = 75.0
    load_trip_s: float = 0.3
    load_hard_pct: float = 80.0
    load_hard_s: float = 0.1
    tracking_error_steps: int = 40
    tracking_error_s: float = 0.4
    stall_window_s: float = 0.3
    stall_min_motion_steps: int = 2
    stall_load_pct: float = 60.0
    # Sustained-load warning: joint parked short of its target, servo still pushing (heat).
    hold_load_warn_pct: float = 25.0
    hold_error_steps: int = 3
    hold_warn_s: float = 3.0
    goal_mismatch_steps: int = 3  # goal register vs last sent setpoint (SYNC_WRITE has no reply)
    goal_mismatch_s: float = 0.3
    comm_fail_count: int = 3
    stale_s: float = 0.1
    supply_voltage_window: tuple[float, float] = SUPPLY_VOLTAGE_WINDOW
    supply_s: float = 0.2
    temperature_warn_c: int = 55
    temperature_trip_c: int = 65
    temperature_trip_s: float = 1.0
    # None disables the GUI heartbeat check (headless use and tests).
    heartbeat_freeze_s: float | None = 0.5
    heartbeat_trip_s: float | None = 2.0

    teach_rate_hz: float = 50.0
    keyframe_tolerance_steps: int = 3
    playback_max_time_scale: float = 1.0

    # HLS3606 only (the SCS0009 has no usable current reading): trip a joint whose phase current
    # stays at or above this fraction of its goal-current cap (the stall current) for this long.
    current_trip_frac: float = 0.9
    current_trip_s: float = 0.2

    def stream_speed(self, setpoint_velocity: float) -> int:
        """Speed register for one streamed SCS0009 goal."""
        return self.stream_register(SCS0009, setpoint_velocity)

    def stream_cap(self, model: ServoModel, max_speed: float | None = None) -> float:
        """Highest streamed speed in the model's steps/s.

        SCS0009: the register range (its speed register is load-dependent and not a limit anyway).
        HLS3606: its speed loop does limit speed, so the cap follows the joint's max_speed.
        """
        if model.is_hls and max_speed is not None:
            return max(self.stream_speed_gain * max_speed, model.native_speed(self.stream_speed_floor))
        return model.native_speed(self.stream_speed_register)

    def stream_register(self, model: ServoModel, setpoint_velocity: float, max_speed: float | None = None) -> int:
        """Speed register for one streamed goal; `setpoint_velocity` in the model's steps/s."""
        cap = self.stream_cap(model, max_speed)
        if self.stream_speed_mode == "fixed":
            return model.speed_register(cap)
        wanted = max(model.native_speed(self.stream_speed_floor), self.stream_speed_gain * abs(setpoint_velocity))
        return model.speed_register(min(cap, wanted))

    def arm_register(self, model: ServoModel) -> int:
        return model.speed_register(model.native_speed(self.arm_speed))

    def speed_cap(self, model: ServoModel, max_speed: float) -> int:
        """Largest speed register the motion policy accepts for a joint (streaming and arming)."""
        return max(model.speed_register(self.stream_cap(model, max_speed)), self.arm_register(model))

    def max_goal_step(self, joint: JointConfig) -> int:
        return math.ceil(joint.max_speed * self.goal_step_window_s) + joint.servo_model.scs_steps(2)
