"""Arming (torque on, holding the present position) and disarming of single joints.

Before every arming the servo is checked against its JointConfig (`verify_identity`): the
model read from the servo must match, and an HLS3606 must be in single-turn position mode
with EEPROM angle limits around the configured travel, the configured position offset and
resolution 1. Its goal-current cap is read fresh from the servo's protection current.

Goal is written before torque so that, whether or not a goal write implicitly
enables torque on this firmware, the joint only ever holds where it already is.
The policy refuses goals to disarmed joints, so no other code path can switch
torque on.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..config import JointConfig, MotionConfig
from ..diagnostics import read_identity
from ..models import SCS0009, ServoModel
from ..registers import REG_TORQUE_ENABLE
from ..safe_bus import SafeBus, UnsafeOperationError
from .policy import REG_GOAL_BLOCK, MotionPolicy, VerifiedIdentity
from .snapshot import STATUS_BLOCK_ADDR, ServoReading, decode_status_block, describe_error

HLS_SETUP_ADDR = 9
HLS_SETUP_LEN = 33 - HLS_SETUP_ADDR + 1  # 9..33: angle limits .. operating mode


@dataclass(frozen=True)
class ArmResult:
    ok: bool
    reason: str
    reading: ServoReading | None = None


def read_status(bus: SafeBus, servo_id: int, model: ServoModel = SCS0009) -> ServoReading | None:
    length = model.status_block_len
    t = bus.read(servo_id, STATUS_BLOCK_ADDR, length)
    if not t.ok or len(t.data) != length:
        return None
    return decode_status_block(t.data, t.error, model)


def verify_identity(bus: SafeBus, servo_id: int, joint: JointConfig) -> tuple[VerifiedIdentity | None, str]:
    """(identity, "") when the servo matches `joint` and may be armed, else (None, reason)."""
    if bus.census is None:
        return None, "no bus census, so HLS secondary IDs are unknown (see the startup log)"
    shared = bus.census.get(servo_id, frozenset()) - {servo_id}
    if shared:
        return None, (f"ID {servo_id} is also the secondary ID of HLS servo(s) {sorted(shared)}; "
                      "change their secondary ID (register 7) first")
    model, identity = read_identity(bus, servo_id)
    if identity is None:
        return None, "no clean reply to the model check"
    expected = joint.servo_model
    if model is not expected:
        found = model.label if model else f"an unsupported servo (registers 0..4 = {identity})"
        return None, f"servo is {found}, config.py says {expected.label}"
    if not model.is_hls:
        return VerifiedIdentity(servo_id, model), ""

    assert joint.min_position is not None and joint.max_position is not None
    t = bus.read(servo_id, HLS_SETUP_ADDR, HLS_SETUP_LEN)
    if not (t.ok and len(t.data) == HLS_SETUP_LEN):
        return None, "no clean reply to the HLS setup check"
    d = model.decode(t.data, base_addr=HLS_SETUP_ADDR)
    lo, hi = int(d["min_angle_limit"].value), int(d["max_angle_limit"].value)
    if (lo, hi) == (0, 0):
        return None, "servo is in multi-turn mode (angle limits 0/0); run `hls-setup --limits-from-config` first"
    if not (lo <= joint.min_position and joint.max_position <= hi):
        return None, f"configured travel [{joint.min_position}, {joint.max_position}] exceeds the servo's angle " \
                     f"limits [{lo}, {hi}]"
    checks = (("angle resolution", int(d["angle_resolution"].value), 1),
              ("position offset", int(d["position_offset"].value), joint.position_offset),
              ("operating mode", int(d["mode"].value), 0))
    for label, value, wanted in checks:
        if value != wanted:
            return None, f"{label} is {value}, expected {wanted}"
    protection = int(d["protection_current"].raw)
    if not 1 <= protection <= 2047:
        return None, f"protection current register reads {protection}"
    cap = protection
    if joint.current_limit_ma is not None:
        cap = min(cap, max(1, math.floor(joint.current_limit_ma / model.current_ma_per_unit)))  # type: ignore[operator]
    return VerifiedIdentity(servo_id, model, cap), ""


def arm_joint(
    bus: SafeBus, policy: MotionPolicy, servo_id: int, joint: JointConfig, motion: MotionConfig
) -> ArmResult:
    if not joint.has_travel_limits:
        return ArmResult(False, "no travel limits configured")
    assert joint.min_position is not None and joint.max_position is not None
    identity, problem = verify_identity(bus, servo_id, joint)
    if identity is None:
        return ArmResult(False, problem)
    model = identity.model
    reading = read_status(bus, servo_id, model)
    if reading is None:
        return ArmResult(False, "no clean status reply")
    if reading.error_byte or reading.status:
        detail = describe_error(reading.error_byte) or f"status 0x{reading.status:02X}"
        return ArmResult(False, f"servo reports an error ({detail})", reading)
    if reading.position < 0:
        return ArmResult(False, f"position {reading.position} is negative (full-angle feedback crossed zero); "
                                "power-cycle the servo with the joint inside its travel", reading)
    if not joint.min_position <= reading.position <= joint.max_position:
        return ArmResult(
            False,
            f"position {reading.position} outside travel [{joint.min_position}, {joint.max_position}]; "
            "move the joint back by hand with torque off",
            reading,
        )

    try:
        policy.add_armed(servo_id, identity)
    except UnsafeOperationError as exc:
        return ArmResult(False, f"refused by motion policy: {exc}", reading)
    hold = reading.position
    policy.seed_goal(servo_id, hold)
    speed = min(motion.arm_register(model), policy.limits[servo_id].speed_cap)
    try:
        goal = bus.write_block(servo_id, REG_GOAL_BLOCK, policy.encode_goal(servo_id, hold, speed))
        torque = bus.write_byte(servo_id, REG_TORQUE_ENABLE, 1) if goal.ok else None
    except UnsafeOperationError as exc:
        disarm_joint(bus, policy, servo_id)
        return ArmResult(False, f"refused by motion policy: {exc}", reading)
    check = read_status(bus, servo_id, model) if torque is not None and torque.ok else None

    problem = None
    if not goal.ok:
        problem = f"goal write failed ({goal.result.value})"
    elif torque is None or not torque.ok:
        problem = "torque write failed"
    elif check is None:
        problem = "no read-back after arming"
    elif check.torque != 1:
        problem = "torque did not switch on"
    elif check.goal != hold:
        problem = f"goal register reads {check.goal}, expected {hold}"
    elif abs(check.position - hold) > model.scs_steps(motion.arm_hold_tolerance):
        problem = f"joint moved to {check.position} while arming (held {hold})"
    if problem:
        disarm_joint(bus, policy, servo_id)
        return ArmResult(False, problem, check or reading)
    return ArmResult(True, f"armed, holding {hold}", check)


def disarm_joint(bus: SafeBus, policy: MotionPolicy, servo_id: int) -> bool:
    """Remove from the armed set first (no goal can follow), then torque off and verify."""
    policy.remove_armed(servo_id)
    bus.write_byte(servo_id, REG_TORQUE_ENABLE, 0)
    check = read_status(bus, servo_id, policy.model(servo_id))
    return check is not None and check.torque == 0
