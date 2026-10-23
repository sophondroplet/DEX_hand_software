"""Packet-level motion policy: the last line of defence against an unsafe goal reaching a servo.

Checked by `SafeBus` at both choke points, on the actual wire bytes (16-bit values in the
byte order of each joint's servo model), independently of whatever code built the packet:

- WRITE 40 (torque), 1 byte: 0 for any known ID; 1 only for armed IDs.
- WRITE 42, 6 bytes (goal position, third word, speed): armed ID, position inside the
  joint's travel, 1 <= speed <= the joint's speed cap. The third word is the goal time on
  the SCS0009 (must be 0) and the goal current on the HLS3606 (1 <= current <= the cap
  verified at arming); on the HLS no word may carry the sign bit. Speed 0 means "maximum
  speed" on SCS servos ("stop" on the HLS) and is always refused. When the joint has a
  `max_goal_step`, the goal may differ from the previous accepted goal by at most
  that many steps: the SCS0009 speed register does not reliably limit speed, so
  this per-packet jump limit is what stops a stray far-away goal from being
  executed at full speed. The first goal after arming must equal the seeded
  (present) position. The one exception is a pre-authorised single-write goal
  (`authorize_goals`): exactly that position and speed, at most the joint's
  max_speed, accepted once.
- SYNC_WRITE at 42 (6 bytes per ID): every entry as above, unique IDs; one bad
  entry rejects the whole packet.
- SYNC_WRITE at 40 (1 byte per ID): torque-off only (all values 0).
- Everything else (EEPROM, lock, REG_WRITE/ACTION, other registers) is refused.

The armed set may only be changed by `control.arming`, which first verifies the servo
against its JointConfig (`VerifiedIdentity`). With nothing armed the policy is read-only
plus torque-off, so an E-stop can always be sent.
"""

from __future__ import annotations

import threading
from collections.abc import Iterator, Mapping, Sequence
from contextlib import contextmanager
from dataclasses import dataclass

from scservo_sdk.protocol_packet_handler import PKT_ID, PKT_LENGTH, PKT_PARAMETER0

from ..config import JointConfig, MotionConfig
from ..models import SCS0009, ServoModel
from ..registers import REG_TORQUE_ENABLE
from ..safe_bus import UnsafeOperationError, WritePolicy

REG_GOAL_BLOCK = 42
GOAL_BLOCK_LEN = 6
HLS_CURRENT_MAX = 2047  # register 44 range (6.5 mA units)


@dataclass(frozen=True)
class JointLimits:
    min_position: int
    max_position: int
    speed_cap: int  # speed register
    max_goal_step: int | None = None  # None: no per-packet jump limit (verify-motion's slow test moves)
    max_speed: int | None = None  # speed register cap for a pre-authorised single-write goal; None: never authorised
    model: ServoModel = SCS0009


@dataclass(frozen=True)
class VerifiedIdentity:
    """What `control.arming.verify_identity` confirmed on the servo; required to arm it."""

    servo_id: int
    model: ServoModel
    current_cap: int | None = None  # HLS3606 goal current (6.5 mA units); None on the SCS0009


def be16(value: int) -> bytes:
    return bytes([(value >> 8) & 0xFF, value & 0xFF])


def goal_block(position: int, speed: int, model: ServoModel = SCS0009, current: int = 0) -> bytes:
    """Wire bytes for registers 42..47: goal position, goal time (SCS0009, always 0) or goal current (HLS3606),
    speed register."""
    return model.goal_block(position, speed, current)


def limits_from_config(
    joints: Mapping[int, JointConfig], motion: MotionConfig, step_guard: bool = True
) -> dict[int, JointLimits]:
    limits = {}
    for servo_id, joint in joints.items():
        if joint.has_travel_limits:
            assert joint.min_position is not None and joint.max_position is not None
            model = joint.servo_model
            # The single-write register may not exceed max_speed: round down (None when even 1 is too fast).
            single = joint.max_speed if model is SCS0009 else model.speed_register(joint.max_speed, floor=True)
            limits[servo_id] = JointLimits(
                joint.min_position,
                joint.max_position,
                motion.speed_cap(model, joint.max_speed),
                motion.max_goal_step(joint) if step_guard else None,
                single or None,
                model,
            )
    return limits


class MotionPolicy(WritePolicy):
    def __init__(
        self,
        limits: Mapping[int, JointLimits],
        known_ids: Sequence[int] | None = None,
        models: Mapping[int, ServoModel] | None = None,
    ) -> None:
        super().__init__("motion")
        self.limits = dict(limits)
        self.known_ids = frozenset(known_ids if known_ids is not None else limits)
        self._models = {i: lim.model for i, lim in self.limits.items()} | dict(models or {})
        self._armed: frozenset[int] = frozenset()
        self._last_goal: dict[int, int] = {}
        self._authorized: dict[int, tuple[int, int]] = {}  # servo ID -> (position, speed), see authorize_goals
        self._current: dict[int, int] = {}  # armed HLS joints: goal current cap from their VerifiedIdentity
        self._lock = threading.Lock()

    @property
    def armed(self) -> frozenset[int]:
        return self._armed

    def model(self, servo_id: int) -> ServoModel:
        return self._models.get(servo_id, SCS0009)

    def current_cap(self, servo_id: int) -> int | None:
        return self._current.get(servo_id)

    def add_armed(self, servo_id: int, identity: VerifiedIdentity) -> None:
        limits = self.limits.get(servo_id)
        if limits is None:
            raise UnsafeOperationError(f"ID {servo_id} has no travel limits and cannot be armed")
        if identity.servo_id != servo_id or identity.model is not limits.model:
            raise UnsafeOperationError(f"ID {servo_id} cannot be armed: verified identity {identity} does not match "
                                       f"the configured {limits.model.label}")
        if limits.model.goal_third_word == "goal_current":
            if identity.current_cap is None or not 1 <= identity.current_cap <= HLS_CURRENT_MAX:
                raise UnsafeOperationError(f"ID {servo_id} cannot be armed without a goal-current cap "
                                           f"(got {identity.current_cap})")
        elif identity.current_cap is not None:
            raise UnsafeOperationError(f"ID {servo_id}: the {limits.model.label} takes no goal current")
        with self._lock:
            self._armed = self._armed | {servo_id}
            if identity.current_cap is not None:
                self._current[servo_id] = identity.current_cap

    def remove_armed(self, servo_id: int) -> None:
        with self._lock:
            self._armed = self._armed - {servo_id}
            self._last_goal.pop(servo_id, None)
            self._authorized.pop(servo_id, None)
            self._current.pop(servo_id, None)

    def clear_armed(self) -> None:
        with self._lock:
            self._armed = frozenset()
            self._last_goal.clear()
            self._authorized.clear()
            self._current.clear()

    def encode_goal(self, servo_id: int, position: int, speed: int) -> bytes:
        """Goal block for an armed joint, in its model's byte order, carrying its goal-current cap on the HLS."""
        return goal_block(position, speed, self.model(servo_id), self._current.get(servo_id, 0))

    def seed_goal(self, servo_id: int, position: int) -> None:
        """Reference for the jump limit: the position the joint holds when it is armed."""
        with self._lock:
            self._last_goal[servo_id] = position

    def last_goal(self, servo_id: int) -> int | None:
        return self._last_goal.get(servo_id)

    @contextmanager
    def authorize_goals(self, goals: Mapping[int, tuple[int, int]]) -> Iterator[None]:
        """Let exactly these (position, speed) goals skip the per-packet jump limit, once each.

        For a single-write move the servo paces the whole move itself, so the speed register is the
        only speed limit left and may not exceed the joint's max_speed. Every other check still
        applies. An entry is consumed when a packet carrying it is accepted; the rest is revoked on exit.
        """
        for servo_id, (_, speed) in goals.items():
            limits = self.limits.get(servo_id)
            if limits is None or limits.max_speed is None or not 1 <= speed <= limits.max_speed:
                cap = None if limits is None else limits.max_speed
                raise UnsafeOperationError(f"single-write speed {speed} for ID {servo_id} refused (max_speed {cap})")
        with self._lock:
            self._authorized = dict(goals)
        try:
            yield
        finally:
            with self._lock:
                self._authorized = {}

    def _commit(self, goals: Sequence[tuple[int, int]]) -> None:
        with self._lock:
            for servo_id, position in goals:
                self._last_goal[servo_id] = position
                self._authorized.pop(servo_id, None)

    def _check_goal(self, servo_id: int, data: Sequence[int]) -> int:
        if servo_id not in self._armed:
            raise UnsafeOperationError(f"goal for ID {servo_id} refused: joint is not armed")
        limits = self.limits[servo_id]
        model = limits.model
        position, third, speed = model.parse_goal_block(data)
        if model.position_sign_bit is not None and (position | third | speed) & (1 << model.position_sign_bit):
            raise UnsafeOperationError(f"goal block for ID {servo_id} refused: sign bit set ({bytes(data).hex()})")
        if not limits.min_position <= position <= limits.max_position:
            raise UnsafeOperationError(
                f"goal {position} for ID {servo_id} outside travel [{limits.min_position}, {limits.max_position}]"
            )
        if model.goal_third_word == "goal_time":
            if third != 0:
                raise UnsafeOperationError(f"goal time {third} refused (only speed-controlled moves are allowed)")
        else:
            cap = self._current.get(servo_id)
            if cap is None or not 1 <= third <= cap:
                raise UnsafeOperationError(f"goal current {third} for ID {servo_id} outside 1..{cap}")
        if not 1 <= speed <= limits.speed_cap:
            raise UnsafeOperationError(f"speed {speed} for ID {servo_id} outside 1..{limits.speed_cap}")
        if limits.max_goal_step is not None:
            previous = self._last_goal.get(servo_id)
            if previous is None:
                raise UnsafeOperationError(f"goal for ID {servo_id} refused: no seeded hold position")
            if self._authorized.get(servo_id) == (position, speed):
                return position
            if abs(position - previous) > limits.max_goal_step:
                raise UnsafeOperationError(
                    f"goal {position} for ID {servo_id} jumps {abs(position - previous)} steps from {previous} "
                    f"(max {limits.max_goal_step} per packet)"
                )
        return position

    def _check_torque(self, servo_id: int, value: int) -> None:
        if servo_id not in self.known_ids:
            raise UnsafeOperationError(f"torque write to unknown ID {servo_id} refused")
        if value == 1 and servo_id not in self._armed:
            raise UnsafeOperationError(f"torque on for ID {servo_id} refused: joint is not armed")
        if value not in (0, 1):
            raise UnsafeOperationError(f"torque value {value} refused")

    def check_write(self, packet: Sequence[int]) -> None:
        servo_id = packet[PKT_ID]
        data_len = packet[PKT_LENGTH] - 3
        address = packet[PKT_PARAMETER0]
        data = packet[PKT_PARAMETER0 + 1 : PKT_PARAMETER0 + 1 + data_len]
        if address == REG_TORQUE_ENABLE and data_len == 1:
            self._check_torque(servo_id, data[0])
        elif address == REG_GOAL_BLOCK and data_len == GOAL_BLOCK_LEN:
            self._commit([(servo_id, self._check_goal(servo_id, data))])
        else:
            raise UnsafeOperationError(f"WRITE addr={address} len={data_len} refused in motion mode")

    def check_sync_write(self, packet: Sequence[int]) -> None:
        param_len = packet[PKT_LENGTH] - 4
        address, data_len = packet[PKT_PARAMETER0], packet[PKT_PARAMETER0 + 1]
        payload = packet[PKT_PARAMETER0 + 2 : PKT_PARAMETER0 + 2 + param_len]
        stride = data_len + 1
        if data_len < 1 or param_len < stride or param_len % stride or len(payload) != param_len:
            raise UnsafeOperationError("malformed SYNC_WRITE")
        entries = [(payload[i], payload[i + 1 : i + stride]) for i in range(0, param_len, stride)]
        ids = [servo_id for servo_id, _ in entries]
        if len(set(ids)) != len(ids):
            raise UnsafeOperationError(f"SYNC_WRITE with duplicate IDs {ids} refused")
        if address == REG_TORQUE_ENABLE and data_len == 1:
            for servo_id, data in entries:
                if data[0] != 0:
                    raise UnsafeOperationError("SYNC_WRITE torque is only allowed for torque-off")
                self._check_torque(servo_id, 0)
        elif address == REG_GOAL_BLOCK and data_len == GOAL_BLOCK_LEN:
            # Validate every entry before committing any: one bad entry rejects the whole packet.
            self._commit([(servo_id, self._check_goal(servo_id, data)) for servo_id, data in entries])
        else:
            raise UnsafeOperationError(f"SYNC_WRITE addr={address} len={data_len} refused")
