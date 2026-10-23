"""`dexhand-servo hls-setup`: guarded EEPROM setup of one HLS3606 for single-turn joint use.

HLS servos can leave the factory in multi-turn mode (angle limits 0/0): the servo then does
not limit travel itself, and with full-angle feedback the reported position can cross zero
and go negative. For a hand without mechanical end stops the joint travel should also be
enforced by the servo, so the HLS bring-up is:

1. `--center` (only while the limits are still 0/0 and the joint has no travel limits in
   config.py): writes the position offset (register 31) that puts the present position in
   the middle of the turn (2048), so the 0/4095 seam is far from the joint's travel. The
   manual does not give the sign convention; the position is read back and the opposite
   sign is written if the first one moved it the wrong way.
2. Power-cycle, measure the travel with `monitor`, enter it in config.py JOINTS
   (`model="hls3606"`, `position_offset=` the offset written).
3. `--limits-from-config`: writes that travel as the EEPROM angle limits (registers 9/11,
   both in one 4-byte WRITE so min < max holds at every moment), which also leaves
   multi-turn mode.
4. Power-cycle and run `diagnose` (whether limits and offset apply without a power cycle
   is not documented).

Safety:
- only the angle limits (one 4-byte WRITE at 9), the position offset (2-byte WRITE at 31)
  and the lock (55, values 0/1) of the one servo named can be written, enforced at packet
  level by `HlsSetupPolicy`; limits 0/0 only for `--restore`;
- the servo must be an HLS3606 (read from the servo) with torque off; the bus census is
  taken first; the operator types the servo ID to confirm;
- the values found before the first change are saved once as `id<K>_hls_original.json`
  under data/servo_tuning/, every change is logged, `--restore` writes them back;
  everything is read back and verified, and the lock is re-enabled in all cases.
"""

from __future__ import annotations

import json
from collections.abc import Callable, Sequence
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

from .config import HLS_POSITION_OFFSET_MAX, JointConfig
from .diagnostics import CensusError, ensure_census, read_identity
from .models import HLS3606
from .safe_bus import SafeBus, UnsafeOperationError, WritePolicy
from .tune_servo import TUNING_DIR

REG_ANGLE_LIMITS = 9  # min (9..10) and max (11..12), written together
REG_POSITION_OFFSET = 31
SETUP_BLOCK_ADDR = 9
SETUP_BLOCK_LEN = 57 - SETUP_BLOCK_ADDR + 1  # 9..57: limits, offset, mode, torque, present position
CENTER = 2048
CENTER_TOLERANCE = 40  # steps (~3.5 deg): torque is off, the joint may be nudged while writing


class HlsSetupAborted(RuntimeError):
    """A precondition failed or the operator declined; nothing was written."""


def encode_offset(offset: int) -> bytes:
    """Register 31: sign-magnitude, BIT15 = negative."""
    return HLS3606.encode_word(abs(offset) | (0x8000 if offset < 0 else 0))


class HlsSetupPolicy(WritePolicy):
    def __init__(self, servo_id: int, allow_zero_limits: bool = False) -> None:
        super().__init__("hls-setup", frozenset({REG_ANGLE_LIMITS, REG_POSITION_OFFSET, HLS3606.reg_lock}),
                         HLS3606.reg_lock, servo_id)
        self.allow_zero_limits = allow_zero_limits

    def check_write(self, packet: Sequence[int]) -> None:
        servo_id, length, address = packet[2], packet[3] - 3, packet[5]
        data = list(packet[6 : 6 + length])
        if servo_id != self.servo_id:
            raise UnsafeOperationError(f"WRITE to ID {servo_id} refused: hls-setup is bound to ID {self.servo_id}")
        if address == HLS3606.reg_lock and length == 1:
            if data[0] not in (0, 1):
                raise UnsafeOperationError(f"lock value {data[0]} refused")
        elif address == REG_ANGLE_LIMITS and length == 4:
            lo, hi = HLS3606.word(data[0], data[1]), HLS3606.word(data[2], data[3])
            zero = (lo, hi) == (0, 0)
            if zero and not self.allow_zero_limits:
                raise UnsafeOperationError("angle limits 0/0 (multi-turn) are only written by --restore")
            if not zero and not (0 <= lo < hi <= HLS3606.position_max):
                raise UnsafeOperationError(f"angle limits {lo}/{hi} refused: need 0 <= min < max <= 4095")
        elif address == REG_POSITION_OFFSET and length == 2:
            raw = HLS3606.word(data[0], data[1])
            if raw & 0x7FFF > HLS_POSITION_OFFSET_MAX:
                raise UnsafeOperationError(f"position offset 0x{raw:04X} refused: magnitude above 4095")
        else:
            raise UnsafeOperationError(f"WRITE of {length} byte(s) to register {address} refused in hls-setup mode")


@dataclass(frozen=True)
class HlsSetup:
    min_angle: int
    max_angle: int
    offset: int
    mode: int
    torque: int
    position: int

    @property
    def multi_turn(self) -> bool:
        return self.min_angle == 0 and self.max_angle == 0


@dataclass
class SetupResult:
    servo_id: int
    before: HlsSetup
    after: HlsSetup | None
    verified: bool
    steps: list[str]


def read_setup(bus: SafeBus, servo_id: int) -> HlsSetup | None:
    t = bus.read(servo_id, SETUP_BLOCK_ADDR, SETUP_BLOCK_LEN)
    if not (t.ok and len(t.data) == SETUP_BLOCK_LEN):
        return None
    d = HLS3606.decode(t.data, base_addr=SETUP_BLOCK_ADDR)
    return HlsSetup(*(int(d[name].value) for name in (
        "min_angle_limit", "max_angle_limit", "position_offset", "mode", "torque_enable", "present_position")))


def _preconditions(bus: SafeBus, servo_id: int) -> HlsSetup:
    try:
        census = ensure_census(bus)
    except CensusError as exc:
        raise HlsSetupAborted(f"bus census failed: {exc}") from exc
    if shared := sorted(census.get(servo_id, frozenset()) - {servo_id}):
        raise HlsSetupAborted(f"ID {servo_id} is also the secondary ID of HLS servo(s) {shared}")
    model, identity = read_identity(bus, servo_id)
    if identity is None:
        raise HlsSetupAborted(f"ID {servo_id} does not reply cleanly")
    if model is not HLS3606:
        raise HlsSetupAborted(f"ID {servo_id} is {model.label if model else 'an unsupported servo'}, not an HLS3606")
    setup = read_setup(bus, servo_id)
    if setup is None:
        raise HlsSetupAborted(f"ID {servo_id} does not reply cleanly")
    if setup.torque:
        raise HlsSetupAborted(f"ID {servo_id} has torque on; switch it off (GUI 'Torque off', or close the GUI) first")
    return setup


def _original_path(directory: Path, servo_id: int) -> Path:
    return directory / f"id{servo_id}_hls_original.json"


def _save_backup(directory: Path, servo_id: int, before: HlsSetup, change: dict, action: str) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    record = {"servo_id": servo_id, "model": HLS3606.key, "time": datetime.now().isoformat(timespec="seconds"),
              "min_angle": before.min_angle, "max_angle": before.max_angle, "offset": before.offset}
    original = _original_path(directory, servo_id)
    if not original.exists():
        original.write_text(json.dumps(record, indent=2), encoding="utf-8")
    with (directory / "changes.jsonl").open("a", encoding="utf-8") as log:
        log.write(json.dumps({**record, "action": f"hls-setup {action}", "before": asdict(before),
                              "change": change}) + "\n")


def _write_eeprom(bus: SafeBus, servo_id: int, writes: list[tuple[str, int, bytes]], steps: list[str],
                  allow_zero_limits: bool = False) -> bool:
    """Unlock, write, and always re-lock; True when every write and the re-lock succeeded."""
    ok = True
    with bus.use_policy(HlsSetupPolicy(servo_id, allow_zero_limits)):
        unlock = bus.write_byte(servo_id, HLS3606.reg_lock, 0)
        steps.append(f"unlock EEPROM: {unlock.result.value}")
        ok = unlock.ok
        try:
            if unlock.ok:
                for label, address, data in writes:
                    written = bus.write_block(servo_id, address, data)
                    steps.append(f"write {label}: {written.result.value}")
                    ok = ok and written.ok
        finally:
            lock = bus.write_byte(servo_id, HLS3606.reg_lock, 1)
            steps.append(f"re-lock EEPROM: {lock.result.value}")
    return ok and lock.ok


def write_limits_from_config(
    bus: SafeBus, servo_id: int, joint: JointConfig | None, confirm: Callable[[str], bool],
    directory: Path = TUNING_DIR,
) -> SetupResult:
    if joint is None or joint.model != HLS3606.key or not joint.has_travel_limits:
        raise HlsSetupAborted(f"ID {servo_id} needs a config.py JOINTS entry with model='hls3606' and measured "
                              "travel limits first")
    assert joint.min_position is not None and joint.max_position is not None
    before = _preconditions(bus, servo_id)
    if before.offset != joint.position_offset:
        raise HlsSetupAborted(f"servo position offset is {before.offset}, but the travel in config.py was entered "
                              f"for offset {joint.position_offset}; re-measure or fix position_offset")
    if not joint.min_position <= before.position <= joint.max_position:
        raise HlsSetupAborted(f"position {before.position} is outside the configured travel "
                              f"[{joint.min_position}, {joint.max_position}]; move the joint into it by hand")
    if (before.min_angle, before.max_angle) == (joint.min_position, joint.max_position):
        raise HlsSetupAborted("the servo's angle limits already equal the configured travel")
    if not confirm(f"ID {servo_id} ({joint.name}): angle limits {before.min_angle}/{before.max_angle} -> "
                   f"{joint.min_position}/{joint.max_position}"
                   + (" (leaves multi-turn mode)" if before.multi_turn else "")):
        raise HlsSetupAborted("operator did not confirm")
    _save_backup(directory, servo_id, before, {"min_angle": joint.min_position, "max_angle": joint.max_position},
                 "limits")
    steps: list[str] = []
    data = HLS3606.encode_word(joint.min_position) + HLS3606.encode_word(joint.max_position)
    written = _write_eeprom(bus, servo_id, [("angle limits", REG_ANGLE_LIMITS, data)], steps)
    after = read_setup(bus, servo_id)
    verified = written and after is not None and (after.min_angle, after.max_angle) == (
        joint.min_position, joint.max_position)
    steps.append(f"read back: {after}" + ("" if verified else "  <- does NOT match the request"))
    return SetupResult(servo_id, before, after, verified, steps)


def center_offset(
    bus: SafeBus, servo_id: int, joint: JointConfig | None, confirm: Callable[[str], bool],
    directory: Path = TUNING_DIR,
) -> SetupResult:
    """Write the position offset that moves the present position to 2048, checking the sign on read-back."""
    if joint is not None and joint.has_travel_limits:
        raise HlsSetupAborted("this joint already has travel limits in config.py; a new offset would shift them "
                              "(remove the limits, re-centre, then re-measure)")
    before = _preconditions(bus, servo_id)
    if not before.multi_turn:
        raise HlsSetupAborted(f"angle limits are {before.min_angle}/{before.max_angle}; re-centring is only done "
                              "before limits are written (use --restore first)")
    if before.position < 0:
        raise HlsSetupAborted(f"position {before.position} is negative; power-cycle the servo first")
    delta = before.position - CENTER
    candidate = before.offset + delta
    if abs(candidate) > HLS_POSITION_OFFSET_MAX:
        candidate -= 4096 if candidate > 0 else -4096
    if not confirm(f"ID {servo_id}: position {before.position}, offset {before.offset} -> {candidate} "
                   f"(aims at position {CENTER}; torque stays off, nothing moves)"):
        raise HlsSetupAborted("operator did not confirm")
    _save_backup(directory, servo_id, before, {"offset": candidate}, "center")

    steps: list[str] = []
    written = _write_eeprom(bus, servo_id, [(f"position offset {candidate}", REG_POSITION_OFFSET,
                                             encode_offset(candidate))], steps)
    after = read_setup(bus, servo_id)
    if written and after is not None and after.offset == candidate:
        # Plain distances: with full-angle feedback a wrong-signed offset shows up as a negative position.
        moved = abs(after.position - before.position) > CENTER_TOLERANCE
        if moved and abs(after.position - CENTER) > CENTER_TOLERANCE:
            # The firmware applies the offset with the opposite sign: use the mirrored value.
            mirrored = 2 * before.offset - candidate
            if abs(mirrored) > HLS_POSITION_OFFSET_MAX:
                mirrored -= 4096 if mirrored > 0 else -4096
            steps.append(f"position moved to {after.position}, away from {CENTER}: writing offset {mirrored}")
            written = _write_eeprom(bus, servo_id, [(f"position offset {mirrored}", REG_POSITION_OFFSET,
                                                     encode_offset(mirrored))], steps)
            after = read_setup(bus, servo_id)
            candidate = mirrored
        elif not moved:
            steps.append("position unchanged: the offset is probably applied at power-up only; power-cycle, then "
                         f"check with --show that the position is near {CENTER}")
    verified = written and after is not None and after.offset == candidate
    steps.append(f"read back: {after}" + ("" if verified else "  <- does NOT match the request"))
    return SetupResult(servo_id, before, after, verified, steps)


def restore_setup(
    bus: SafeBus, servo_id: int, confirm: Callable[[str], bool], directory: Path = TUNING_DIR
) -> SetupResult:
    path = _original_path(directory, servo_id)
    if not path.exists():
        raise HlsSetupAborted(f"no saved original values for ID {servo_id} ({path})")
    original = json.loads(path.read_text(encoding="utf-8"))
    before = _preconditions(bus, servo_id)
    target = (int(original["min_angle"]), int(original["max_angle"]), int(original["offset"]))
    if not confirm(f"ID {servo_id}: restore angle limits {before.min_angle}/{before.max_angle} -> "
                   f"{target[0]}/{target[1]}, offset {before.offset} -> {target[2]}"):
        raise HlsSetupAborted("operator did not confirm")
    _save_backup(directory, servo_id, before, {"min_angle": target[0], "max_angle": target[1], "offset": target[2]},
                 "restore")
    steps: list[str] = []
    data = HLS3606.encode_word(target[0]) + HLS3606.encode_word(target[1])
    written = _write_eeprom(bus, servo_id, [("angle limits", REG_ANGLE_LIMITS, data),
                                            ("position offset", REG_POSITION_OFFSET, encode_offset(target[2]))],
                            steps, allow_zero_limits=True)
    after = read_setup(bus, servo_id)
    verified = written and after is not None and (after.min_angle, after.max_angle, after.offset) == target
    steps.append(f"read back: {after}" + ("" if verified else "  <- does NOT match the request"))
    return SetupResult(servo_id, before, after, verified, steps)


def show_setup(bus: SafeBus, servo_id: int) -> HlsSetup:
    model, _ = read_identity(bus, servo_id)
    if model is not HLS3606:
        raise HlsSetupAborted(f"ID {servo_id} is not a responding HLS3606")
    setup = read_setup(bus, servo_id)
    if setup is None:
        raise HlsSetupAborted(f"ID {servo_id} does not reply cleanly")
    return setup

