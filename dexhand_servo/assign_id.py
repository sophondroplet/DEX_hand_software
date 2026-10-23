"""Guarded ID assignment for a single factory-fresh servo.

Same-ID servos cannot be told apart in software: a WRITE to ID 1 lands on every
servo that answers to ID 1. So this only writes after the bus has been shown to
hold exactly one servo at `from_id`, the target ID is free, and the operator has
typed the target ID back. The only registers ever written are ID (5) and the
model's EEPROM lock (48 on the SCS0009, 55 on the HLS3606), enforced by
`SafeBus.id_assignment_mode()`. The servo model is read from the servo before anything
is written.

HLS servos also act on WRITEs addressed to their secondary ID, which PING cannot see. The
bus census (`diagnostics.take_census`) is taken first; neither the old nor the new ID may
be another servo's secondary ID.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

from .config import BringupConfig
from .diagnostics import CensusError, Verdict, assess_link, dump_registers, ensure_census, read_identity
from .models import HLS3606, SCS0009, ServoModel
from .registers import MAX_SERVO_ID, REG_ID
from .safe_bus import CommResult, SafeBus, Transaction

WRITABLE_MODELS = (SCS0009, HLS3606)

TARGET_FREE_PROBES = 3


class AssignmentAborted(RuntimeError):
    """A precondition failed; nothing was written."""


@dataclass
class AssignmentResult:
    from_id: int
    to_id: int
    success: bool
    steps: list[str] = field(default_factory=list)
    writes: list[Transaction] = field(default_factory=list)


def read_snapshot(bus: SafeBus, servo_id: int) -> dict[str, Any]:
    dump = dump_registers(bus, servo_id)
    keys = ("model_number", "firmware_major", "firmware_minor", "present_voltage",
            "present_temperature", "present_position", "torque_enable", "baud_code")
    snapshot: dict[str, Any] = {k: dump.decoded[k].value for k in keys if k in dump.decoded}
    snapshot["model"] = dump.model.label if dump.model else "unknown"
    return snapshot


def writable_model(bus: SafeBus, servo_id: int, command: str) -> ServoModel:
    """The servo's model, read from the servo; refuses models this command cannot write safely."""
    model, identity = read_identity(bus, servo_id)
    if identity is None:
        raise AssignmentAborted(f"ID {servo_id} does not reply cleanly to a READ of registers 0..4")
    if model not in WRITABLE_MODELS:
        found = model.label if model else f"unsupported servo (registers 0..4 = {identity})"
        raise AssignmentAborted(f"ID {servo_id} is {found}; {command} supports only "
                                + ", ".join(m.label for m in WRITABLE_MODELS) + " for now")
    return model


def assign_id(
    bus: SafeBus,
    from_id: int,
    to_id: int,
    cfg: BringupConfig,
    confirm: Callable[[dict[str, Any]], bool],
) -> AssignmentResult:
    if not 0 <= from_id <= MAX_SERVO_ID:
        raise AssignmentAborted(f"--from {from_id} outside 0..{MAX_SERVO_ID}")
    if not 1 <= to_id <= MAX_SERVO_ID:
        raise AssignmentAborted(f"--to {to_id} outside 1..{MAX_SERVO_ID}")
    if to_id == from_id:
        raise AssignmentAborted("--to equals --from")

    try:
        census = ensure_census(bus)
    except CensusError as exc:
        raise AssignmentAborted(f"bus census failed: {exc}") from exc
    if shared := sorted(census.get(from_id, frozenset()) - {from_id}):
        raise AssignmentAborted(f"ID {from_id} is also the secondary ID of HLS servo(s) {shared}; "
                                "the ID write would rename them too")
    if shared := sorted(census.get(to_id, frozenset()) - {from_id}):
        raise AssignmentAborted(f"ID {to_id} is the secondary ID of HLS servo(s) {shared}; WRITEs to the new "
                                "ID would reach them too")

    for _ in range(TARGET_FREE_PROBES):
        probe = bus.ping(to_id)
        if probe.result is not CommResult.TIMEOUT:
            raise AssignmentAborted(f"ID {to_id} is not free (PING returned {probe.result.value})")

    assessment = assess_link(bus, from_id, cfg, read_trials=cfg.single_servo_read_trials)
    failures = (assessment.ping.timeout + assessment.ping.corrupt + assessment.read.timeout
                + assessment.read.corrupt)
    if assessment.verdict is not Verdict.HEALTHY_SINGLE or failures:
        detail = "; ".join(assessment.reasons) or f"{failures} failed transaction(s)"
        raise AssignmentAborted(
            f"Cannot confirm exactly one servo at ID {from_id} ({assessment.verdict.value}: {detail}). "
            "Connect only one un-assigned servo's signal line."
        )

    model = writable_model(bus, from_id, "assign-id")
    snapshot = read_snapshot(bus, from_id)
    snapshot.update(from_id=from_id, to_id=to_id, lock_register=model.reg_lock)
    if not confirm(snapshot):
        raise AssignmentAborted("operator did not confirm")

    result = AssignmentResult(from_id, to_id, success=False)
    with bus.id_assignment_mode(model, from_id):
        unlock = bus.write_byte(from_id, model.reg_lock, 0)
        result.writes.append(unlock)
        if not unlock.ok:
            result.steps.append(f"unlock EEPROM on ID {from_id}: {unlock.result.value}; stopped, ID unchanged")
            return result
        result.steps.append(f"unlock EEPROM on ID {from_id}: OK")

        # Some firmware acknowledges the ID write from the new ID, which the SDK
        # reports as a timeout; the verification below is what counts.
        change = bus.write_byte(from_id, REG_ID, to_id)
        result.writes.append(change)
        result.steps.append(f"write ID {from_id} -> {to_id}: {change.result.value} (verified below)")

    with bus.id_assignment_mode(model, to_id):
        lock = bus.write_byte(to_id, model.reg_lock, 1)
        result.writes.append(lock)
        result.steps.append(f"re-lock EEPROM on ID {to_id}: {lock.result.value}")

    target_ok = all(bus.ping(to_id).ok for _ in range(3))
    id_read = bus.read(to_id, REG_ID, 1)
    lock_read = bus.read(to_id, model.reg_lock, 1)
    old_gone = all(bus.ping(from_id).result is CommResult.TIMEOUT for _ in range(3))
    checks = {
        f"PING {to_id} answers": target_ok,
        f"ID register reads {to_id}": id_read.ok and id_read.data == [to_id],
        "EEPROM lock re-enabled": lock_read.ok and lock_read.data == [1],
        f"ID {from_id} no longer answers": old_gone,
    }
    for name, passed in checks.items():
        result.steps.append(f"verify {name}: {'OK' if passed else 'FAILED'}")
    result.success = all(checks.values())
    _refresh_census(bus, model, from_id, to_id, result)
    return result


def _refresh_census(bus: SafeBus, model: ServoModel, from_id: int, to_id: int, result: AssignmentResult) -> None:
    """Move the renamed servo in `bus.census`; for an HLS, re-read its secondary ID."""
    assert bus.census is not None
    census = {k: v - {from_id} for k, v in bus.census.items()}
    if model.reg_secondary_id is not None:
        read = bus.read(to_id, model.reg_secondary_id, 1)
        if not (read.ok and read.data):
            bus.census = None  # unknown again: writes stay refused until a new census
            result.steps.append("read secondary ID: FAILED; bus census cleared")
            return
        census[read.data[0]] = census.get(read.data[0], frozenset()) | {to_id}
        result.steps.append(f"secondary ID (register {model.reg_secondary_id}) reads {read.data[0]}")
    bus.census = {k: v for k, v in census.items() if v}
