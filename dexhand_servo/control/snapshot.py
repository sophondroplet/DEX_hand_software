"""Immutable state published by the control worker and consumed by the GUI."""

from __future__ import annotations

from dataclasses import dataclass, field

from ..models import SCS0009, ServoModel

STATUS_BLOCK_ADDR = 40
STATUS_BLOCK_LEN = SCS0009.status_block_len  # SCS0009 40..66: torque, goal, lock, present state, status, moving

# Servo status-packet error bits (SDK protocol_packet_handler ERRBIT_*).
ERROR_BITS = {1: "voltage", 2: "angle sensor", 4: "overheat", 8: "over-current", 32: "overload"}
ERRBIT_VOLTAGE = 1


def describe_error(byte: int) -> str:
    names = [name for bit, name in ERROR_BITS.items() if byte & bit]
    return ", ".join(names) if names else (f"0x{byte:02X}" if byte else "")


@dataclass(frozen=True)
class ServoReading:
    """One decoded read of the status block (40..66; 40..70 on the HLS3606, which adds the current).

    Positions and speeds are in the servo's own units; `current_ma` is None without a current reading.
    """

    torque: int
    goal: int
    goal_speed: int
    position: int
    speed: int
    load_pct: float
    voltage: float
    temperature: int
    status: int
    moving: int
    error_byte: int
    current_ma: float | None = None


def decode_status_block(data: list[int], error_byte: int, model: ServoModel = SCS0009) -> ServoReading:
    d = model.decode(data, base_addr=STATUS_BLOCK_ADDR)
    current = d.get("present_current") if model.current_ma_per_unit else None
    return ServoReading(
        torque=int(d["torque_enable"].value),
        goal=int(d["goal_position"].value),
        goal_speed=int(d["goal_speed"].value),
        position=int(d["present_position"].value),
        speed=int(d["present_speed"].value),
        load_pct=float(d["present_load"].value),
        voltage=float(d["present_voltage"].value),
        temperature=int(d["present_temperature"].value),
        status=int(d["status"].value),
        moving=int(d["moving"].value),
        error_byte=error_byte,
        current_ma=None if current is None else float(current.value),
    )


@dataclass(frozen=True)
class JointState:
    servo_id: int
    name: str
    min_position: int | None
    max_position: int | None
    reading: ServoReading | None  # last good read, may be stale
    comm_ok: bool  # last read succeeded
    consecutive_failures: int
    age_s: float | None  # time since the last good read
    armed: bool
    target: float | None  # where the operator / plan wants the joint
    setpoint: float | None  # rate-limited value actually sent this cycle
    fault: str | None  # latched fault for this joint
    warnings: tuple[str, ...] = ()
    model: ServoModel = SCS0009
    online: bool = True  # False: no reply at startup / last rescan; not polled, not armable


@dataclass(frozen=True)
class HandSnapshot:
    time: float
    cycle_hz: float
    mode: str
    joints: dict[int, JointState]
    estop_latched: bool
    estop_confirmed: bool | None  # None: no E-stop yet; False: torque-off could not be verified
    faults: tuple[str, ...]  # latched faults, newest last
    motion_frozen: bool
    comm_lost: bool
    recording_frames: int | None  # None when not recording
    plan_progress: float | None  # 0..1 while a pose / sync / playback move runs
    port: str
    # Result of the most recent torque-off-all (E-stop, protective trip, startup); None = none yet.
    torque_off_confirmed: bool | None = None
    cycle_busy_ms: float = 0.0  # smoothed time spent inside one control cycle
    cycle_budget_ms: float = 20.0
    notes: tuple[str, ...] = field(default_factory=tuple)
    offline: tuple[int, ...] = ()  # joints that did not reply at startup / last rescan


@dataclass(frozen=True)
class Event:
    time: float
    level: str  # "info" | "warn" | "error"
    text: str
