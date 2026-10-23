"""Safety supervisor: turns one cycle of servo readings into protective actions.

Holds only debounce timers and a short history; it never touches the bus. The
worker applies the returned trips (torque off, latched fault). Clock values are
passed in, so every threshold/debounce can be tested deterministically.
"""

from __future__ import annotations

import statistics
from collections import deque
from collections.abc import Mapping
from dataclasses import dataclass, field

from ..config import MotionConfig
from .snapshot import ERRBIT_VOLTAGE, ServoReading, describe_error


@dataclass(frozen=True)
class JointInput:
    reading: ServoReading | None  # this cycle's read; None if it failed
    consecutive_failures: int
    last_ok: float | None  # clock time of the last good read
    armed: bool
    sent_setpoint: float | None  # goal sent this cycle (armed joints only)
    # Single-write move in flight: the servo paces itself and `sent_setpoint` is the final goal, so lag
    # behind it is expected. Tracking-error, stall and parked-under-load checks pause until it ends.
    free_running: bool = False
    current_cap_ma: float | None = None  # armed HLS joints: goal-current cap (stall current) in mA


@dataclass(frozen=True)
class Trip:
    scope: str  # "joint" | "all"
    servo_id: int | None
    reason: str


@dataclass
class SupervisorResult:
    trips: list[Trip] = field(default_factory=list)
    warnings: dict[int, list[str]] = field(default_factory=dict)
    freeze_motion: bool = False
    blocking: list[str] = field(default_factory=list)  # conditions that forbid arming right now
    comm_lost: bool = False
    sustained_load: dict[int, str] = field(default_factory=dict)  # joints parked under load (heat risk)

    def warn(self, servo_id: int, text: str) -> None:
        self.warnings.setdefault(servo_id, []).append(text)


class Supervisor:
    """`scales`: native steps per SCS0009 step per joint (step thresholds in MotionConfig are SCS0009 steps)."""

    def __init__(self, motion: MotionConfig, scales: Mapping[int, float] | None = None) -> None:
        self.cfg = motion
        self._scales = dict(scales or {})
        self._since: dict[tuple, float] = {}
        self._history: dict[int, deque[tuple[float, float, int]]] = {}
        self._last_setpoint: dict[int, float] = {}

    def reset(self) -> None:
        self._since.clear()
        self._history.clear()

    def _held(self, key: tuple, active: bool, now: float, window: float) -> bool:
        """True once `active` has been continuously true for `window` seconds."""
        if not active:
            self._since.pop(key, None)
            return False
        start = self._since.setdefault(key, now)
        return now - start >= window

    def evaluate(
        self, now: float, joints: Mapping[int, JointInput], heartbeat_age: float | None = None
    ) -> SupervisorResult:
        cfg = self.cfg
        out = SupervisorResult()
        any_armed = any(j.armed for j in joints.values())

        # Communication: stale or repeatedly failing data means the torque state is unknown.
        for servo_id, j in joints.items():
            stale = j.last_ok is None or now - j.last_ok > cfg.stale_s
            if j.consecutive_failures >= cfg.comm_fail_count or (stale and j.consecutive_failures > 0):
                out.comm_lost = True
                out.blocking.append(f"no reliable data from ID {servo_id}")
                if any_armed:
                    out.trips.append(Trip("all", servo_id, f"communication lost with ID {servo_id}"))

        readings = {i: j.reading for i, j in joints.items() if j.reading is not None}

        voltages = [r.voltage for r in readings.values()]
        if voltages:
            median = statistics.median(voltages)
            lo, hi = cfg.supply_voltage_window
            outside = not lo <= median <= hi
            if outside:
                out.blocking.append(f"supply {median:.1f} V outside {lo}-{hi} V")
            if self._held(("supply",), outside and any_armed, now, cfg.supply_s):
                out.trips.append(Trip("all", None, f"supply {median:.1f} V outside {lo}-{hi} V"))

        if heartbeat_age is not None and any_armed:
            if cfg.heartbeat_trip_s is not None and heartbeat_age > cfg.heartbeat_trip_s:
                out.trips.append(Trip("all", None, f"GUI unresponsive for {heartbeat_age:.1f} s"))
            elif cfg.heartbeat_freeze_s is not None and heartbeat_age > cfg.heartbeat_freeze_s:
                out.freeze_motion = True

        for servo_id, r in readings.items():
            j = joints[servo_id]
            self._check_joint(now, servo_id, j, r, out)
        return out

    def _check_joint(self, now: float, servo_id: int, j: JointInput, r: ServoReading, out: SupervisorResult) -> None:
        cfg = self.cfg
        key = lambda name: (name, servo_id)
        scale = self._scales.get(servo_id, 1.0)

        # Torque state must match what the worker believes.
        if j.armed and r.torque == 0:
            out.trips.append(Trip("all", servo_id, f"ID {servo_id} dropped torque by itself "
                                                   "(servo overload cut-out or brown-out reset)"))
        elif not j.armed and r.torque == 1:
            out.trips.append(Trip("all", servo_id, f"ID {servo_id} has torque on although not armed"))

        if r.error_byte or r.status:
            detail = describe_error(r.error_byte) or f"status 0x{r.status:02X}"
            if j.armed:
                scope = "all" if r.error_byte & ERRBIT_VOLTAGE else "joint"
                out.trips.append(Trip(scope, servo_id, f"ID {servo_id} reports {detail}"))
            else:
                out.warn(servo_id, f"servo reports {detail}")

        load = abs(r.load_pct)
        hard = self._held(key("load_hard"), j.armed and load >= cfg.load_hard_pct, now, cfg.load_hard_s)
        soft = self._held(key("load"), j.armed and load >= cfg.load_trip_pct, now, cfg.load_trip_s)
        if hard or soft:
            out.trips.append(Trip("joint", servo_id, f"ID {servo_id} load {load:.0f}% (possible interference)"))
        elif load >= cfg.load_warn_pct:
            out.warn(servo_id, f"load {load:.0f}%")

        if r.current_ma is not None and j.current_cap_ma is not None:
            limit = cfg.current_trip_frac * j.current_cap_ma
            at_cap = j.armed and abs(r.current_ma) >= limit
            if self._held(key("current"), at_cap, now, cfg.current_trip_s):
                out.trips.append(Trip("joint", servo_id, f"ID {servo_id} current {abs(r.current_ma):.0f} mA at its "
                                                         f"{j.current_cap_ma:.0f} mA cap (stalled?)"))

        hot = self._held(key("temp"), j.armed and r.temperature >= cfg.temperature_trip_c, now,
                         cfg.temperature_trip_s)
        if hot:
            out.trips.append(Trip("joint", servo_id, f"ID {servo_id} temperature {r.temperature} degC"))
        elif r.temperature >= cfg.temperature_warn_c:
            out.warn(servo_id, f"temperature {r.temperature} degC")

        if not j.armed or j.sent_setpoint is None:
            self._history.pop(servo_id, None)
            self._last_setpoint.pop(servo_id, None)
            for name in ("track", "goal", "stall", "hold"):
                self._since.pop(key(name), None)
            return

        mismatch = abs(r.goal - round(j.sent_setpoint)) > cfg.goal_mismatch_steps * scale
        if self._held(key("goal"), mismatch, now, cfg.goal_mismatch_s):
            out.trips.append(Trip("joint", servo_id, f"ID {servo_id} goal register {r.goal} does not follow "
                                                     f"the sent setpoint {j.sent_setpoint:.0f}"))

        if j.free_running:
            self._history.pop(servo_id, None)
            self._last_setpoint.pop(servo_id, None)
            for name in ("track", "stall", "hold"):
                self._since.pop(key(name), None)
            return

        # Parked short of the target while still pushing: the proportional-only servo loop cannot
        # overcome the joint's friction, and the load stays on indefinitely.
        previous = self._last_setpoint.get(servo_id)
        self._last_setpoint[servo_id] = j.sent_setpoint
        static = previous is not None and abs(j.sent_setpoint - previous) < 0.5
        short = r.position - j.sent_setpoint
        parked = (static and load >= cfg.hold_load_warn_pct and abs(short) >= cfg.hold_error_steps * scale)
        if self._held(key("hold"), parked, now, cfg.hold_warn_s):
            held_for = now - self._since[key("hold")]
            text = (f"holding {load:.0f}% load {short:+.0f} steps from its target for {held_for:.0f} s - "
                    "not reaching it (friction?); torque off to let the servo cool")
            out.sustained_load[servo_id] = text
            out.warn(servo_id, text)

        error = abs(r.position - j.sent_setpoint)
        if self._held(key("track"), error > cfg.tracking_error_steps * scale, now, cfg.tracking_error_s):
            out.trips.append(Trip("joint", servo_id, f"ID {servo_id} tracking error {error:.0f} steps"))

        history = self._history.setdefault(servo_id, deque())
        history.append((now, j.sent_setpoint, r.position))
        while history and now - history[0][0] > cfg.stall_window_s:
            history.popleft()
        if history and now - history[0][0] >= cfg.stall_window_s * 0.9:
            _, sp0, pos0 = history[0]
            commanded = abs(j.sent_setpoint - sp0)
            moved = abs(r.position - pos0)
            if (commanded > 2 * cfg.stall_min_motion_steps * scale and moved < cfg.stall_min_motion_steps * scale
                    and load > cfg.stall_load_pct):
                out.trips.append(Trip("joint", servo_id, f"ID {servo_id} stalled under {load:.0f}% load"))
