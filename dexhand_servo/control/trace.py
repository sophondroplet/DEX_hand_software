"""Motion traces: what was commanded vs what the joints did, for every pose / sync / playback move.

One row per joint per control cycle. `position` and `goal_register` come from the read at the
start of the cycle, so they reflect the setpoint sent one cycle earlier (~1/cycle_hz lag).
"""

from __future__ import annotations

import csv
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path

from .snapshot import ServoReading


@dataclass(frozen=True)
class TraceRow:
    t: float
    servo_id: int
    target: float | None  # plan output before the rate limiter
    setpoint: float | None  # value sent this cycle
    position: int | None
    goal_register: int | None
    load_pct: float | None
    torque: int | None
    speed_register: int | None = None  # speed sent with this cycle's goal


@dataclass(frozen=True)
class JointArrival:
    servo_id: int
    name: str
    start: float
    goal: float
    arrival_s: float | None  # first time the joint stayed within tolerance of its goal
    final_error: float | None
    max_tracking_error: float | None  # |position - previous setpoint|
    max_load_pct: float | None
    max_goal_step: float | None = None  # largest goal change between two cycles (stair height)
    stalled_pct: float | None = None  # cycles where the goal advanced but the joint did not move


@dataclass
class TraceRecorder:
    label: str
    t0: float
    goals: dict[int, float]
    names: dict[int, str]
    tolerance: float = 3.0
    tolerances: dict[int, float] = field(default_factory=dict)  # per joint (own step units); else `tolerance`
    rows: list[TraceRow] = field(default_factory=list)
    plan_end: float | None = None  # clock time the plan finished (tail recording continues)
    stopped: bool = False

    def add(self, now: float, servo_id: int, target: float | None, setpoint: float | None,
            reading: ServoReading | None, speed_register: int | None = None) -> None:
        self.rows.append(TraceRow(
            round(now - self.t0, 4), servo_id, target, setpoint,
            reading.position if reading else None,
            reading.goal if reading else None,
            reading.load_pct if reading else None,
            reading.torque if reading else None,
            speed_register,
        ))

    def arrivals(self) -> list[JointArrival]:
        out = []
        for servo_id, goal in self.goals.items():
            rows = [r for r in self.rows if r.servo_id == servo_id]
            positions = [(r.t, r.position) for r in rows if r.position is not None]
            tolerance = self.tolerances.get(servo_id, self.tolerance)
            # Arrival = first sample after the last one outside tolerance (and the joint ends inside it).
            arrival = None
            if positions and abs(positions[-1][1] - goal) <= tolerance:
                last_out = max((i for i, (_, p) in enumerate(positions) if abs(p - goal) > tolerance),
                               default=-1)
                arrival = positions[last_out + 1][0]
            previous_setpoint = None
            tracking = []
            for r in rows:
                if r.position is not None and previous_setpoint is not None:
                    tracking.append(abs(r.position - previous_setpoint))
                if r.setpoint is not None:
                    previous_setpoint = r.setpoint
            loads = [abs(r.load_pct) for r in rows if r.load_pct is not None]
            steps, advanced, stalled = [], 0, 0
            for a, b in zip(rows, rows[1:], strict=False):
                if a.setpoint is None or b.setpoint is None:
                    continue
                step = abs(b.setpoint - a.setpoint)
                steps.append(step)
                if step >= 1 and a.position is not None and b.position is not None:
                    advanced += 1
                    stalled += a.position == b.position
            out.append(JointArrival(
                servo_id=servo_id,
                name=self.names.get(servo_id, str(servo_id)),
                start=float(positions[0][1]) if positions else float("nan"),
                goal=goal,
                arrival_s=arrival,
                final_error=(positions[-1][1] - goal) if positions else None,
                max_tracking_error=max(tracking) if tracking else None,
                max_load_pct=max(loads) if loads else None,
                max_goal_step=max(steps) if steps else None,
                stalled_pct=round(100.0 * stalled / advanced, 1) if advanced else None,
            ))
        return out

    def summary_line(self) -> str:
        arrivals = self.arrivals()
        times = [a.arrival_s for a in arrivals if a.arrival_s is not None]
        missing = [a.servo_id for a in arrivals if a.arrival_s is None]
        parts = [f"ID{a.servo_id} {a.arrival_s:.2f}s" if a.arrival_s is not None else f"ID{a.servo_id} not reached"
                 for a in arrivals]
        spread = f"arrival spread {max(times) - min(times):.2f} s" if len(times) > 1 else ""
        worst = max((a.max_tracking_error or 0 for a in arrivals), default=0)
        stair = max((a.max_goal_step or 0 for a in arrivals), default=0)
        stalls = [a for a in arrivals if a.stalled_pct is not None]
        stall = max(stalls, key=lambda a: a.stalled_pct) if stalls else None  # type: ignore[arg-type,return-value]
        stall_text = f"; most stalled ID{stall.servo_id} {stall.stalled_pct:.0f}%" if stall else ""
        tail = f"; not within {self.tolerance:.0f} steps: IDs {missing}" if missing else ""
        return (f"trace '{self.label}'{' (stopped)' if self.stopped else ''}: {spread}; "
                f"max tracking error {worst:.0f} steps; max goal step {stair:.0f}{stall_text}; "
                + ", ".join(parts) + tail)


def write_trace_csv(recorder: TraceRecorder, path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t_s", "id", "target", "setpoint", "position", "goal_register", "load_pct", "torque",
                         "speed_register"])
        for r in recorder.rows:
            writer.writerow([r.t, r.servo_id, _fmt(r.target), _fmt(r.setpoint), r.position, r.goal_register,
                             r.load_pct, r.torque, r.speed_register])
    return path


def _fmt(value: float | None) -> str:
    return "" if value is None else f"{value:.2f}"


def trace_payload(recorder: TraceRecorder, joint_order: Sequence[int], extra: Mapping[str, object]) -> dict:
    arrivals = {a.servo_id: a for a in recorder.arrivals()}
    joints = []
    for servo_id in joint_order:
        if servo_id not in recorder.goals:
            continue
        rows = [r for r in recorder.rows if r.servo_id == servo_id]
        a = arrivals[servo_id]
        joints.append({
            "id": servo_id,
            "name": a.name,
            "t": [r.t for r in rows],
            "setpoint": [None if r.setpoint is None else round(r.setpoint, 2) for r in rows],
            "position": [r.position for r in rows],
            "load": [r.load_pct for r in rows],
            "speed": [r.speed_register for r in rows],
            "summary": {
                "start": a.start, "goal": a.goal, "arrival_s": a.arrival_s, "final_error": a.final_error,
                "max_tracking_error": a.max_tracking_error, "max_load_pct": a.max_load_pct,
                "max_goal_step": a.max_goal_step, "stalled_pct": a.stalled_pct,
            },
        })
    return {
        "label": recorder.label,
        "stopped": recorder.stopped,
        "plan_duration_s": None if recorder.plan_end is None else round(recorder.plan_end - recorder.t0, 3),
        "tolerance": recorder.tolerance,
        "joints": joints,
        **extra,
    }
