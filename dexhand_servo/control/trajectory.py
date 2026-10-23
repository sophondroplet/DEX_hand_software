"""Setpoint generation. Every motion source (slider, jog, pose, sync, playback) produces
targets; every target passes the per-joint `RateLimiter` before it is sent, so speed
and acceleration limits hold regardless of the source.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

MIN_JERK_PEAK = 1.875  # peak velocity of a minimum-jerk move = 1.875 * distance / duration
MIN_JERK_PEAK_ACC = 10 / math.sqrt(3)  # peak acceleration = 5.774 * distance / duration^2


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class RateLimiter:
    """Velocity- and acceleration-limited tracking of a target inside [lo, hi].

    The target's own velocity is fed forward. Without it, following a moving target
    (a planned trajectory) needs a lag of v^2 / (2a) steps - about 8 steps at 100 steps/s -
    which differs per joint and breaks synchronised arrival.
    """

    APPROACH_GAIN = 10.0  # 1/s: linear correction near the target, avoids chatter from sqrt()

    def __init__(self, position: float, max_speed: float, acceleration: float, lo: float, hi: float) -> None:
        self.max_speed = max_speed
        self.acceleration = acceleration
        self.lo, self.hi = lo, hi
        self.position = _clamp(position, lo, hi)
        self.target = self.position
        self._previous_target = self.target
        self.velocity = 0.0

    def set_target(self, target: float) -> None:
        self.target = _clamp(target, self.lo, self.hi)

    def reset(self, position: float) -> None:
        self.position = self.target = self._previous_target = _clamp(position, self.lo, self.hi)
        self.velocity = 0.0

    def step(self, dt: float) -> float:
        if dt <= 0:
            return self.position
        target_velocity = (self.target - self._previous_target) / dt
        self._previous_target = self.target
        error = self.target - self.position
        # Feasible target (planned trajectories are built to be): follow it exactly, no lag at all.
        # Landing on it must also leave us moving like the target itself, or we could not stop in time.
        needed = error / dt
        reach = self.acceleration * dt + 1e-9
        if (abs(needed) <= self.max_speed + 1e-9 and abs(needed - self.velocity) <= reach
                and abs(needed - target_velocity) <= reach):
            self.velocity = needed if abs(error) > 1e-12 else 0.0
            self.position = self.target
            return self.position
        moving_target = abs(target_velocity) > 1e-9
        if not moving_target and abs(error) < 0.5 and abs(self.velocity) <= self.acceleration * dt:
            self.position, self.velocity = self.target, 0.0
            return self.position
        # Approach term: fastest speed that can still stop at the target, linear when close.
        approach = math.copysign(
            min(math.sqrt(2.0 * self.acceleration * abs(error)), self.APPROACH_GAIN * abs(error)), error
        )
        desired = _clamp(target_velocity + approach, -self.max_speed, self.max_speed)
        dv = _clamp(desired - self.velocity, -self.acceleration * dt, self.acceleration * dt)
        self.velocity += dv
        new_position = self.position + self.velocity * dt
        if not moving_target and (self.target - new_position) * error < 0:
            new_position, self.velocity = self.target, 0.0  # never overshoot a stationary target
        self.position = _clamp(new_position, self.lo, self.hi)
        return self.position


def min_jerk(s: float) -> float:
    s = _clamp(s, 0.0, 1.0)
    return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)


@dataclass(frozen=True)
class Segment:
    start: Mapping[int, float]
    goal: Mapping[int, float]
    t0: float
    duration: float
    profile: str = "minjerk"  # "minjerk" | "linear"

    def sample(self, now: float) -> dict[int, float]:
        s = 1.0 if self.duration <= 0 else _clamp((now - self.t0) / self.duration, 0.0, 1.0)
        k = min_jerk(s) if self.profile == "minjerk" else s
        return {i: self.start[i] + (self.goal[i] - self.start[i]) * k for i in self.goal}

    @property
    def end(self) -> float:
        return self.t0 + self.duration


Acceleration = float | Mapping[int, float] | None  # one value, or per joint (joints in different step units)


def _per_joint(acceleration: Acceleration, servo_id: int) -> float | None:
    return acceleration.get(servo_id) if isinstance(acceleration, Mapping) else acceleration


def sync_duration(
    start: Mapping[int, float], goal: Mapping[int, float], max_speeds: Mapping[int, float],
    min_duration: float, peak_factor: float = MIN_JERK_PEAK, acceleration: Acceleration = None,
) -> float:
    """Duration so that every joint stays under its speed (and acceleration) limit and all arrive together.

    Respecting the acceleration limit here keeps the per-joint RateLimiter from engaging during a
    planned move; otherwise long moves get clipped and joints leave the shared normalised curve.
    """
    needed = [peak_factor * abs(goal[i] - start[i]) / max_speeds[i] for i in goal]
    for i in goal:
        if acc := _per_joint(acceleration, i):
            needed.append(math.sqrt(MIN_JERK_PEAK_ACC * abs(goal[i] - start[i]) / acc))
    return max([min_duration, *needed])


class MotionPlan:
    """A sequence of segments sampled by time: pose / sync moves and keyframe playback."""

    def __init__(self, segments: Sequence[Segment], label: str) -> None:
        if not segments:
            raise ValueError("empty plan")
        self.segments = list(segments)
        self.label = label
        self.joints = frozenset(i for seg in segments for i in seg.goal)

    @property
    def t0(self) -> float:
        return self.segments[0].t0

    @property
    def end(self) -> float:
        return self.segments[-1].end

    def sample(self, now: float) -> tuple[dict[int, float], float, bool]:
        """Targets, progress 0..1, done."""
        for segment in self.segments:
            if now < segment.end:
                targets = segment.sample(now)
                break
        else:
            targets = self.segments[-1].sample(self.end)
        span = self.end - self.t0
        progress = 1.0 if span <= 0 else _clamp((now - self.t0) / span, 0.0, 1.0)
        return targets, progress, now >= self.end


def plan_move(
    start: Mapping[int, float], goal: Mapping[int, float], max_speeds: Mapping[int, float],
    now: float, min_duration: float, label: str, acceleration: Acceleration = None,
) -> MotionPlan:
    goal = {i: float(goal[i]) for i in goal}
    start = {i: float(start[i]) for i in goal}
    duration = sync_duration(start, goal, max_speeds, min_duration, acceleration=acceleration)
    return MotionPlan([Segment(start, goal, now, duration)], label)


@dataclass(frozen=True)
class Keyframe:
    t: float
    positions: dict[int, int]


def compress_keyframes(frames: Sequence[Keyframe], tolerance: int | Mapping[int, int]) -> list[Keyframe]:
    """Keep a frame whenever any joint moved more than `tolerance` steps since the last kept one.

    `tolerance` is one value or one per joint (joints of different servo models use different steps).
    """
    if not frames:
        return []
    limit = (lambda i: tolerance[i]) if isinstance(tolerance, Mapping) else (lambda i: tolerance)
    kept = [frames[0]]
    for frame in frames[1:-1]:
        last = kept[-1].positions
        if any(abs(frame.positions[i] - last[i]) > limit(i) for i in frame.positions):
            kept.append(frame)
    if len(frames) > 1:
        kept.append(frames[-1])
    return kept


def plan_playback(
    keyframes: Sequence[Keyframe], start: Mapping[int, float], max_speeds: Mapping[int, float],
    now: float, time_scale: float, min_duration: float, acceleration: Acceleration = None,
) -> MotionPlan:
    """Smooth approach from the actual position to frame 0, then the recorded path.

    `time_scale` <= 1 slows playback down; every segment is stretched further if it
    would exceed a joint's speed limit.
    """
    if not keyframes:
        raise ValueError("recording has no keyframes")
    if not 0 < time_scale <= 1.0:
        raise ValueError("time_scale must be in (0, 1]")
    first = {i: float(v) for i, v in keyframes[0].positions.items()}
    segments = [Segment({i: float(start[i]) for i in first}, first, now,
                        sync_duration(start, first, max_speeds, min_duration, acceleration=acceleration))]
    for previous, current in zip(keyframes, keyframes[1:], strict=False):
        a = {i: float(v) for i, v in previous.positions.items()}
        b = {i: float(v) for i, v in current.positions.items()}
        recorded = (current.t - previous.t) / time_scale
        duration = max(recorded, sync_duration(a, b, max_speeds, 0.0, peak_factor=1.0))
        segments.append(Segment(a, b, segments[-1].end, duration, profile="linear"))
    return MotionPlan(segments, "playback")
