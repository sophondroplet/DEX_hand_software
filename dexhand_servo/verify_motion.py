"""`dexhand-servo verify-motion --id K`: check firmware behaviour the GUI driver relies on.

Runs on ONE joint, with small moves (default +-20 steps) around its present
position, under the same MotionPolicy as the GUI. Every step is announced and
confirmed by the operator; any exit (error, Ctrl-C, "n") ends with torque off.

Checks:
  A. does writing a goal with torque off switch torque on implicitly?
  B. does torque-on alone hold the present position?
  C. goal-speed register units (measured steps/s per register unit), load sign,
     and which direction is flexion (operator observation)
  D. does SYNC_WRITE at 42 (goal) and at 40 (torque off) work?
  E. optional (--overload-test): with the joint held by hand, does the servo's own
     overload cut-out release torque, and what do registers 40/65 read afterwards?
  F. optional (--speed-sweep): actual speed in both directions at several speed-register
     values, and a suggested `JointConfig.max_speed` (70 % of the slower direction at the
     highest register, which is what GUI streaming uses).
  G. optional (--single-write): ONE goal write per direction (default +-40 steps) at a few
     speed registers, sampled as fast as the bus allows, to see how the servo executes a
     long move on its own: start latency, whether it ramps up (acceleration) or jumps to
     full speed, steady speed vs register, overshoot and settling time.
Speed 0 and goal-time moves are never sent (the motion policy refuses them).

Amplitudes, step counts and speed registers given on the command line are in SCS0009 units
(~0.293 deg per step, register ~steps/s) and are converted to the joint's servo model, so the
same command moves an HLS3606 joint by the same angle at the same angular speed. Positions,
measured speeds and the suggested max_speed in the report are in the joint's own steps.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import dataclass, field, replace
from datetime import datetime
from typing import Any

from .config import JointConfig, MotionConfig
from .control.arming import arm_joint, disarm_joint, read_status, verify_identity
from .control.policy import REG_GOAL_BLOCK, MotionPolicy, limits_from_config
from .control.snapshot import ServoReading
from .control.trace import TraceRecorder, TraceRow
from .diagnostics import CensusError, ensure_census
from .models import SCS0009, ServoModel
from .registers import REG_TORQUE_ENABLE
from .safe_bus import SafeBus

Ask = Callable[[str], str]
Say = Callable[[str], None]


class VerifyAborted(RuntimeError):
    pass


@dataclass
class VerifyReport:
    servo_id: int
    joint: str
    started_at: str = field(default_factory=lambda: datetime.now().isoformat(timespec="seconds"))
    results: dict[str, Any] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)
    completed: bool = False
    torque_off_confirmed: bool | None = None

    def to_dict(self) -> dict[str, Any]:
        return dict(self.__dict__)


@dataclass
class Sample:
    t: float
    position: int
    load: float
    torque: int
    goal: int
    status: int
    current_ma: float | None = None


def _sample(bus: SafeBus, servo_id: int, t0: float, model: ServoModel = SCS0009) -> Sample | None:
    r = read_status(bus, servo_id, model)
    if r is None:
        return None
    return Sample(round(time.perf_counter() - t0, 4), r.position, r.load_pct, r.torque, r.goal, r.status,
                  r.current_ma)


def _watch(bus: SafeBus, servo_id: int, seconds: float, until: Callable[[Sample], bool] | None = None,
           interval: float = 0.005, t0: float | None = None, model: ServoModel = SCS0009) -> list[Sample]:
    t0 = time.perf_counter() if t0 is None else t0
    samples: list[Sample] = []
    while time.perf_counter() - t0 < seconds:
        s = _sample(bus, servo_id, t0, model)
        if s is not None:
            samples.append(s)
            if until is not None and until(s):
                break
        if interval:
            time.sleep(interval)
    return samples


def analyse_single_move(
    samples: list[Sample], origin: int, target: int, register: int, model: ServoModel = SCS0009
) -> dict[str, Any]:
    """Profile of one servo-executed move; times are relative to the goal write."""
    direction = 1 if target >= origin else -1
    span = abs(target - origin)
    out: dict[str, Any] = {"register": register, "from": origin, "to": target, "samples": len(samples)}
    if len(samples) < 3 or span == 0:
        return out
    progress = [(s.t, (s.position - origin) * direction) for s in samples]
    started = next((t for t, d in progress if d >= model.scs_steps(2)), None)
    out["latency_ms"] = None if started is None else round(started * 1000, 1)
    # Velocity by central difference over ~10 ms (two pointers, linear time), robust to the
    # 1-step position quantisation. Entries: (t, displacement, velocity).
    velocity: list[tuple[float, float, float]] = []
    back = ahead = 0
    for i, (t, d) in enumerate(progress):
        while back < i and t - progress[back + 1][0] >= 0.005:
            back += 1
        ahead = max(ahead, i)
        while ahead < len(progress) - 1 and progress[ahead][0] - t < 0.005:
            ahead += 1
        if t - progress[back][0] >= 0.005 and progress[ahead][0] - t >= 0.005:
            velocity.append((t, d, (progress[ahead][1] - progress[back][1]) / (progress[ahead][0] - progress[back][0])))
    middle = [v for _, d, v in velocity if 0.3 * span <= d <= 0.7 * span]
    steady = sorted(middle)[len(middle) // 2] if middle else None
    out["steady_steps_per_s"] = None if steady is None else round(steady, 1)
    out["steady_vs_register"] = None if not steady else round(steady / model.steps_per_s(register), 2)
    if steady and started is not None:
        reached_90 = next((t for t, _, v in velocity if t >= started and v >= 0.9 * steady), None)
        ramp = None if reached_90 is None else max(0.0, reached_90 - started)
        out["ramp_ms"] = None if ramp is None else round(ramp * 1000, 1)
        out["approx_acceleration"] = None if not ramp else round(0.9 * steady / ramp)
    out["overshoot_steps"] = max(0, max(d for _, d in progress) - span)
    inside = [abs(d - span) <= model.scs_steps(1) for _, d in progress]
    settle_index = next((i for i in range(len(inside)) if all(inside[i:])), None)
    out["reached"] = settle_index is not None
    out["settle_ms"] = None if settle_index is None else round(progress[settle_index][0] * 1000, 1)
    loads = [abs(s.load) for s in samples]
    out["max_load_pct"] = max(loads)
    out["trace"] = {"t": [s.t for s in samples], "position": [s.position for s in samples],
                    "load": [s.load for s in samples]}
    return out


def _rate(samples: list[Sample], origin: int, target: int) -> float | None:
    """Speed over the middle 20-80 % of a move, so start-up and settling do not distort it."""
    span = target - origin
    if not samples or span == 0:
        return None
    lo, hi = sorted((origin + 0.2 * span, origin + 0.8 * span))
    band = [s for s in samples if lo <= s.position <= hi]
    if len(band) < 2 or band[-1].t <= band[0].t:
        return None
    return round(abs(band[-1].position - band[0].position) / (band[-1].t - band[0].t), 1)


def _require(reading: ServoReading | None, what: str) -> ServoReading:
    if reading is None:
        raise VerifyAborted(f"no status reply during {what}")
    return reading


def run_verify_motion(
    bus: SafeBus,
    servo_id: int,
    joint: JointConfig,
    motion: MotionConfig,
    ask: Ask,
    say: Say,
    amplitude: int = 20,
    speed: int = 50,
    overload_test: bool = False,
    speed_sweep: tuple[int, ...] = (),
    single_write: tuple[int, ...] = (),
    single_write_steps: int = 40,
) -> VerifyReport:
    if not joint.has_travel_limits:
        raise VerifyAborted(f"ID {servo_id} has no travel limits in config.py")
    assert joint.min_position is not None and joint.max_position is not None
    try:
        ensure_census(bus)
    except CensusError as exc:
        raise VerifyAborted(f"bus census failed: {exc}") from exc
    identity, problem = verify_identity(bus, servo_id, joint)
    if identity is None:
        raise VerifyAborted(problem)
    model = identity.model
    # Command-line values are SCS0009 units: the same angles and angular speeds on every model.
    amplitude, single_write_steps = model.scs_steps(amplitude), model.scs_steps(single_write_steps)
    margin, near, moved = model.scs_steps(5), model.scs_steps(1), model.scs_steps(2)
    to_register = lambda value: model.speed_register(model.native_speed(value))  # noqa: E731
    speed_sweep = tuple(sorted({to_register(r) for r in speed_sweep}))
    single_write = tuple(sorted({to_register(r) for r in single_write}))
    # No per-packet jump limit here: the test moves are single small goals (<= 40 SCS0009 steps) by design.
    limits = limits_from_config({servo_id: joint}, motion, step_guard=False)[servo_id]
    # The operator asked for these registers explicitly; the HLS speed loop paces each small test move.
    limits = replace(limits, speed_cap=max([limits.speed_cap, *speed_sweep, *single_write]))
    policy = MotionPolicy({servo_id: limits}, known_ids=[servo_id])
    bus.set_policy(policy)
    cap = limits.speed_cap
    speed = min(to_register(speed), cap)
    report = VerifyReport(servo_id, joint.name)

    def status(what: str) -> ServoReading:
        return _require(read_status(bus, servo_id, model), what)

    def watch(seconds: float, until: Callable[[Sample], bool] | None = None, **kwargs: Any) -> list[Sample]:
        return _watch(bus, servo_id, seconds, until, model=model, **kwargs)

    def goal(position: int, register: int) -> bytes:
        return policy.encode_goal(servo_id, position, register)

    def confirm(text: str) -> None:
        if ask(f"{text} [y/N] (type y + Enter; Enter alone = no) ").strip().lower() not in ("y", "yes"):
            raise VerifyAborted("operator declined")

    def torque_off() -> bool:
        return disarm_joint(bus, policy, servo_id)

    try:
        start = status("pre-check")
        if start.torque:
            say("Torque was on; switching it off first.")
            torque_off()
            start = status("pre-check")
        lo, hi = joint.min_position + amplitude + margin, joint.max_position - amplitude - margin
        if not lo <= start.position <= hi:
            raise VerifyAborted(
                f"position {start.position} too close to the travel ends; move the joint by hand into [{lo}, {hi}]"
            )
        report.results["start"] = {"position": start.position, "goal": start.goal, "voltage": start.voltage}
        say(f"ID {servo_id} ({joint.name}, {model.label}) at {start.position}, travel [{joint.min_position}, "
            f"{joint.max_position}], moves of +-{amplitude} steps at speed register {speed} (cap {cap}).")
        confirm("Hand clear, power cut-off within reach. Start?")

        # A: implicit torque on goal write.
        policy.add_armed(servo_id, identity)
        bus.write_block(servo_id, REG_GOAL_BLOCK, goal(start.position, speed))
        after = status("test A")
        report.results["A_goal_write_enables_torque"] = bool(after.torque)
        say(f"A. goal write with torque off -> torque {'ON (implicit)' if after.torque else 'stays off'}")
        torque_off()

        # B: torque-on alone holds?
        before = status("test B")
        if abs(before.goal - before.position) > model.scs_steps(motion.arm_hold_tolerance):
            report.results["B_torque_only_hold"] = "skipped: goal register does not mirror position"
            say("B. skipped (goal register does not mirror the position)")
        else:
            policy.add_armed(servo_id, identity)
            bus.write_byte(servo_id, REG_TORQUE_ENABLE, 1)
            samples = watch(0.5)
            drift = max((abs(s.position - before.position) for s in samples), default=None)
            report.results["B_torque_only_hold"] = {"max_drift_steps": drift, "samples": len(samples)}
            say(f"B. torque on without a goal write: max drift {drift} steps")
            torque_off()

        # C: speed units, load sign, flexion direction.
        confirm(f"C. Arm and move +{amplitude} then back?")
        armed = arm_joint(bus, policy, servo_id, joint, motion)
        if not armed.ok or armed.reading is None:
            raise VerifyAborted(f"arming failed: {armed.reason}")
        base = armed.reading.position
        moves = []
        for direction in (+1, -1):
            target = base + direction * amplitude if direction > 0 else base
            origin = status("test C").position
            bus.write_block(servo_id, REG_GOAL_BLOCK, goal(target, speed))
            samples = watch(3.0, until=lambda s, g=target: abs(s.position - g) <= near)
            moving = [s for s in samples if abs(s.position - origin) >= moved and abs(s.position - target) >= moved]
            measured = _rate(samples, origin, target)
            loads = [s.load for s in moving]
            moves.append({
                "from": origin, "to": target,
                "reached": bool(samples) and abs(samples[-1].position - target) <= near,
                "measured_steps_per_s": None if measured is None else round(measured, 1),
                "mean_load_pct": round(sum(loads) / len(loads), 1) if loads else None,
            })
            if direction > 0:
                answer = ask(f"Did the joint FLEX (bend) during the +{amplitude} move? [y/n/?] ").strip().lower()
                report.results["C_plus_is_flexion"] = {"y": True, "yes": True, "n": False, "no": False}.get(answer)
        rates = [m["measured_steps_per_s"] for m in moves if m["measured_steps_per_s"]]
        ratio = round(sum(rates) / len(rates) / speed, 3) if rates else None
        report.results["C_moves"] = moves
        report.results["C_steps_per_s_per_register_unit"] = ratio
        say(f"C. measured {rates} steps/s at register {speed} -> {ratio} steps/s per unit; "
            f"load sign while moving +: {moves[0]['mean_load_pct']}")

        # D: SYNC_WRITE.
        confirm(f"D. Move +{amplitude // 2} and back using SYNC_WRITE, then SYNC_WRITE torque off?")
        mid = base + amplitude // 2
        bus.sync_write(REG_GOAL_BLOCK, 6, {servo_id: goal(mid, speed)})
        samples = watch(3.0, until=lambda s: abs(s.position - mid) <= near)
        reached = bool(samples) and abs(samples[-1].position - mid) <= near
        bus.sync_write(REG_GOAL_BLOCK, 6, {servo_id: goal(base, speed)})
        watch(3.0, until=lambda s: abs(s.position - base) <= near)
        policy.remove_armed(servo_id)
        bus.sync_write(REG_TORQUE_ENABLE, 1, {servo_id: b"\x00"})
        off = status("test D")
        report.results["D_sync_write_goal"] = reached
        report.results["D_sync_write_torque_off"] = off.torque == 0
        say(f"D. SYNC_WRITE goal {'OK' if reached else 'NOT reached'}; "
            f"SYNC_WRITE torque off {'OK' if off.torque == 0 else 'FAILED'}")

        # F: optional speed sweep.
        if speed_sweep:
            registers = sorted(speed_sweep)
            confirm(f"F. Speed sweep: move +{amplitude} and back at speed registers {registers}?")
            armed = arm_joint(bus, policy, servo_id, joint, motion)
            if not armed.ok or armed.reading is None:
                raise VerifyAborted(f"arming failed: {armed.reason}")
            base = armed.reading.position
            sweep = []
            for register in registers:
                entry: dict[str, Any] = {"register": register}
                for key, target in (("plus", base + amplitude), ("minus", base)):
                    origin = status("test F").position
                    bus.write_block(servo_id, REG_GOAL_BLOCK, goal(target, register))
                    samples = watch(3.0, until=lambda s, g=target: abs(s.position - g) <= near)
                    entry[f"{key}_steps_per_s"] = _rate(samples, origin, target)
                    entry[f"{key}_reached"] = bool(samples) and abs(samples[-1].position - target) <= near
                    entry[f"{key}_max_load_pct"] = max((abs(s.load) for s in samples), default=None)
                    time.sleep(0.2)
                sweep.append(entry)
                say(f"F. register {register:4d}: + {entry['plus_steps_per_s']} / - {entry['minus_steps_per_s']} steps/s")
            torque_off()
            top = sweep[-1]
            rates = [r for r in (top["plus_steps_per_s"], top["minus_steps_per_s"]) if r]
            # A joint that never reaches the target is force-limited (friction / tendon load vs the servo's
            # proportional-only position loop); its creep rate is not a speed limit and must not become
            # max_speed, or every synchronised move of the hand slows to that crawl.
            reached = top["plus_reached"] and top["minus_reached"]
            ceiling = math.floor(model.steps_per_s(model.max_speed_register))
            recommended = (max(model.scs_steps(10), min(ceiling, int(0.7 * min(rates))))
                           if reached and len(rates) == 2 else None)
            report.results["F_speed_sweep"] = sweep
            report.results["F_recommended_max_speed"] = recommended
            report.results["F_force_limited"] = not reached
            if recommended:
                say(f"F. suggested config: JointConfig(\"{joint.name}\", {joint.min_position}, {joint.max_position}, "
                    f"max_speed={recommended})  (70 % of the slower direction at register {top['register']})")
            elif not reached:
                loads = [top["plus_max_load_pct"], top["minus_max_load_pct"]]
                say(f"F. NO speed recommendation: the joint did not reach a {amplitude}-step target even at register "
                    f"{top['register']} (max load {loads} %). It is force-limited, not speed-limited: check tendon "
                    "friction / preload. Leave max_speed at the default meanwhile.")
            else:
                say("F. no recommendation: a direction did not produce a usable speed measurement")

        # G: optional single-write long moves.
        if single_write:
            steps = single_write_steps
            now_at = status("test G").position
            lo_g, hi_g = joint.min_position + margin, joint.max_position - steps - margin
            if not lo_g <= now_at <= hi_g:
                report.results["G_single_write"] = f"skipped: position {now_at} not within [{lo_g}, {hi_g}]"
                say(f"G. skipped: needs the joint within [{lo_g}, {hi_g}] (it is at {now_at})")
            else:
                confirm(f"G. Single-write moves of +{steps} and back at speed registers {sorted(single_write)}?")
                armed = arm_joint(bus, policy, servo_id, joint, motion)
                if not armed.ok or armed.reading is None:
                    raise VerifyAborted(f"arming failed: {armed.reason}")
                base = armed.reading.position
                moves = []
                for register in sorted(single_write):
                    for target in (base + steps, base):
                        origin = status("test G").position
                        t_cmd = time.perf_counter()
                        bus.write_block(servo_id, REG_GOAL_BLOCK, goal(target, register))
                        settled: list[float] = []

                        def done(s: Sample, g: int = target) -> bool:
                            # stop 0.3 s after first reaching the goal, to see overshoot and settling
                            if abs(s.position - g) <= near and not settled:
                                settled.append(s.t)
                            return bool(settled) and s.t - settled[0] >= 0.3

                        samples = watch(4.0, until=done, interval=0.0, t0=t_cmd)
                        move = analyse_single_move(samples, origin, target, register, model)
                        moves.append(move)
                        say(f"G. reg {register:4d} {origin}->{target}: latency {move.get('latency_ms')} ms, "
                            f"ramp {move.get('ramp_ms')} ms, steady {move.get('steady_steps_per_s')} steps/s "
                            f"(x{move.get('steady_vs_register')}), overshoot {move.get('overshoot_steps')}, "
                            f"settled {move.get('settle_ms')} ms, max load {move.get('max_load_pct')} %")
                        time.sleep(0.3)
                torque_off()
                report.results["G_single_write"] = moves

        # E: optional overload cut-out.
        if overload_test:
            confirm("E. Hold the finger FIRMLY so it cannot move, then confirm. The servo will push for up to 2.5 s.")
            armed = arm_joint(bus, policy, servo_id, joint, motion)
            if not armed.ok or armed.reading is None:
                raise VerifyAborted(f"arming failed: {armed.reason}")
            push = armed.reading.position + amplitude
            bus.write_block(servo_id, REG_GOAL_BLOCK, goal(push, speed))
            samples = watch(2.5, until=lambda s: s.torque == 0)
            torque_off()
            currents = [abs(s.current_ma) for s in samples if s.current_ma is not None]
            report.results["E_overload"] = {
                "max_load_pct": max((abs(s.load) for s in samples), default=None),
                "max_current_ma": max(currents, default=None),
                "current_cap_ma": (None if identity.current_cap is None
                                   else identity.current_cap * (model.current_ma_per_unit or 0)),
                "servo_released_torque": any(s.torque == 0 for s in samples),
                "released_after_s": next((s.t for s in samples if s.torque == 0), None),
                "status_values": sorted({s.status for s in samples}),
            }
            say(f"E. {report.results['E_overload']}")
        report.completed = True
    except KeyboardInterrupt:
        report.notes.append("interrupted by operator")
    except VerifyAborted as exc:
        report.notes.append(f"aborted: {exc}")
    finally:
        report.torque_off_confirmed = torque_off()
    return report


def single_write_recorder(moves: list[dict[str, Any]]) -> TraceRecorder | None:
    """Test G as a trace (one pseudo-joint per move), so the standard trace HTML can plot it."""
    recorder = TraceRecorder("single-write", 0.0, {}, {}, tolerance=1.0)
    for index, move in enumerate(moves, start=1):
        trace = move.get("trace")
        if not trace:
            continue
        goal = float(move["to"])
        recorder.goals[index] = goal
        recorder.names[index] = f"reg {move['register']}: {move['from']} -> {move['to']}"
        for t, position, load in zip(trace["t"], trace["position"], trace["load"], strict=False):
            recorder.rows.append(TraceRow(t, index, goal, goal, position, None, load, 1, move["register"]))
    return recorder if recorder.goals else None
