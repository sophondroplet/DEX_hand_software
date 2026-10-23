"""The control worker: the only thread that touches the servo bus while the GUI runs.

Cycle (default 50 Hz):
  1. service E-stop            (also checked before every single transaction)
  2. apply queued commands     (arm/disarm, mode, moves, recording, ...)
  3. read the status block of every joint (40..66, HLS 40..70; also confirms the previous SYNC_WRITE)
  4. safety supervisor -> trips (torque off + latched fault)
  5. advance motion            (plan / slider / jog targets -> per-joint RateLimiter)
  6. one SYNC_WRITE of setpoints, only to armed joints whose torque was just read as on
     (a single-write pose instead sends its whole goal once and is then left to the servos)
  7. publish an immutable HandSnapshot

Reading before writing matters: if a goal write implicitly re-enables torque on
this firmware, writing first would silently undo a servo's own overload cut-out
before the supervisor could see it.

Joints may use different servo models: positions and speeds stay in each joint's own
units, MotionConfig step values (SCS0009 units) are converted per joint, and every
goal block is encoded by the policy for its joint's model.

Joints that do not reply at startup are OFFLINE: they are not polled, not supervised, not
counted in torque-off confirmation and cannot be armed, so the GUI stays usable with an
incomplete servo set. `RescanBus` (only while nothing is armed) refreshes the online set and
the HLS secondary-ID census. A joint that drops out after being online is still treated as a
communication loss by the supervisor.

The GUI only calls the thread-safe methods: estop, submit, set_target, heartbeat,
snapshot, drain_events, take_recording, start, stop.
"""

from __future__ import annotations

import queue
import threading
import time
from collections import deque
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from datetime import datetime
from enum import Enum
from pathlib import Path

from ..config import JointConfig, MotionConfig
from ..diagnostics import CensusError, ensure_census
from ..registers import REG_TORQUE_ENABLE
from ..safe_bus import SafeBus, UnsafeOperationError
from .arming import arm_joint, disarm_joint, read_status
from .policy import GOAL_BLOCK_LEN, REG_GOAL_BLOCK, MotionPolicy, limits_from_config
from .snapshot import Event, HandSnapshot, JointState, ServoReading
from .supervisor import JointInput, Supervisor, SupervisorResult, Trip
from .trace import TraceRecorder, trace_payload, write_trace_csv
from .trajectory import Keyframe, MotionPlan, RateLimiter, compress_keyframes, plan_move, plan_playback

TRACE_TAIL_S = 0.6  # keep recording after a planned move ends, to see the joints settle
PLAN_ACCEL_FRACTION = 0.9  # plans stay under the limiter's acceleration so it never reshapes them
BUSY_WARN_FRACTION = 0.8
BUSY_WARN_CYCLES = 25
# Single-write poses. verify-motion G: the servo starts 50-100 ms after the goal write.
SINGLE_WRITE_LATENCY_S = 0.1
SINGLE_WRITE_ARRIVAL_STEPS = 3
SINGLE_WRITE_TIMEOUT_FACTOR = 2.0
SINGLE_WRITE_TIMEOUT_MARGIN_S = 1.0


class Mode(str, Enum):
    SLIDER = "slider"
    POSE = "pose"
    SYNC = "sync"
    JOG = "jog"
    TEACH = "teach"


# ----------------------------------------------------------------------------- commands


@dataclass(frozen=True)
class ArmJoints:
    ids: tuple[int, ...]


@dataclass(frozen=True)
class DisarmJoints:
    ids: tuple[int, ...]


@dataclass(frozen=True)
class ResetFaults:
    pass


@dataclass(frozen=True)
class SetMode:
    mode: Mode


@dataclass(frozen=True)
class MoveTo:
    targets: dict[int, int]
    label: str
    # True: one SYNC_WRITE of the final goals at each joint's max_speed; the servos pace the move.
    single_write: bool = False


@dataclass(frozen=True)
class Jog:
    servo_id: int
    delta: int


@dataclass(frozen=True)
class StartRecording:
    pass


@dataclass(frozen=True)
class StopRecording:
    pass


@dataclass(frozen=True)
class Playback:
    keyframes: tuple[Keyframe, ...]
    time_scale: float


@dataclass(frozen=True)
class StopMotion:
    pass


@dataclass(frozen=True)
class RescanBus:
    pass


@dataclass
class SingleWriteMove:
    """A pose sent as one SYNC_WRITE. The bus write happens in the cycle's send phase, after the read."""

    label: str
    goals: dict[int, int]
    speeds: dict[int, int]
    sent_at: float | None = None  # None: not sent yet
    expected_s: float = 0.0
    deadline: float = 0.0
    stop_reason: str | None = None  # set: hold where the joints are at the next send phase


Command = (ArmJoints | DisarmJoints | ResetFaults | SetMode | MoveTo | Jog | StartRecording | StopRecording
           | Playback | StopMotion | RescanBus)


class Controller:
    def __init__(
        self,
        bus: SafeBus,
        joints: Mapping[int, JointConfig],
        motion: MotionConfig,
        *,
        clock: Callable[[], float] = time.perf_counter,
        sleep: Callable[[float], None] = time.sleep,
        log_path: Path | None = None,
        trace_dir: Path | None = None,
    ) -> None:
        self.bus = bus
        self.joints = dict(joints)
        self.motion = motion
        if motion.stream_speed_mode not in ("dynamic", "fixed"):
            raise ValueError(f"stream_speed_mode must be 'dynamic' or 'fixed', got {motion.stream_speed_mode!r}")
        self._models = {i: joint.servo_model for i, joint in self.joints.items()}
        self.policy = MotionPolicy(limits_from_config(self.joints, motion), known_ids=list(self.joints),
                                   models=self._models)
        bus.set_policy(self.policy)
        bus.keep_history = False
        self._clock = clock
        self._sleep = sleep
        self._log_path = log_path
        self._trace_dir = trace_dir

        self._estop = threading.Event()
        self._estop_reason = "operator"
        self._stop = threading.Event()
        self._commands: queue.Queue[Command] = queue.Queue()
        self._slider_lock = threading.Lock()
        self._slider_targets: dict[int, float] = {}
        self._snapshot_lock = threading.Lock()
        self._snapshot: HandSnapshot | None = None
        self._events_lock = threading.Lock()
        self._events: deque[Event] = deque(maxlen=1000)
        self._heartbeat = clock()
        self._thread: threading.Thread | None = None

        # Worker-thread state.
        self.mode = Mode.SLIDER
        self._supervisor = Supervisor(motion, scales={i: m.step_scale for i, m in self._models.items()})
        self._last_result = SupervisorResult()
        self._limiters: dict[int, RateLimiter] = {}
        self._plan: MotionPlan | None = None
        self._plan_progress: float | None = None
        self._single: SingleWriteMove | None = None
        self._sent: dict[int, float] = {}
        self._readings: dict[int, ServoReading | None] = dict.fromkeys(self.joints)
        self._fail_counts: dict[int, int] = dict.fromkeys(self.joints, 0)
        self._last_ok: dict[int, float | None] = dict.fromkeys(self.joints)
        self._joint_faults: dict[int, str] = {}
        self._online: set[int] = set(self.joints)  # decided in startup(), refreshed by RescanBus
        self._torque_off_replies: dict[int, ServoReading | None] = {}
        self._global_faults: list[str] = []
        self._estop_latched = False
        self._estop_confirmed: bool | None = None
        self._torque_off_confirmed: bool | None = None
        self._recording: list[Keyframe] | None = None
        self._record_t0 = 0.0
        self._last_record_t = -1.0
        self._finished_recording: list[Keyframe] | None = None
        self._fresh: dict[int, ServoReading | None] = {}
        self._plan_targets: dict[int, float] = {}
        self._sent_now: dict[int, float] = {}
        self._speed_now: dict[int, int] = {}
        self._trace: TraceRecorder | None = None
        self._trace_mode = motion.stream_speed_mode
        self._busy_ms = 0.0
        self._busy_streak = 0
        self._sustained_warned: set[int] = set()
        self._last_cycle: float | None = None
        self._cycle_hz = 0.0
        self.cycles = 0
        self.overruns = 0

    # ------------------------------------------------------------------ thread-safe API

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="dexhand-control", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        """Torque off everything, then end the worker and close the port."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout)

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def estop(self, reason: str = "operator") -> None:
        self._estop_reason = reason
        self._estop.set()

    def submit(self, command: Command) -> None:
        self._commands.put(command)

    def set_target(self, servo_id: int, position: float) -> None:
        """Latest-wins slider target; ignored unless the joint is armed in slider mode."""
        with self._slider_lock:
            self._slider_targets[servo_id] = position

    def heartbeat(self) -> None:
        self._heartbeat = self._clock()

    def snapshot(self) -> HandSnapshot | None:
        with self._snapshot_lock:
            return self._snapshot

    def drain_events(self) -> list[Event]:
        with self._events_lock:
            events = list(self._events)
            self._events.clear()
        return events

    def take_recording(self) -> list[Keyframe] | None:
        recording, self._finished_recording = self._finished_recording, None
        return recording

    # ------------------------------------------------------------------ loop

    def startup(self) -> bool:
        """Bring every servo to a known state (torque off) before the first cycle.

        A previous session may have crashed with joints holding; starting from torque-off
        means nothing moves until the operator arms joints explicitly. Joints that do not
        reply become offline.
        """
        self._online = set(self.joints)
        self._torque_off_all()
        replies = self._torque_off_replies
        self._online = {i for i, reading in replies.items() if reading is not None}
        offline = sorted(set(self.joints) - self._online)
        for servo_id in offline:
            self._fail_counts[servo_id] = 0
        confirmed = bool(self._online) and all(replies[i].torque == 0 for i in self._online)  # type: ignore[union-attr]
        self._torque_off_confirmed = confirmed
        hint = f"; offline (no reply): {offline} - connect them and press Rescan bus" if offline else ""
        if not self._online:
            self._emit("error", "startup: no joint replied (check servo power and wiring)")
        elif confirmed:
            self._emit("info", f"startup: torque off confirmed on {len(self._online)} joint(s){hint}")
        else:
            self._emit("error", f"startup: could not confirm torque off on every responding joint{hint}")
        self._take_census("startup")
        return confirmed

    def _take_census(self, context: str) -> None:
        try:
            census = ensure_census(self.bus)
        except CensusError as exc:
            self._emit("error", f"{context}: bus census failed, arming is refused until a successful rescan: {exc}")
            return
        shared = {i: sorted(census.get(i, frozenset()) - {i}) for i in self.joints}
        for servo_id, others in shared.items():
            if others:
                self._emit("error", f"{context}: joint ID {servo_id} is also the secondary ID of HLS servo(s) "
                                    f"{others}; it cannot be armed")
        self._emit("info", f"{context}: bus census done, "
                           + (f"HLS secondary IDs {sorted(census)}" if census else "no HLS servo found"))

    def _run(self) -> None:
        period = 1.0 / self.motion.cycle_hz
        self._emit("info", f"control loop started on {self.bus.port_name} at {self.motion.cycle_hz:.0f} Hz")
        try:
            self.startup()
        except BaseException as exc:  # noqa: BLE001
            self._emit("error", f"startup failed: {exc!r}")
        next_cycle = self._clock()
        while not self._stop.is_set():
            cycle_start = self._clock()
            try:
                self.run_cycle()
            except BaseException as exc:  # noqa: BLE001 - the worker must never die silently
                self._emit("error", f"internal error in control cycle: {exc!r}")
                try:
                    self._latch_global(f"internal error: {exc!r}")
                    self._torque_off_all()
                except BaseException as nested:  # noqa: BLE001
                    self._emit("error", f"torque-off after internal error failed: {nested!r}")
            self._account_busy(self._clock() - cycle_start, period)
            next_cycle += period
            delay = next_cycle - self._clock()
            if delay > 0:
                self._sleep(delay)
            elif -delay > period:
                self.overruns += 1
                next_cycle = self._clock()
        try:
            confirmed = self._torque_off_all()
            self._emit("info" if confirmed else "error",
                       "shutdown: torque off confirmed" if confirmed else "shutdown: torque off NOT confirmed")
        finally:
            self.bus.close()

    def run_cycle(self) -> None:
        """One control cycle; public so tests can drive the controller without a thread."""
        now = self._clock()
        dt = 0.0 if self._last_cycle is None else min(0.1, now - self._last_cycle)
        if self._last_cycle is not None and dt > 0:
            self._cycle_hz = 0.9 * self._cycle_hz + 0.1 / dt if self._cycle_hz else 1.0 / dt
        self._last_cycle = now
        self.cycles += 1

        self._service_estop()
        self._process_commands(now)
        self._service_estop()

        self._read_all(now)
        self._service_estop()
        self._supervise(now)

        frozen = self._last_result.freeze_motion
        setpoints = {} if frozen else self._advance_motion(now, dt)
        confirmed = {i: sp for i, sp in setpoints.items()
                     if (r := self._fresh.get(i)) is not None and r.torque == 1}
        self._sent_now = {}
        self._speed_now = {}
        if self._single is not None and not self._estop.is_set():
            self._service_single_write(now, frozen)
        if confirmed and not self._estop.is_set():
            self._send_setpoints(confirmed)
        self._trace_cycle(now)
        self._record(now)
        self._publish(now)

    # ------------------------------------------------------------------ E-stop / torque off

    def _service_estop(self) -> bool:
        if not self._estop.is_set():
            return False
        confirmed = self._torque_off_all()
        self._estop_latched = True
        self._estop_confirmed = confirmed
        self._drain_commands()
        with self._slider_lock:
            self._slider_targets.clear()
        self._estop.clear()
        if confirmed:
            self._emit("error", f"E-STOP ({self._estop_reason}): torque off confirmed on all joints")
        else:
            self._emit("error", f"E-STOP ({self._estop_reason}): torque off NOT confirmed - cut servo power")
        return True

    def _torque_off_all(self) -> bool:
        """Disarm first (no goal can follow), then one SYNC_WRITE plus acknowledged per-ID writes."""
        self.policy.clear_armed()
        self._limiters.clear()
        self._plan = None
        self._plan_progress = None
        self._single = None
        self._sent.clear()
        self._finish_trace(stopped=True)
        # The broadcast covers offline joints too; acknowledged writes and read-backs go to online joints only,
        # so an E-stop never waits for reply timeouts of servos that are not there.
        self.bus.sync_write(REG_TORQUE_ENABLE, 1, {i: b"\x00" for i in self.joints})
        ids = [i for i in self.joints if i in self._online]
        for servo_id in ids:
            self.bus.write_byte(servo_id, REG_TORQUE_ENABLE, 0)
        confirmed = True
        self._torque_off_replies = {}
        for servo_id in ids:
            reading = read_status(self.bus, servo_id, self._models[servo_id])
            self._torque_off_replies[servo_id] = reading
            self._store_reading(servo_id, reading, self._clock())
            confirmed &= reading is not None and reading.torque == 0
        self._torque_off_confirmed = confirmed
        return confirmed

    def _drain_commands(self) -> None:
        while True:
            try:
                self._commands.get_nowait()
            except queue.Empty:
                return

    # ------------------------------------------------------------------ commands

    def _process_commands(self, now: float) -> None:
        for _ in range(64):  # bounded so a flood of commands cannot starve the cycle
            try:
                command = self._commands.get_nowait()
            except queue.Empty:
                return
            if self._estop.is_set():
                return
            self._handle(command, now)

    def _handle(self, command: Command, now: float) -> None:
        if isinstance(command, ArmJoints):
            ids = command.ids
            if len(ids) > 1:
                skipped = [i for i in ids if i not in self._online]
                if skipped:
                    self._emit("info", f"arm: offline IDs {skipped} skipped")
                ids = tuple(i for i in ids if i in self._online)
            for servo_id in ids:
                self._arm(servo_id)
        elif isinstance(command, DisarmJoints):
            for servo_id in command.ids:
                if servo_id in self._online or servo_id in self.policy.armed:
                    self._disarm(servo_id, "operator")
        elif isinstance(command, ResetFaults):
            self._reset_faults()
        elif isinstance(command, SetMode):
            try:
                mode = Mode(command.mode)
            except ValueError:
                self._emit("warn", f"unknown mode {command.mode!r} ignored")
                return
            self._set_mode(mode)
        elif isinstance(command, MoveTo):
            self._move_to(command, now)
        elif isinstance(command, Jog):
            self._jog(command)
        elif isinstance(command, StartRecording):
            self._start_recording(now)
        elif isinstance(command, StopRecording):
            self._stop_recording()
        elif isinstance(command, Playback):
            self._playback(command, now)
        elif isinstance(command, StopMotion):
            self._stop_motion("operator")
        elif isinstance(command, RescanBus):
            self._rescan()

    def _arm(self, servo_id: int) -> None:
        name = self._name(servo_id)
        reason = None
        if self._estop_latched:
            reason = "E-stop is latched; reset faults first"
        elif self._global_faults:
            reason = f"fault latched ({self._global_faults[-1]}); reset faults first"
        elif servo_id in self._joint_faults:
            reason = f"joint fault latched ({self._joint_faults[servo_id]}); reset faults first"
        elif self._last_result.blocking:
            reason = "; ".join(self._last_result.blocking)
        elif self._recording is not None:
            reason = "recording in progress (teach needs torque off)"
        elif servo_id in self.policy.armed:
            return
        elif servo_id not in self._online:
            reason = "offline (no reply at startup); connect it and press Rescan bus"
        if reason:
            self._emit("warn", f"arm {name} refused: {reason}")
            return
        result = arm_joint(self.bus, self.policy, servo_id, self.joints[servo_id], self.motion)
        if not result.ok or result.reading is None:
            self._emit("warn", f"arm {name} failed: {result.reason}")
            return
        joint = self.joints[servo_id]
        assert joint.min_position is not None and joint.max_position is not None
        position = float(result.reading.position)
        self._limiters[servo_id] = RateLimiter(position, joint.max_speed, self._acceleration(servo_id),
                                               joint.min_position, joint.max_position)
        self._sent[servo_id] = position
        self._emit("info", f"{name} armed, holding {result.reading.position}")

    def _disarm(self, servo_id: int, why: str) -> None:
        if self._plan is not None and servo_id in self._plan.joints:
            label = self._plan_label()
            self._plan = None
            self._plan_progress = None
            self._finish_trace(stopped=True)
            self._emit("warn", f"{label} stopped: {self._name(servo_id)} disarmed")
        single = self._single
        if single is not None and servo_id in single.goals:
            del single.goals[servo_id], single.speeds[servo_id]
            if not single.goals:
                self._single = None
                self._finish_trace(stopped=True)
                self._emit("warn", f"single-write move '{single.label}' stopped: {self._name(servo_id)} disarmed")
            elif single.stop_reason is None:
                single.stop_reason = f"{self._name(servo_id)} disarmed"
        self._limiters.pop(servo_id, None)
        self._sent.pop(servo_id, None)
        confirmed = disarm_joint(self.bus, self.policy, servo_id)
        level = "info" if confirmed else "error"
        self._emit(level, f"{self._name(servo_id)} torque off ({why})" + ("" if confirmed else " NOT confirmed"))

    def _rescan(self) -> None:
        """Refresh the online set and the HLS secondary-ID census; only while nothing moves or is armed."""
        if self.policy.armed:
            busy = "disarm all joints first"
        elif self._recording is not None:
            busy = "recording in progress"
        elif self._plan is not None or self._single is not None:
            busy = "a move is in progress"
        else:
            busy = None
        if busy:
            self._emit("warn", f"rescan refused: {busy}")
            return
        now = self._clock()
        online = set()
        for servo_id in self.joints:
            model = self._models[servo_id]
            reading = read_status(self.bus, servo_id, model) or read_status(self.bus, servo_id, model)
            if reading is not None:
                online.add(servo_id)
                self._store_reading(servo_id, reading, now)
            else:
                self._readings[servo_id] = None
                self._fail_counts[servo_id] = 0
        joined = sorted(online - self._online)
        self._online = online
        for servo_id in joined:
            # A servo plugged in mid-session may hold torque from an earlier session.
            self.bus.write_byte(servo_id, REG_TORQUE_ENABLE, 0)
        self._supervisor.reset()
        self.bus.census = None
        offline = sorted(set(self.joints) - online)
        self._emit("info", f"rescan: online {sorted(online)}" + (f", newly connected {joined}" if joined else "")
                           + (f"; offline {offline}" if offline else ""))
        self._take_census("rescan")

    def _reset_faults(self) -> None:
        self._estop_latched = False
        self._estop_confirmed = None
        self._global_faults.clear()
        self._joint_faults.clear()
        self._supervisor.reset()
        self._emit("info", "faults reset (joints stay disarmed; arm them again explicitly)")

    def _set_mode(self, mode: Mode) -> None:
        if mode == self.mode:
            return
        self._stop_motion(f"mode change to {mode.value}")
        if self._recording is not None:
            self._stop_recording()
        with self._slider_lock:
            self._slider_targets.clear()
        self.mode = mode
        self._emit("info", f"mode: {mode.value}")

    def _stop_motion(self, why: str) -> None:
        if self._single is not None and self._single.stop_reason is None:
            self._single.stop_reason = why
        if self._plan is not None:
            self._emit("info", f"{self._plan_label()} stopped ({why})")
            self._finish_trace(stopped=True)
        self._plan = None
        self._plan_progress = None
        for limiter in self._limiters.values():
            stopping = limiter.velocity * abs(limiter.velocity) / (2.0 * limiter.acceleration)
            limiter.set_target(limiter.position + stopping)

    def _check_targets(self, targets: Mapping[int, float], what: str) -> dict[int, float] | None:
        armed = {i: float(v) for i, v in targets.items() if i in self.policy.armed}
        skipped = sorted(set(targets) - set(armed))
        outside = [i for i, v in armed.items() if not self._within(i, v)]
        if outside:
            self._emit("warn", f"{what} refused: targets outside travel for IDs {outside}")
            return None
        if not armed:
            self._emit("warn", f"{what} refused: none of its joints is armed")
            return None
        if skipped:
            self._emit("warn", f"{what}: IDs {skipped} not armed, left where they are")
        return armed

    def _move_to(self, command: MoveTo, now: float) -> None:
        if self.mode not in (Mode.POSE, Mode.SYNC):
            self._emit("warn", f"move '{command.label}' ignored outside pose/sync mode")
            return
        if self._single is not None:
            self._emit("warn", f"move '{command.label}' refused: single-write move '{self._single.label}' "
                               "in progress (Stop motion first)")
            return
        goal = self._check_targets(command.targets, f"move '{command.label}'")
        if goal is None:
            return
        if command.single_write:
            registers = {i: self.policy.limits[i].max_speed for i in goal}
            too_slow = sorted(i for i, register in registers.items() if register is None)
            if too_slow:
                self._emit("warn", f"single-write move '{command.label}' refused: max_speed of IDs {too_slow} is "
                                   "below one speed-register unit")
                return
            self._stop_motion(f"single-write move '{command.label}'")
            ids = list(goal)
            self._single = SingleWriteMove(command.label, {i: round(goal[i]) for i in ids},
                                           {i: registers[i] for i in ids})  # type: ignore[misc]
            self._start_trace(command.label, goal, now, mode="single write")
            self._plan_targets = dict(goal)
            return
        start = {i: self._limiters[i].position for i in goal}
        speeds = {i: float(self.joints[i].max_speed) for i in goal}
        self._plan = plan_move(start, goal, speeds, now, self.motion.min_move_duration_s, command.label,
                               acceleration={i: self._acceleration(i) * PLAN_ACCEL_FRACTION for i in goal})
        self._start_trace(command.label, goal, now)
        span = self._plan.end - now
        self._emit("info", f"move '{command.label}' started ({len(goal)} joints, {span:.1f} s)")

    def _jog(self, command: Jog) -> None:
        limiter = self._limiters.get(command.servo_id)
        if self.mode is not Mode.JOG or limiter is None:
            self._emit("warn", f"jog {self._name(command.servo_id)} ignored (jog mode and armed joint required)")
            return
        # Jog steps are SCS0009 steps: the same angle on every model.
        delta = self._models[command.servo_id].scs_steps(abs(command.delta))
        limiter.set_target(limiter.target + (delta if command.delta > 0 else -delta))

    def _start_recording(self, now: float) -> None:
        if self.mode is not Mode.TEACH:
            self._emit("warn", "recording needs teach mode")
            return
        if self.policy.armed:
            self._emit("warn", "recording refused: disarm all joints first (teach is done by hand, torque off)")
            return
        self._recording = []
        self._record_t0 = now
        self._last_record_t = -1.0
        self._emit("info", "recording started - move the hand by hand")

    def _stop_recording(self) -> None:
        if self._recording is None:
            return
        frames = self._recording
        self._recording = None
        keyframes = compress_keyframes(frames, {i: m.scs_steps(self.motion.keyframe_tolerance_steps)
                                                for i, m in self._models.items()})
        self._finished_recording = keyframes
        outside = sorted({i for kf in keyframes for i, v in kf.positions.items() if not self._within(i, v)})
        note = f"; positions outside travel on IDs {outside} (cannot be saved)" if outside else ""
        duration = frames[-1].t if frames else 0.0
        self._emit("info", f"recording stopped: {len(frames)} samples, {duration:.1f} s, "
                           f"{len(keyframes)} keyframes{note}")

    def _playback(self, command: Playback, now: float) -> None:
        if self.mode is not Mode.TEACH:
            self._emit("warn", "playback needs teach mode")
            return
        if self._recording is not None:
            self._emit("warn", "playback refused while recording")
            return
        if not command.keyframes:
            self._emit("warn", "playback refused: empty recording")
            return
        joints = set(command.keyframes[0].positions)
        missing = sorted(joints - self.policy.armed)
        if missing:
            self._emit("warn", f"playback refused: arm IDs {missing} first")
            return
        outside = sorted({i for kf in command.keyframes for i, v in kf.positions.items() if not self._within(i, v)})
        if outside:
            self._emit("warn", f"playback refused: keyframes outside travel for IDs {outside}")
            return
        scale = min(command.time_scale, self.motion.playback_max_time_scale)
        start = {i: self._limiters[i].position for i in joints}
        speeds = {i: float(self.joints[i].max_speed) for i in joints}
        try:
            self._plan = plan_playback(command.keyframes, start, speeds, now, scale, self.motion.min_move_duration_s,
                                       acceleration={i: self._acceleration(i) * PLAN_ACCEL_FRACTION for i in joints})
        except ValueError as exc:
            self._emit("warn", f"playback refused: {exc}")
            return
        self._start_trace("playback", {i: float(v) for i, v in command.keyframes[-1].positions.items()}, now)
        self._emit("info", f"playback started ({len(command.keyframes)} keyframes, x{scale:.2f}, "
                           f"{self._plan.end - now:.1f} s)")

    # ------------------------------------------------------------------ motion

    def _advance_motion(self, now: float, dt: float) -> dict[int, float]:
        if self._plan is not None:
            targets, progress, done = self._plan.sample(now)
            self._plan_progress = progress
            self._plan_targets = targets
            for servo_id, target in targets.items():
                if servo_id in self._limiters:
                    self._limiters[servo_id].set_target(target)
            if done:
                self._emit("info", f"{self._plan_label()} finished")
                self._plan = None
                self._plan_progress = None
                if self._trace is not None:
                    self._trace.plan_end = now
        with self._slider_lock:
            slider = self._slider_targets
            self._slider_targets = {}
        if self.mode is Mode.SLIDER and self._plan is None:
            for servo_id, target in slider.items():
                if servo_id in self._limiters:
                    self._limiters[servo_id].set_target(target)
        free = self._single.goals if self._single is not None else {}
        return {servo_id: limiter.step(dt) for servo_id, limiter in self._limiters.items() if servo_id not in free}

    def _send_setpoints(self, setpoints: Mapping[int, float]) -> None:
        entries = {}
        speeds = {}
        for servo_id, setpoint in setpoints.items():
            limiter = self._limiters.get(servo_id)
            speeds[servo_id] = self.motion.stream_register(self._models[servo_id], limiter.velocity if limiter else 0.0,
                                                           self.joints[servo_id].max_speed)
            entries[servo_id] = self.policy.encode_goal(servo_id, round(setpoint), speeds[servo_id])
        try:
            self.bus.sync_write(REG_GOAL_BLOCK, GOAL_BLOCK_LEN, entries)
        except UnsafeOperationError as exc:
            # Should be unreachable (limiters clamp to travel); treat as a software fault.
            self._latch_global(f"setpoint refused by motion policy: {exc}")
            self._torque_off_all()
            return
        sent = {servo_id: float(round(sp)) for servo_id, sp in setpoints.items()}
        self._sent_now.update(sent)
        self._speed_now.update(speeds)
        self._sent.update(sent)

    # ------------------------------------------------------------------ single-write poses

    def _service_single_write(self, now: float, frozen: bool) -> None:
        move = self._single
        assert move is not None
        if frozen and move.stop_reason is None:
            move.stop_reason = "GUI heartbeat late"
        if move.stop_reason is not None:
            self._stop_single_write(move)
        elif move.sent_at is None:
            self._send_single_write(move, now)
        else:
            fresh = {i: self._fresh.get(i) for i in move.goals}
            far = {i: None if r is None else r.position for i, r in fresh.items()
                   if r is None or abs(r.position - move.goals[i])
                   > self._models[i].scs_steps(SINGLE_WRITE_ARRIVAL_STEPS)}
            if not far:
                self._end_single_write(move, now)
                self._emit("info", f"single-write move '{move.label}' finished ({now - move.sent_at:.2f} s, "
                                   f"expected {move.expected_s:.2f} s)")
            elif now >= move.deadline:
                self._end_single_write(move, now)
                where = ", ".join(f"ID {i} at {p} (goal {move.goals[i]})" for i, p in far.items())
                self._emit("warn", f"single-write move '{move.label}' did not arrive within "
                                   f"{move.deadline - move.sent_at:.1f} s: {where}; normal supervision resumes")

    def _send_single_write(self, move: SingleWriteMove, now: float) -> None:
        readings = {i: self._fresh.get(i) for i in move.goals}
        # Same rule as streaming: a goal write may switch torque back on, so only to joints just read as on.
        off = sorted(i for i, r in readings.items() if r is None or r.torque != 1)
        if off:
            self._single = None
            self._finish_trace(stopped=True)
            self._emit("warn", f"single-write move '{move.label}' refused: torque not confirmed on IDs {off}")
            return
        authorized = {i: (move.goals[i], move.speeds[i]) for i in move.goals}
        entries = {i: self.policy.encode_goal(i, position, speed) for i, (position, speed) in authorized.items()}
        try:
            with self.policy.authorize_goals(authorized):
                self.bus.sync_write(REG_GOAL_BLOCK, GOAL_BLOCK_LEN, entries)
        except UnsafeOperationError as exc:
            self._single = None
            self._latch_global(f"single-write move refused by motion policy: {exc}")
            self._torque_off_all()
            return
        durations = {i: abs(move.goals[i] - r.position)  # type: ignore[union-attr]
                     / self._models[i].steps_per_s(move.speeds[i]) for i, r in readings.items()}
        move.sent_at = now
        move.expected_s = max(durations.values()) + SINGLE_WRITE_LATENCY_S
        move.deadline = now + move.expected_s * SINGLE_WRITE_TIMEOUT_FACTOR + SINGLE_WRITE_TIMEOUT_MARGIN_S
        sent = {i: float(g) for i, g in move.goals.items()}
        self._sent_now.update(sent)
        self._speed_now.update(move.speeds)
        self._sent.update(sent)
        self._emit("info", f"single-write move '{move.label}' sent: {len(move.goals)} joints in one SYNC_WRITE "
                           f"at their max_speed, expected {move.expected_s:.2f} s")

    def _end_single_write(self, move: SingleWriteMove, now: float) -> None:
        """Hand the joints back to streaming, which resumes at the goal they were sent."""
        self._single = None
        for servo_id, goal in move.goals.items():
            if (limiter := self._limiters.get(servo_id)) is not None:
                limiter.reset(goal)
        if self._trace is not None:
            self._trace.plan_end = now

    def _stop_single_write(self, move: SingleWriteMove) -> None:
        self._single = None
        self._finish_trace(stopped=True)
        if move.sent_at is None:
            self._emit("info", f"single-write move '{move.label}' cancelled ({move.stop_reason})")
            return
        hold = {}
        for servo_id in move.goals:
            reading = self._fresh.get(servo_id)
            joint = self.joints[servo_id]
            if reading is None or reading.torque != 1 or servo_id not in self.policy.armed:
                continue  # the supervisor handles a joint that lost torque or communication
            assert joint.min_position is not None and joint.max_position is not None
            hold[servo_id] = min(max(reading.position, joint.min_position), joint.max_position)
        speeds = {i: min(self.motion.arm_register(self._models[i]), move.speeds[i]) for i in hold}
        if hold:
            try:
                with self.policy.authorize_goals({i: (hold[i], speeds[i]) for i in hold}):
                    self.bus.sync_write(REG_GOAL_BLOCK, GOAL_BLOCK_LEN,
                                        {i: self.policy.encode_goal(i, hold[i], speeds[i]) for i in hold})
            except UnsafeOperationError as exc:
                self._latch_global(f"single-write hold refused by motion policy: {exc}")
                self._torque_off_all()
                return
            sent = {i: float(p) for i, p in hold.items()}
            self._sent_now.update(sent)
            self._speed_now.update(speeds)
            self._sent.update(sent)
        for servo_id in move.goals:
            if (limiter := self._limiters.get(servo_id)) is not None:
                # A joint without a fresh read keeps its sent goal, the policy's jump-limit reference.
                limiter.reset(hold.get(servo_id, move.goals[servo_id]))
        kept = sorted(set(move.goals) - set(hold))
        self._emit("info", f"single-write move '{move.label}' stopped ({move.stop_reason}): "
                           f"holding IDs {sorted(hold)} where they are"
                           + (f"; IDs {kept} not read this cycle, still heading to their goal" if kept else ""))

    # ------------------------------------------------------------------ reading / supervision

    def _store_reading(self, servo_id: int, reading: ServoReading | None, now: float) -> None:
        self._readings[servo_id] = reading if reading is not None else self._readings.get(servo_id)
        if reading is None:
            self._fail_counts[servo_id] += 1
        else:
            self._fail_counts[servo_id] = 0
            self._last_ok[servo_id] = now

    def _read_all(self, now: float) -> None:
        self._fresh = {}
        for servo_id in self.joints:
            if self._estop.is_set():
                return
            if servo_id not in self._online:
                self._fresh[servo_id] = None
                continue
            reading = read_status(self.bus, servo_id, self._models[servo_id])
            self._fresh[servo_id] = reading
            self._store_reading(servo_id, reading, self._clock())

    def _supervise(self, now: float) -> None:
        fresh = self._fresh
        single = self._single
        free = single.goals if single is not None and single.sent_at is not None else {}
        inputs = {
            servo_id: JointInput(
                reading=fresh.get(servo_id),
                consecutive_failures=self._fail_counts[servo_id],
                last_ok=self._last_ok[servo_id],
                armed=servo_id in self.policy.armed,
                sent_setpoint=self._sent.get(servo_id) if servo_id in self.policy.armed else None,
                free_running=servo_id in free,
                current_cap_ma=self._current_cap_ma(servo_id),
            )
            for servo_id in self.joints
            if servo_id in self._online
        }
        heartbeat_age = now - self._heartbeat if self.motion.heartbeat_freeze_s is not None else None
        result = self._supervisor.evaluate(now, inputs, heartbeat_age)
        if result.freeze_motion and not self._last_result.freeze_motion:
            self._emit("warn", "GUI heartbeat late: motion frozen")
        for servo_id, text in result.sustained_load.items():
            if servo_id not in self._sustained_warned:
                self._emit("warn", f"{self._name(servo_id)} {text}")
        self._sustained_warned = set(result.sustained_load)
        self._last_result = result
        self._apply_trips(result.trips)

    def _apply_trips(self, trips: list[Trip]) -> None:
        if any(t.scope == "all" for t in trips):
            reasons = [t.reason for t in trips if t.scope == "all"]
            new = [r for r in reasons if r not in self._global_faults]
            for reason in new:
                self._latch_global(reason)
            confirmed = self._torque_off_all()
            if new:
                self._emit("error", "all joints torque off: " + "; ".join(new)
                           + ("" if confirmed else " - torque off NOT confirmed, cut servo power"))
            return
        for trip in trips:
            assert trip.servo_id is not None
            if trip.servo_id in self.policy.armed:
                self._joint_faults[trip.servo_id] = trip.reason
                self._emit("error", f"protective stop: {trip.reason}")
                self._disarm(trip.servo_id, "protective stop")

    def _latch_global(self, reason: str) -> None:
        if reason not in self._global_faults:
            self._global_faults.append(reason)

    def _record(self, now: float) -> None:
        if self._recording is None:
            return
        if now - self._last_record_t < 1.0 / self.motion.teach_rate_hz - 1e-6:
            return
        fresh = self._fresh
        online = [i for i in self.joints if i in self._online]
        if not online or any(fresh.get(i) is None for i in online):
            return
        self._last_record_t = now
        positions = {i: fresh[i].position for i in online}  # type: ignore[union-attr]
        self._recording.append(Keyframe(round(now - self._record_t0, 4), positions))

    # ------------------------------------------------------------------ publishing

    def _publish(self, now: float) -> None:
        warnings = self._last_result.warnings
        single = self._single
        single_goals = single.goals if single is not None else {}
        progress = self._plan_progress
        if single is not None and single.sent_at is not None:
            progress = min(1.0, (now - single.sent_at) / max(single.expected_s, 1e-6))
        joints = {}
        for servo_id, joint in self.joints.items():
            limiter = self._limiters.get(servo_id)
            target = single_goals.get(servo_id, limiter.target if limiter else None)
            last_ok = self._last_ok[servo_id]
            joints[servo_id] = JointState(
                servo_id=servo_id,
                name=joint.name,
                min_position=joint.min_position,
                max_position=joint.max_position,
                reading=self._readings.get(servo_id),
                comm_ok=self._fail_counts[servo_id] == 0,
                consecutive_failures=self._fail_counts[servo_id],
                age_s=None if last_ok is None else now - last_ok,
                armed=servo_id in self.policy.armed,
                target=target,
                setpoint=self._sent.get(servo_id),
                fault=self._joint_faults.get(servo_id),
                warnings=tuple(warnings.get(servo_id, ())),
                model=self._models[servo_id],
                online=servo_id in self._online,
            )
        snapshot = HandSnapshot(
            time=now,
            cycle_hz=self._cycle_hz,
            mode=self.mode.value,
            joints=joints,
            estop_latched=self._estop_latched,
            estop_confirmed=self._estop_confirmed,
            faults=tuple(self._global_faults),
            motion_frozen=self._last_result.freeze_motion,
            comm_lost=self._last_result.comm_lost,
            recording_frames=None if self._recording is None else len(self._recording),
            plan_progress=progress,
            port=self.bus.port_name,
            torque_off_confirmed=self._torque_off_confirmed,
            cycle_busy_ms=self._busy_ms,
            cycle_budget_ms=1000.0 / self.motion.cycle_hz,
            notes=tuple(self._last_result.blocking),
            offline=tuple(i for i in self.joints if i not in self._online),
        )
        with self._snapshot_lock:
            self._snapshot = snapshot

    # ------------------------------------------------------------------ traces / timing

    def _start_trace(self, label: str, goals: Mapping[int, float], now: float, mode: str | None = None) -> None:
        self._finish_trace(stopped=True)
        self._plan_targets = {}
        self._trace_mode = mode or self.motion.stream_speed_mode
        names = {i: self.joints[i].name for i in goals}
        tolerances = {i: float(self._models[i].scs_steps(3)) for i in goals}
        self._trace = TraceRecorder(label, now, {i: float(v) for i, v in goals.items()}, names,
                                    tolerances=tolerances)

    def _trace_cycle(self, now: float) -> None:
        trace = self._trace
        if trace is None:
            return
        for servo_id in trace.goals:
            trace.add(now, servo_id, self._plan_targets.get(servo_id), self._sent_now.get(servo_id),
                      self._fresh.get(servo_id), self._speed_now.get(servo_id))
        if trace.plan_end is not None and now - trace.plan_end >= TRACE_TAIL_S:
            self._finish_trace()

    def _finish_trace(self, stopped: bool = False) -> None:
        trace, self._trace = self._trace, None
        if trace is None:
            return
        trace.stopped = stopped
        self._plan_targets = {}
        self._emit("info", trace.summary_line())
        if self._trace_dir is None or not trace.rows:
            return
        extra = {"started_at": datetime.now().isoformat(timespec="seconds"), "port": self.bus.port_name,
                 "cycle_hz": round(self._cycle_hz, 1), "stream_speed_mode": self._trace_mode}
        order = list(self.joints)
        # File output off the control thread: a slow disk must not stretch a control cycle.
        threading.Thread(target=self._write_trace, args=(trace, order, extra), daemon=True).start()

    def _write_trace(self, trace: TraceRecorder, order: list[int], extra: dict) -> None:
        from ..trace_plots import write_trace_html

        stem = f"trace_{datetime.now():%Y%m%d_%H%M%S}_{_ascii(trace.label)}"
        assert self._trace_dir is not None
        try:
            csv_path = write_trace_csv(trace, self._trace_dir / f"{stem}.csv")
            html_path = write_trace_html(trace_payload(trace, order, {**extra, "csv_name": csv_path.name}),
                                         self._trace_dir / f"{stem}.html")
            self._emit("info", f"trace saved: {html_path}")
        except OSError as exc:
            self._emit("warn", f"trace could not be saved: {exc}")

    def _account_busy(self, busy_s: float, period: float) -> None:
        busy_ms = busy_s * 1000.0
        self._busy_ms = busy_ms if not self._busy_ms else 0.9 * self._busy_ms + 0.1 * busy_ms
        if busy_s > BUSY_WARN_FRACTION * period:
            self._busy_streak += 1
            if self._busy_streak == BUSY_WARN_CYCLES:
                self._emit("warn", f"control cycle needs {busy_ms:.1f} ms of its {period * 1000:.1f} ms budget; "
                                   "lower MotionConfig.cycle_hz (or check bus latency)")
        else:
            self._busy_streak = 0

    # ------------------------------------------------------------------ helpers

    def _acceleration(self, servo_id: int) -> float:
        """MotionConfig.acceleration (SCS0009 steps/s^2) in the joint's own steps/s^2."""
        return self.motion.acceleration * self._models[servo_id].step_scale

    def _current_cap_ma(self, servo_id: int) -> float | None:
        cap = self.policy.current_cap(servo_id)
        per_unit = self._models[servo_id].current_ma_per_unit
        return None if cap is None or per_unit is None else cap * per_unit

    def _within(self, servo_id: int, value: float) -> bool:
        joint = self.joints.get(servo_id)
        return (joint is not None and joint.min_position is not None and joint.max_position is not None
                and joint.min_position <= value <= joint.max_position)

    def _name(self, servo_id: int) -> str:
        joint = self.joints.get(servo_id)
        return f"ID {servo_id} ({joint.name})" if joint else f"ID {servo_id}"

    def _plan_label(self) -> str:
        return f"move '{self._plan.label}'" if self._plan is not None else "move"

    def _emit(self, level: str, text: str) -> None:
        event = Event(self._clock(), level, text)
        with self._events_lock:
            self._events.append(event)
        if self._log_path is not None:
            try:
                with self._log_path.open("a", encoding="utf-8") as handle:
                    handle.write(f"{datetime.now().isoformat(timespec='milliseconds')} {level.upper():5} {text}\n")
            except OSError:
                pass


def _ascii(label: str) -> str:
    stem = "".join(c if c.isascii() and (c.isalnum() or c in "-_") else "-" for c in label).strip("-")
    return stem or "move"
