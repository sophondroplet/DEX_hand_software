from __future__ import annotations

import time
from dataclasses import replace

import pytest
from fake_bus import FakeBus, FakeServo

from dexhand_servo.config import JointConfig, MotionConfig
from dexhand_servo.control.worker import (
    ArmJoints,
    Controller,
    DisarmJoints,
    Jog,
    Mode,
    MoveTo,
    Playback,
    ResetFaults,
    SetMode,
    StartRecording,
    StopMotion,
    StopRecording,
)
from dexhand_servo.control.policy import goal_block
from dexhand_servo.safe_bus import SafeBus

JOINTS = {1: JointConfig("a", 300, 700, max_speed=150), 2: JointConfig("b", 300, 700, max_speed=150)}
MOTION = MotionConfig(heartbeat_freeze_s=None, heartbeat_trip_s=None)
DT = 0.02


@pytest.fixture
def rig(clock):
    fake = FakeBus([FakeServo(1, position=500, position_noise=0), FakeServo(2, position=450, position_noise=0)],
                   clock=clock)
    bus = SafeBus("/dev/fake", 1_000_000, latency_ms=2, serial_factory=fake.serial_factory)
    bus.census = {}  # census taken: no HLS secondary IDs
    bus.open()
    ctrl = Controller(bus, JOINTS, MOTION, clock=clock)
    ctrl.startup()
    yield fake, ctrl, clock
    bus.close()


def cycles(ctrl, clock, n):
    for _ in range(n):
        clock.advance(DT)
        ctrl.run_cycle()
    return ctrl.snapshot()


def goal_writes(fake):
    return [w for w in fake.writes() if w[1] == 42] + [s for s in fake.sync_writes() if s[0] == 42]


def messages(ctrl):
    return " | ".join(e.text for e in ctrl.drain_events())


def test_startup_turns_torque_off(clock) -> None:
    fake = FakeBus([FakeServo(1, position=500, torque_enable=1)], clock=clock)
    bus = SafeBus("/dev/fake", 1_000_000, latency_ms=2, serial_factory=fake.serial_factory)
    bus.census = {}  # census taken: no HLS secondary IDs
    bus.open()
    ctrl = Controller(bus, {1: JOINTS[1]}, MOTION, clock=clock)
    assert ctrl.startup()
    assert fake.servo(1).torque == 0


def test_arm_refused_outside_travel_without_any_goal_write(clock) -> None:
    fake = FakeBus([FakeServo(1, position=250, position_noise=0)], clock=clock)
    bus = SafeBus("/dev/fake", 1_000_000, latency_ms=2, serial_factory=fake.serial_factory)
    bus.census = {}  # census taken: no HLS secondary IDs
    bus.open()
    ctrl = Controller(bus, {1: JOINTS[1]}, MOTION, clock=clock)
    ctrl.startup()
    ctrl.submit(ArmJoints((1,)))
    snap = cycles(ctrl, clock, 3)
    assert not snap.joints[1].armed and fake.servo(1).torque == 0
    assert goal_writes(fake) == []
    assert "outside travel" in messages(ctrl)


def test_arm_holds_then_slider_moves_within_limits(rig) -> None:
    fake, ctrl, clock = rig
    ctrl.submit(ArmJoints((1,)))
    snap = cycles(ctrl, clock, 5)
    assert snap.joints[1].armed and fake.servo(1).torque == 1
    assert abs(fake.servo(1).position - 500) <= 1, "arming must not move the joint"

    ctrl.set_target(1, 900)  # beyond travel: clamped to 700
    previous = 500.0
    for _ in range(200):
        snap = cycles(ctrl, clock, 1)
        setpoint = snap.joints[1].setpoint
        assert abs(setpoint - previous) <= 150 * DT + 1.0, "setpoint exceeds the joint speed limit"
        previous = setpoint
    assert snap.joints[1].setpoint == 700
    assert abs(fake.servo(1).position - 700) <= 2
    assert not snap.faults and snap.joints[1].fault is None


def test_estop_beats_a_command_flood(rig) -> None:
    fake, ctrl, clock = rig
    ctrl.submit(ArmJoints((1, 2)))
    cycles(ctrl, clock, 3)
    ctrl.submit(SetMode(Mode.JOG))
    for _ in range(1000):
        ctrl.submit(Jog(1, 5))
    ctrl.estop("test")
    snap = cycles(ctrl, clock, 1)
    assert fake.servo(1).torque == 0 and fake.servo(2).torque == 0
    assert snap.estop_latched and snap.estop_confirmed
    assert ctrl._commands.empty(), "queued commands must be discarded"

    ctrl.submit(ArmJoints((1,)))
    snap = cycles(ctrl, clock, 2)
    assert not snap.joints[1].armed, "arming stays refused while the E-stop is latched"
    ctrl.submit(ResetFaults())
    ctrl.submit(ArmJoints((1,)))
    snap = cycles(ctrl, clock, 2)
    assert snap.joints[1].armed and not snap.estop_latched


def test_interference_trips_only_that_joint(rig) -> None:
    fake, ctrl, clock = rig
    ctrl.submit(ArmJoints((1, 2)))
    cycles(ctrl, clock, 3)
    fake.servo(1).blocked = True
    ctrl.set_target(1, 650)
    snap = cycles(ctrl, clock, 50)
    assert fake.servo(1).torque == 0 and not snap.joints[1].armed
    assert snap.joints[1].fault and "load" in snap.joints[1].fault
    assert snap.joints[2].armed and fake.servo(2).torque == 1


def test_communication_loss_drops_all_torque(rig) -> None:
    fake, ctrl, clock = rig
    ctrl.submit(ArmJoints((1, 2)))
    cycles(ctrl, clock, 3)
    fake.servo(2).drop_replies = 3
    snap = cycles(ctrl, clock, 4)
    assert any("communication lost" in f for f in snap.faults)
    assert fake.servo(1).torque == 0


def test_usb_unplug_is_survivable(rig) -> None:
    fake, ctrl, clock = rig
    ctrl.submit(ArmJoints((1,)))
    cycles(ctrl, clock, 3)
    fake.raise_on_write = True
    snap = cycles(ctrl, clock, 5)  # must not raise
    assert snap.comm_lost and snap.faults
    assert not ctrl.bus._port.is_using, "busy flag must be released after a failed exchange"
    fake.raise_on_write = False
    snap = cycles(ctrl, clock, 2)
    assert snap.joints[1].comm_ok


def test_servo_dropping_torque_by_itself_trips_all(rig) -> None:
    fake, ctrl, clock = rig
    ctrl.submit(ArmJoints((1, 2)))
    cycles(ctrl, clock, 3)
    fake.servo(1).regs[40] = 0  # e.g. the servo's own overload cut-out
    snap = cycles(ctrl, clock, 2)
    assert any("dropped torque" in f for f in snap.faults)
    assert fake.servo(2).torque == 0


def test_supply_sag_trips_all(rig) -> None:
    fake, ctrl, clock = rig
    ctrl.submit(ArmJoints((1, 2)))
    cycles(ctrl, clock, 3)
    for servo in fake.servos:
        servo.voltage_dv = 52
    snap = cycles(ctrl, clock, 15)
    assert any("supply" in f for f in snap.faults)
    assert all(s.torque == 0 for s in fake.servos)
    ctrl.submit(ResetFaults())
    ctrl.submit(ArmJoints((1,)))
    snap = cycles(ctrl, clock, 2)
    assert not snap.joints[1].armed, "arming blocked while the supply is still out of range"


def test_synchronised_pose_move(rig) -> None:
    fake, ctrl, clock = rig
    ctrl.submit(ArmJoints((1, 2)))
    ctrl.submit(SetMode(Mode.POSE))
    cycles(ctrl, clock, 3)
    ctrl.submit(MoveTo({1: 650, 2: 350}, "test pose"))
    arrivals = {}
    for i in range(200):
        snap = cycles(ctrl, clock, 1)
        for servo_id, goal in ((1, 650), (2, 350)):
            if servo_id not in arrivals and abs(snap.joints[servo_id].setpoint - goal) <= 1:
                arrivals[servo_id] = i
    assert set(arrivals) == {1, 2}
    assert abs(arrivals[1] - arrivals[2]) <= 3, f"joints should arrive together: {arrivals}"
    assert not snap.faults


def test_move_refused_outside_travel(rig) -> None:
    fake, ctrl, clock = rig
    ctrl.submit(ArmJoints((1,)))
    ctrl.submit(SetMode(Mode.SYNC))
    ctrl.submit(MoveTo({1: 800}, "bad"))
    cycles(ctrl, clock, 3)
    assert "outside travel" in messages(ctrl)
    assert ctrl._plan is None


def test_teach_record_and_playback(rig) -> None:
    fake, ctrl, clock = rig
    ctrl.submit(SetMode(Mode.TEACH))
    ctrl.submit(StartRecording())
    cycles(ctrl, clock, 2)
    for step in range(50):  # operator moves joint 1 by hand
        fake.servo(1).position = 500 + step * 2
        cycles(ctrl, clock, 1)
    ctrl.submit(StopRecording())
    cycles(ctrl, clock, 1)
    keyframes = ctrl.take_recording()
    assert keyframes and keyframes[-1].positions[1] == 598

    ctrl.submit(Playback(tuple(keyframes), 1.0))
    cycles(ctrl, clock, 1)
    assert "arm IDs" in messages(ctrl), "playback needs the joints armed"

    ctrl.submit(ArmJoints((1, 2)))
    cycles(ctrl, clock, 2)
    fake.servo(1).position = 520  # hand rests somewhere else before playback
    cycles(ctrl, clock, 2)
    ctrl.submit(Playback(tuple(keyframes), 1.0))
    snap = cycles(ctrl, clock, 250)
    assert abs(fake.servo(1).position - 598) <= 2
    assert not snap.faults


def test_recording_refused_while_armed(rig) -> None:
    fake, ctrl, clock = rig
    ctrl.submit(ArmJoints((1,)))
    ctrl.submit(SetMode(Mode.TEACH))
    ctrl.submit(StartRecording())
    snap = cycles(ctrl, clock, 2)
    assert snap.recording_frames is None
    ctrl.submit(DisarmJoints((1,)))
    ctrl.submit(StartRecording())
    snap = cycles(ctrl, clock, 2)
    assert snap.recording_frames is not None


def test_gui_heartbeat_watchdog(clock) -> None:
    fake = FakeBus([FakeServo(1, position=500, position_noise=0)], clock=clock)
    bus = SafeBus("/dev/fake", 1_000_000, latency_ms=2, serial_factory=fake.serial_factory)
    bus.census = {}  # census taken: no HLS secondary IDs
    bus.open()
    ctrl = Controller(bus, {1: JOINTS[1]}, replace(MOTION, heartbeat_freeze_s=0.5, heartbeat_trip_s=2.0), clock=clock)
    ctrl.startup()
    ctrl.heartbeat()
    ctrl.submit(ArmJoints((1,)))
    snap = cycles(ctrl, clock, 30)
    assert snap.motion_frozen and snap.joints[1].armed
    snap = cycles(ctrl, clock, 80)
    assert any("unresponsive" in f for f in snap.faults) and fake.servo(1).torque == 0


def test_worker_thread_lifecycle() -> None:
    fake = FakeBus([FakeServo(1, position=500)])
    bus = SafeBus("/dev/fake", 1_000_000, latency_ms=2, serial_factory=fake.serial_factory)
    bus.census = {}  # census taken: no HLS secondary IDs
    bus.open()
    ctrl = Controller(bus, {1: JOINTS[1]}, MOTION)
    ctrl.start()
    ctrl.submit(ArmJoints((1,)))
    deadline = time.perf_counter() + 2.0
    while time.perf_counter() < deadline:
        snap = ctrl.snapshot()
        if snap and snap.joints[1].armed:
            break
        time.sleep(0.02)
    assert snap and snap.joints[1].armed and snap.cycle_hz > 20
    ctrl.estop("test")
    time.sleep(0.1)
    assert fake.servo(1).torque == 0
    ctrl.stop()
    assert not ctrl.running and not bus._port.is_open


def test_set_mode_accepts_plain_strings(rig) -> None:
    fake, ctrl, clock = rig
    ctrl.submit(SetMode("pose"))  # what a Qt item-data round trip delivers
    ctrl.submit(SetMode("nonsense"))
    snap = cycles(ctrl, clock, 2)
    assert ctrl.mode is Mode.POSE and snap.mode == "pose"
    assert not snap.faults
    assert "unknown mode" in messages(ctrl)


def test_pose_move_streams_at_full_register_and_writes_a_trace(clock, tmp_path) -> None:
    import json
    import re

    fake = FakeBus([FakeServo(1, position=500, position_noise=0), FakeServo(2, position=450, position_noise=0)],
                   clock=clock)
    bus = SafeBus("/dev/fake", 1_000_000, latency_ms=2, serial_factory=fake.serial_factory)
    bus.census = {}  # census taken: no HLS secondary IDs
    bus.open()
    ctrl = Controller(bus, JOINTS, replace(MOTION, stream_speed_mode="fixed"), clock=clock, trace_dir=tmp_path)
    ctrl.startup()
    ctrl.submit(ArmJoints((1, 2)))
    ctrl.submit(SetMode(Mode.POSE))
    cycles(ctrl, clock, 3)
    ctrl.submit(MoveTo({1: 600, 2: 380}, "grip test"))
    cycles(ctrl, clock, 150)
    speeds = {int.from_bytes(entry[4:6], "big")
              for address, _, entries in fake.sync_writes() if address == 42 for entry in entries.values()}
    assert speeds == {MOTION.stream_speed_register}
    events = messages(ctrl)
    assert "trace 'grip test'" in events and "arrival spread" in events
    deadline = time.perf_counter() + 2
    while time.perf_counter() < deadline and not list(tmp_path.glob("trace_*.html")):
        time.sleep(0.02)
    html = next(tmp_path.glob("trace_*_grip-test.html")).read_text(encoding="utf-8")
    data = json.loads(re.search(r'id="trace-data">(.*?)</script>', html, re.S).group(1))
    arrivals = {j["id"]: j["summary"]["arrival_s"] for j in data["joints"]}
    assert all(a is not None for a in arrivals.values())
    assert abs(arrivals[1] - arrivals[2]) < 0.1, arrivals
    assert len(list(tmp_path.glob("trace_*.csv"))) == 1


def test_estop_mid_move_closes_the_trace_as_stopped(rig) -> None:
    fake, ctrl, clock = rig
    ctrl.submit(ArmJoints((1,)))
    ctrl.submit(SetMode(Mode.SYNC))
    cycles(ctrl, clock, 2)
    ctrl.submit(MoveTo({1: 650}, "long"))
    cycles(ctrl, clock, 10)
    ctrl.estop("test")
    cycles(ctrl, clock, 1)
    assert "trace 'long' (stopped)" in messages(ctrl)


def test_dynamic_speed_register_follows_setpoint_velocity(rig) -> None:
    fake, ctrl, clock = rig
    assert ctrl.motion.stream_speed_mode == "dynamic"
    ctrl.submit(ArmJoints((1,)))
    ctrl.submit(SetMode(Mode.POSE))
    cycles(ctrl, clock, 3)
    start = len(fake.received)
    ctrl.submit(MoveTo({1: 650}, "speed test"))
    cycles(ctrl, clock, 160)  # 1.9 s move + 0.6 s trace tail
    goal_packets = [p for p in fake.received[start:] if p[4] == 0x83 and p[5] == 42]
    goals = [int.from_bytes(p[8:10], "big") for p in goal_packets]
    speeds = [int.from_bytes(p[12:14], "big") for p in goal_packets]
    m = ctrl.motion
    assert min(speeds) == m.stream_speed_floor, "settling / start uses the floor"
    peak_velocity = max(abs(b - a) for a, b in zip(goals, goals[1:])) / DT
    assert max(speeds) <= m.stream_speed_gain * (peak_velocity + 1 / DT) + 1
    assert max(speeds) > m.stream_speed_floor and max(speeds) < m.stream_speed_register
    assert abs(fake.servo(1).position - 650) <= 2
    events = messages(ctrl)
    assert "max goal step" in events


def test_unknown_stream_speed_mode_is_rejected(clock) -> None:
    fake = FakeBus([FakeServo(1)], clock=clock)
    bus = SafeBus("/dev/fake", 1_000_000, latency_ms=2, serial_factory=fake.serial_factory)
    bus.census = {}  # census taken: no HLS secondary IDs
    with pytest.raises(ValueError, match="stream_speed_mode"):
        Controller(bus, {1: JOINTS[1]}, replace(MOTION, stream_speed_mode="turbo"), clock=clock)


def armed_pose_rig(rig):
    fake, ctrl, clock = rig
    ctrl.submit(ArmJoints((1, 2)))
    ctrl.submit(SetMode(Mode.POSE))
    cycles(ctrl, clock, 3)
    messages(ctrl)
    return fake, ctrl, clock


def goal_sync_writes(fake, start=0):
    return [entries for address, _, entries in fake.sync_writes()[start:] if address == 42]


def test_single_write_pose_is_one_packet_paced_by_the_servos(rig) -> None:
    fake, ctrl, clock = armed_pose_rig(rig)
    before = len(fake.sync_writes())
    ctrl.submit(MoveTo({1: 650, 2: 350}, "grip", single_write=True))
    snap = cycles(ctrl, clock, 1)
    assert goal_sync_writes(fake, before) == [{1: goal_block(650, 150), 2: goal_block(350, 150)}], \
        "one packet, final goals, each joint at its max_speed"
    assert snap.joints[1].target == 650 and snap.plan_progress is not None

    streamed, n = 0, 0
    while ctrl._single is not None and n < 200:
        start = len(fake.sync_writes())
        cycles(ctrl, clock, 1)
        streamed += len(goal_sync_writes(fake, start))
        n += 1
    assert streamed == 0, "nothing may overwrite the goal while the servos pace the move"
    assert 1.0 / DT - 5 <= n <= (1.0 + 0.1) / DT + 5, f"150 steps at 150 steps/s took {n} cycles"
    assert abs(fake.servo(1).position - 650) < 4 and abs(fake.servo(2).position - 350) < 4
    events = messages(ctrl)
    assert "single-write move 'grip' finished" in events

    start = len(fake.sync_writes())
    snap = cycles(ctrl, clock, 40)
    assert {e[1][:2] for e in goal_sync_writes(fake, start)} == {goal_block(650, 1)[:2]}, \
        "streaming resumes at the goal"
    assert abs(fake.servo(1).position - 650) <= 1 and abs(fake.servo(2).position - 350) <= 1
    assert not snap.faults and snap.joints[1].fault is None and snap.joints[2].fault is None
    assert "trace 'grip'" in messages(ctrl)


def test_stop_mid_single_write_holds_where_the_joint_is(rig) -> None:
    fake, ctrl, clock = armed_pose_rig(rig)
    ctrl.submit(MoveTo({1: 650}, "long", single_write=True))
    cycles(ctrl, clock, 20)
    ctrl.submit(StopMotion())
    cycles(ctrl, clock, 1)
    hold = fake.servo(1).goal
    assert 530 < hold < 620, hold
    snap = cycles(ctrl, clock, 30)
    assert abs(fake.servo(1).position - hold) <= 2 and fake.servo(1).goal == hold
    events = messages(ctrl)
    assert "single-write move 'long' stopped (operator)" in events and "holding IDs [1]" in events
    assert snap.joints[1].armed and not snap.faults and snap.joints[1].fault is None


def test_single_write_timeout_warns_then_normal_supervision_trips(rig) -> None:
    fake, ctrl, clock = armed_pose_rig(rig)
    fake.servo(1).blocked_load_pct = 10.0  # friction the P-only servo loop cannot overcome
    fake.servo(1).blocked = True
    ctrl.submit(MoveTo({1: 650}, "stuck", single_write=True))
    snap = cycles(ctrl, clock, 150)  # deadline = (1.0 + 0.1) s * 2 + 1 s = 3.2 s
    assert snap.joints[1].armed and snap.joints[1].fault is None, "no tracking trip while free running"
    snap = cycles(ctrl, clock, 40)
    assert "did not arrive" in messages(ctrl)
    assert snap.joints[1].fault and "tracking" in snap.joints[1].fault
    assert fake.servo(1).torque == 0 and fake.servo(2).torque == 1


def test_disarming_one_joint_mid_single_write_holds_the_others(rig) -> None:
    fake, ctrl, clock = armed_pose_rig(rig)
    ctrl.submit(MoveTo({1: 650, 2: 350}, "pair", single_write=True))
    cycles(ctrl, clock, 15)
    ctrl.submit(DisarmJoints((2,)))
    cycles(ctrl, clock, 1)
    hold = fake.servo(1).goal
    assert hold < 640
    snap = cycles(ctrl, clock, 20)
    assert abs(fake.servo(1).position - hold) <= 2 and fake.servo(2).torque == 0
    assert "stopped (ID 2 (b) disarmed)" in messages(ctrl)
    assert not snap.faults


def test_moves_refused_while_a_single_write_is_in_flight_and_estop_ends_it(rig) -> None:
    fake, ctrl, clock = armed_pose_rig(rig)
    ctrl.submit(MoveTo({1: 650}, "a", single_write=True))
    cycles(ctrl, clock, 5)
    ctrl.submit(MoveTo({1: 400}, "b"))
    ctrl.submit(MoveTo({1: 400}, "c", single_write=True))
    cycles(ctrl, clock, 2)
    assert messages(ctrl).count("in progress") == 2
    assert fake.servo(1).goal == 650
    ctrl.estop("test")
    snap = cycles(ctrl, clock, 1)
    assert fake.servo(1).torque == 0 and ctrl._single is None and snap.estop_confirmed
    assert "trace 'a' (stopped)" in messages(ctrl)


def test_mode_change_mid_single_write_holds_then_slider_takes_over(rig) -> None:
    fake, ctrl, clock = armed_pose_rig(rig)
    ctrl.submit(MoveTo({1: 650}, "a", single_write=True))
    cycles(ctrl, clock, 10)
    ctrl.submit(SetMode(Mode.SLIDER))
    cycles(ctrl, clock, 1)
    hold = fake.servo(1).goal
    assert hold < 640
    ctrl.set_target(1, hold - 40)
    snap = cycles(ctrl, clock, 60)
    assert abs(fake.servo(1).position - (hold - 40)) <= 2
    assert not snap.faults and snap.joints[1].fault is None
