"""The controller with an incomplete servo set: joints that do not reply at startup are offline."""

from __future__ import annotations

from fake_bus import FakeBus, FakeServo

from dexhand_servo.config import JointConfig, MotionConfig
from dexhand_servo.control.worker import (
    ArmJoints,
    Controller,
    DisarmJoints,
    Mode,
    RescanBus,
    SetMode,
    StartRecording,
    StopRecording,
)
from dexhand_servo.diagnostics import take_census
from dexhand_servo.models import HLS3606
from dexhand_servo.safe_bus import SafeBus

JOINTS = {1: JointConfig("a", 300, 700, max_speed=150), 2: JointConfig("b", 300, 700, max_speed=150)}
MOTION = MotionConfig(heartbeat_freeze_s=None, heartbeat_trip_s=None)
DT = 0.02


def controller(clock, servos, census_ids=range(4)):
    fake = FakeBus(servos, clock=clock)
    bus = SafeBus("/dev/fake", 1_000_000, latency_ms=2, serial_factory=fake.serial_factory)
    bus.open()
    take_census(bus, ids=census_ids)
    ctrl = Controller(bus, JOINTS, MOTION, clock=clock)
    confirmed = ctrl.startup()
    return fake, bus, ctrl, confirmed


def cycles(ctrl, clock, n):
    for _ in range(n):
        clock.advance(DT)
        ctrl.run_cycle()
    return ctrl.snapshot()


def messages(ctrl) -> str:
    return " | ".join(e.text for e in ctrl.drain_events())


def addressed_to(fake, servo_id: int, since: int = 0) -> list[bytes]:
    return [p for p in fake.received[since:] if p[2] == servo_id]


def test_missing_joint_is_offline_and_the_rest_stays_usable(clock) -> None:
    fake, _, ctrl, confirmed = controller(clock, [FakeServo(1, position=500, position_noise=0)])
    assert confirmed, "torque off is confirmed on the joints that are there"
    assert "offline (no reply): [2]" in messages(ctrl)

    start = len(fake.received)
    ctrl.submit(ArmJoints((1, 2)))  # "Arm all"
    snap = cycles(ctrl, clock, 5)
    assert snap.joints[1].armed and not snap.joints[2].armed
    assert snap.offline == (2,) and not snap.joints[2].online and snap.joints[1].online
    assert not snap.comm_lost and not snap.notes, "an offline joint blocks nothing"
    assert "offline IDs [2] skipped" in messages(ctrl)

    ctrl.set_target(1, 600)
    cycles(ctrl, clock, 60)
    assert abs(fake.servo(1).position - 600) <= 2
    assert addressed_to(fake, 2, start) == [], "an offline joint is never polled"

    ctrl.submit(ArmJoints((2,)))
    cycles(ctrl, clock, 1)
    assert "offline (no reply at startup)" in messages(ctrl)

    ctrl.estop("test")
    snap = cycles(ctrl, clock, 1)
    assert snap.estop_latched and snap.estop_confirmed and fake.servo(1).torque == 0


def test_torque_off_all_skips_offline_joints_without_errors(clock) -> None:
    _, _, ctrl, _ = controller(clock, [FakeServo(1, position=500, position_noise=0)])
    messages(ctrl)
    ctrl.submit(DisarmJoints((1, 2)))  # "Torque off all"
    cycles(ctrl, clock, 1)
    assert "NOT confirmed" not in messages(ctrl)


def test_joint_dropping_out_mid_session_still_trips_everything(clock) -> None:
    servo2 = FakeServo(2, position=450, position_noise=0)
    fake, _, ctrl, _ = controller(clock, [FakeServo(1, position=500, position_noise=0), servo2])
    ctrl.submit(ArmJoints((1,)))
    snap = cycles(ctrl, clock, 3)
    assert snap.joints[1].armed and snap.offline == ()
    servo2.drop_replies = 10_000
    snap = cycles(ctrl, clock, 5)
    assert not snap.joints[1].armed and fake.servo(1).torque == 0
    assert snap.comm_lost and any("communication lost with ID 2" in f for f in snap.faults)


def test_rescan_adds_a_servo_connected_later(clock) -> None:
    fake, bus, ctrl, _ = controller(clock, [FakeServo(1, position=500, position_noise=0)])
    ctrl.submit(ArmJoints((1,)))
    cycles(ctrl, clock, 2)
    ctrl.submit(RescanBus())
    cycles(ctrl, clock, 1)
    assert "rescan refused: disarm all joints first" in messages(ctrl)

    ctrl.submit(DisarmJoints((1,)))
    late = FakeServo(2, position=450, position_noise=0, torque_enable=1)
    fake.servos.append(late)
    ctrl.submit(RescanBus())
    snap = cycles(ctrl, clock, 2)
    log = messages(ctrl)
    assert "newly connected [2]" in log and "rescan: bus census done" in log
    assert snap.offline == () and late.torque == 0, "a joining servo is switched to torque off"
    assert bus.census == {}
    ctrl.submit(ArmJoints((2,)))
    snap = cycles(ctrl, clock, 2)
    assert snap.joints[2].armed


def test_rescan_redoes_the_census(clock) -> None:
    fake, bus, ctrl, _ = controller(clock, [FakeServo(1, position=500, position_noise=0)])
    fake.servos.append(FakeServo(9, position=2000, model=HLS3606, secondary_id=1))
    ctrl.submit(RescanBus())
    cycles(ctrl, clock, 1)
    assert bus.census == {1: frozenset({9})}
    assert "joint ID 1 is also the secondary ID of HLS servo(s) [9]" in messages(ctrl)
    ctrl.submit(ArmJoints((1,)))
    snap = cycles(ctrl, clock, 2)
    assert not snap.joints[1].armed

    odd = FakeServo(7)
    odd.regs[2] = 7  # unsupported model: census fails closed
    fake.servos[-1] = odd
    ctrl.submit(RescanBus())
    cycles(ctrl, clock, 1)
    assert bus.census is None and "bus census failed" in messages(ctrl)


def test_teach_records_only_online_joints(clock) -> None:
    _, _, ctrl, _ = controller(clock, [FakeServo(1, position=500, position_noise=0)])
    ctrl.submit(SetMode(Mode.TEACH))
    ctrl.submit(StartRecording())
    cycles(ctrl, clock, 10)
    ctrl.submit(StopRecording())
    cycles(ctrl, clock, 1)
    recording = ctrl.take_recording()
    assert recording and all(set(kf.positions) == {1} for kf in recording)


def test_no_responding_joint_is_not_reported_as_confirmed(clock) -> None:
    _, _, ctrl, confirmed = controller(clock, [])
    assert not confirmed and "no joint replied" in messages(ctrl)
