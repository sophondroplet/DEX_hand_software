from __future__ import annotations

import pytest
from fake_bus import FakeBus, FakeServo

from dexhand_servo.safe_bus import (
    ID_ASSIGNMENT,
    READ_ONLY,
    UnsafeOperationError,
    validate_instruction_packet,
)


def packet(servo_id: int, instruction: int, *params: int) -> list[int]:
    return [0xFF, 0xFF, servo_id, len(params) + 2, instruction, *params, 0]


ALWAYS_REFUSED = [
    packet(1, 0x04, 42, 1, 0),  # REG_WRITE goal position
    packet(1, 0x05),  # ACTION
    packet(1, 0x06),  # parameter restore
    packet(1, 0x08),  # reboot
    packet(1, 0x0A),  # reset
    packet(1, 0x0B, 0, 0),  # position calibration
    packet(0xFE, 0x83, 42, 6, 1, 0, 0, 0, 0, 0, 0),  # SYNC_WRITE
    packet(0xFE, 0x82, 56, 2, 1),  # SYNC_READ (broadcast)
    packet(0xFE, 0x01),  # broadcast PING
]


@pytest.mark.parametrize("pkt", ALWAYS_REFUSED)
@pytest.mark.parametrize("policy", [READ_ONLY, ID_ASSIGNMENT])
def test_dangerous_instructions_refused_in_every_mode(pkt: list[int], policy) -> None:
    with pytest.raises(UnsafeOperationError):
        validate_instruction_packet(pkt, policy)


@pytest.mark.parametrize("address", [5, 40, 42, 48])
def test_read_only_refuses_any_write(address: int) -> None:
    with pytest.raises(UnsafeOperationError):
        validate_instruction_packet(packet(1, 0x03, address, 0), READ_ONLY)


@pytest.mark.parametrize(
    "pkt",
    [
        packet(1, 0x03, 40, 1),  # torque enable
        packet(1, 0x03, 42, 1, 0),  # goal position
        packet(1, 0x03, 5, 2, 0),  # multi-byte write starting at ID
        packet(1, 0x03, 48, 2),  # invalid lock value
        packet(1, 0x03, 5, 254),  # broadcast as new ID
    ],
)
def test_id_assignment_mode_only_allows_id_and_lock(pkt: list[int]) -> None:
    with pytest.raises(UnsafeOperationError):
        validate_instruction_packet(pkt, ID_ASSIGNMENT)


def test_allowed_packets() -> None:
    validate_instruction_packet(packet(1, 0x01), READ_ONLY)
    validate_instruction_packet(packet(1, 0x02, 56, 8), READ_ONLY)
    validate_instruction_packet(packet(1, 0x03, 5, 3), ID_ASSIGNMENT)
    validate_instruction_packet(packet(3, 0x03, 48, 1), ID_ASSIGNMENT)


def test_sdk_motion_helpers_never_reach_the_wire(open_bus) -> None:
    fake = FakeBus([FakeServo(1)])
    bus = open_bus(fake)
    handler = bus._handler  # reach past SafeBus on purpose: prove the SDK paths are covered too

    attempts = [
        lambda: handler.WritePos(1, 900, 0, 1000),
        lambda: handler.RegWritePos(1, 900, 0, 1000),
        lambda: handler.RegAction(),
        lambda: handler.action(1),
        lambda: handler.unLockEprom(1),
        lambda: handler.write1ByteTxRx(1, 40, 1),
        lambda: handler.write2ByteTxOnly(1, 42, 900),
        lambda: handler.PWMMode(1),
        lambda: handler.syncReadTx(56, 2, [1], 1),
    ]
    for attempt in attempts:
        with pytest.raises(UnsafeOperationError):
            attempt()

    handler.SyncWritePos(1, 900, 0, 1000)
    with pytest.raises(UnsafeOperationError):
        handler.groupSyncWrite.txPacket()

    assert fake.received == []
    assert bus.ping(1).ok, "bus must stay usable after refused operations"


def test_port_layer_rechecks_final_bytes(open_bus) -> None:
    fake = FakeBus([FakeServo(1)])
    bus = open_bus(fake)
    frame = packet(1, 0x03, 42, 1, 0)
    with pytest.raises(UnsafeOperationError):
        bus._port.writePort(frame)
    assert fake.received == []
    assert bus.ping(1).ok


def test_write_byte_refused_outside_id_mode_and_for_other_registers(open_bus) -> None:
    fake = FakeBus([FakeServo(1)])
    bus = open_bus(fake)
    with pytest.raises(UnsafeOperationError):
        bus.write_byte(1, 5, 3)
    with bus.id_assignment_mode(), pytest.raises(UnsafeOperationError):
        bus.write_byte(1, 40, 1)
    assert bus.policy is READ_ONLY
    assert fake.writes() == []


class _AllowAll(READ_ONLY.__class__):
    """Lets every WRITE / SYNC_WRITE through the policy layer, so only the census check decides."""

    def __init__(self) -> None:
        super().__init__("allow-all", frozenset(range(256)))

    def check_write(self, packet) -> None:
        pass

    def check_sync_write(self, packet) -> None:
        pass


def _bus_with(open_bus, census):
    from fake_bus import FakeBus, FakeServo

    from dexhand_servo.models import HLS3606

    fake = FakeBus([FakeServo(1), FakeServo(7, position=2000, model=HLS3606, secondary_id=5)])
    bus = open_bus(fake)
    bus.census = census
    bus.set_policy(_AllowAll())
    return fake, bus


def test_no_census_refuses_every_write_except_torque_off(open_bus) -> None:
    fake, bus = _bus_with(open_bus, None)
    for attempt in (lambda: bus.write_byte(1, 5, 3), lambda: bus.write_byte(1, 40, 1),
                    lambda: bus.sync_write(42, 6, {1: bytes(6)})):
        with pytest.raises(UnsafeOperationError, match="no bus census"):
            attempt()
    assert bus.write_byte(1, 40, 0).ok
    bus.sync_write(40, 1, {1: b"\x00", 7: b"\x00"})
    assert fake.writes() == [(1, 40, 0)] and len(fake.sync_writes()) == 1


def test_writes_to_a_secondary_id_are_refused_but_torque_off_passes(open_bus) -> None:
    fake, bus = _bus_with(open_bus, {5: frozenset({7})})
    with pytest.raises(UnsafeOperationError, match="secondary ID of HLS servo"):
        bus.write_byte(5, 41, 0)
    with pytest.raises(UnsafeOperationError, match="secondary ID of HLS servo"):
        bus.sync_write(42, 6, {1: bytes(6), 5: bytes(6)})
    bus.write_byte(7, 41, 0)  # the HLS itself, by its primary ID
    bus.write_byte(5, 40, 0)  # torque-off is always allowed (E-stop)
    bus.sync_write(40, 1, {5: b"\x00", 1: b"\x00"})
    assert [w[:2] for w in fake.writes()] == [(7, 41), (5, 40)]
    assert fake.servo(7).torque == 0


def test_census_finds_secondary_ids_and_fails_closed(open_bus) -> None:
    from fake_bus import FakeBus, FakeServo

    from dexhand_servo.diagnostics import CensusError, take_census
    from dexhand_servo.models import HLS3606

    hls = [FakeServo(3, position=2000, model=HLS3606, secondary_id=0),
           FakeServo(4, position=2000, model=HLS3606, secondary_id=0),
           FakeServo(9, position=2000, model=HLS3606, secondary_id=9)]
    bus = open_bus(FakeBus([FakeServo(1), *hls]))
    assert take_census(bus, ids=range(12)) == {0: frozenset({3, 4}), 9: frozenset({9})}
    assert bus.history == [], "census traffic is not kept"

    odd = FakeServo(2)
    odd.regs[2] = 7  # unsupported model
    bus = open_bus(FakeBus([odd]))
    bus.census = None
    with pytest.raises(CensusError, match="unsupported"):
        take_census(bus, ids=range(4))
    assert bus.census is None

    twins = [FakeServo(1, position=2000, model=HLS3606, secondary_id=0),
             FakeServo(1, position=2000, model=HLS3606, secondary_id=8)]
    bus = open_bus(FakeBus(twins))
    bus.census = None
    with pytest.raises(CensusError, match="several servos"):
        take_census(bus, ids=range(3))
    assert bus.census is None
