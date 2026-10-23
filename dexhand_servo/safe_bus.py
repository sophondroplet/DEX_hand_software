"""Guarded access to the SCS servo bus on top of the Feetech SDK.

The hand has no mechanical end stops, so nothing that can move a servo or alter
its configuration may ever reach the wire by accident. Every outgoing packet is
validated twice against the active `WritePolicy`:

1. `GuardedPacketHandler.txPacket` - the single choke point every SDK send path
   (write, reg_write, action, sync_write, ...) goes through, checked before the
   SDK touches the port.
2. `TrafficLoggingPortHandler.writePort` - the final framed bytes, just before
   `serial.write`, so even a direct port write is covered.

The default policy allows only PING and READ. `SafeBus.id_assignment_mode()`
temporarily allows single-byte WRITEs to the ID and EEPROM-lock registers and
nothing else. Motion is only possible under `control.policy.MotionPolicy`,
which validates every goal against the joint's travel limits. Broadcast is
refused except as the address of a policy-validated SYNC_WRITE.

HLS servos also act on WRITE / SYNC_WRITE packets addressed to their secondary ID
(register 7), which PING and READ cannot reveal. `SafeBus.census` maps each secondary
ID found on the bus to the HLS servos answering to it (`diagnostics.take_census`).
Until a census is taken, every WRITE / SYNC_WRITE except torque-off is refused; after
it, a write addressed to another servo's secondary ID is refused. Torque-off is always
allowed, so an E-stop can never be blocked.
"""

from __future__ import annotations

import time
from collections.abc import Callable, Iterator, Sequence
from contextlib import contextmanager
from dataclasses import dataclass
from enum import Enum
from typing import Any

import serial
from scservo_sdk import PortHandler, scscl
from scservo_sdk.protocol_packet_handler import (
    PKT_ID,
    PKT_INSTRUCTION,
    PKT_LENGTH,
    PKT_PARAMETER0,
)
from scservo_sdk.scservo_def import (
    BROADCAST_ID,
    COMM_RX_CORRUPT,
    COMM_RX_TIMEOUT,
    COMM_SUCCESS,
    INST_PING,
    INST_READ,
    INST_SYNC_WRITE,
    INST_WRITE,
)

from .models import SCS0009, ServoModel
from .registers import MAX_SERVO_ID, REG_ID, REG_TORQUE_ENABLE, SUPPORTED_BAUDRATES

# Secondary ID -> primary IDs of the HLS servos that also act on WRITEs addressed to it.
Census = dict[int, frozenset[int]]


class UnsafeOperationError(RuntimeError):
    """Raised instead of sending a packet the active policy does not allow."""


class WritePolicy:
    """Decides which WRITE / SYNC_WRITE packets may reach the wire. PING/READ are always allowed.

    The base class allows single-byte WRITEs to `allowed_write_addresses` only and no SYNC_WRITE.
    `lock_address` is the EEPROM lock of the servo model the policy was built for (its value
    must be 0 or 1); `servo_id`, when given, binds the policy to that one servo.
    """

    def __init__(
        self,
        name: str,
        allowed_write_addresses: frozenset[int] = frozenset(),
        lock_address: int = SCS0009.reg_lock,
        servo_id: int | None = None,
    ) -> None:
        self.name = name
        self.allowed_write_addresses = allowed_write_addresses
        self.lock_address = lock_address
        self.servo_id = servo_id

    def check_write(self, packet: Sequence[int]) -> None:
        if not self.allowed_write_addresses:
            raise UnsafeOperationError(f"WRITE refused: bus is in {self.name} mode")
        if self.servo_id is not None and packet[PKT_ID] != self.servo_id:
            raise UnsafeOperationError(
                f"WRITE to ID {packet[PKT_ID]} refused: {self.name} mode is bound to ID {self.servo_id}"
            )
        if packet[PKT_LENGTH] != 4:
            raise UnsafeOperationError("only single-byte WRITEs are allowed")
        address, value = packet[PKT_PARAMETER0], packet[PKT_PARAMETER0 + 1]
        if address not in self.allowed_write_addresses:
            raise UnsafeOperationError(f"WRITE to register {address} refused in {self.name} mode")
        if address == self.lock_address and value not in (0, 1):
            raise UnsafeOperationError(f"lock value {value} refused")
        if address == REG_ID and value > MAX_SERVO_ID:
            raise UnsafeOperationError(f"ID value {value} refused")

    def check_sync_write(self, packet: Sequence[int]) -> None:
        raise UnsafeOperationError(f"SYNC_WRITE refused in {self.name} mode")

    def __repr__(self) -> str:
        return f"<WritePolicy {self.name}>"


READ_ONLY = WritePolicy("read-only")


def id_assignment_policy(model: ServoModel, servo_id: int | None = None) -> WritePolicy:
    """Single-byte WRITEs to the ID register and the model's EEPROM lock only."""
    return WritePolicy("id-assignment", frozenset({REG_ID, model.reg_lock}), model.reg_lock, servo_id)


ID_ASSIGNMENT = id_assignment_policy(SCS0009)


def validate_instruction_packet(packet: Sequence[int], policy: WritePolicy) -> None:
    if len(packet) < 6:
        raise UnsafeOperationError(f"malformed packet ({len(packet)} bytes)")
    servo_id = packet[PKT_ID]
    length = packet[PKT_LENGTH]
    instruction = packet[PKT_INSTRUCTION]

    if len(packet) < length + 3:
        raise UnsafeOperationError("packet shorter than its length field")

    if instruction == INST_SYNC_WRITE:
        if servo_id != BROADCAST_ID:
            raise UnsafeOperationError("SYNC_WRITE must use the broadcast address")
        policy.check_sync_write(packet)
        return

    if servo_id > MAX_SERVO_ID:
        raise UnsafeOperationError(f"ID 0x{servo_id:02X} refused: broadcast/reserved IDs are never addressed")

    if instruction == INST_PING:
        if length != 2:
            raise UnsafeOperationError("PING with parameters refused")
        return

    if instruction == INST_READ:
        if length != 4:
            raise UnsafeOperationError("READ with unexpected parameter count refused")
        address, count = packet[PKT_PARAMETER0], packet[PKT_PARAMETER0 + 1]
        if count < 1 or address + count > 0x100:
            raise UnsafeOperationError(f"READ addr={address} len={count} out of range")
        return

    if instruction == INST_WRITE:
        policy.check_write(packet)
        return

    raise UnsafeOperationError(f"instruction 0x{instruction:02X} is never sent by this tool")


def _write_targets(packet: Sequence[int]) -> tuple[list[int], bool] | None:
    """(addressed IDs, is torque-off) of a WRITE / SYNC_WRITE; None for other instructions."""
    instruction = packet[PKT_INSTRUCTION]
    address = packet[PKT_PARAMETER0]
    if instruction == INST_WRITE:
        data = list(packet[PKT_PARAMETER0 + 1 : PKT_PARAMETER0 + 1 + packet[PKT_LENGTH] - 3])
        return [packet[PKT_ID]], address == REG_TORQUE_ENABLE and data == [0]
    if instruction == INST_SYNC_WRITE:
        data_len = packet[PKT_PARAMETER0 + 1]
        payload = packet[PKT_PARAMETER0 + 2 : PKT_PARAMETER0 + 2 + packet[PKT_LENGTH] - 4]
        stride = data_len + 1
        entries = [payload[i : i + stride] for i in range(0, len(payload), stride)]
        torque_off = address == REG_TORQUE_ENABLE and data_len == 1 and all(list(e[1:]) == [0] for e in entries)
        return [e[0] for e in entries], torque_off
    return None


def check_census(packet: Sequence[int], census: Census | None) -> None:
    """Refuse writes that could also reach an HLS servo through its secondary ID."""
    targets = _write_targets(packet)
    if targets is None:
        return
    ids, torque_off = targets
    if torque_off:
        return
    if census is None:
        raise UnsafeOperationError(
            "WRITE refused: no bus census yet, so HLS secondary IDs are unknown; only torque-off may be sent"
        )
    for servo_id in ids:
        others = census.get(servo_id, frozenset()) - {servo_id}
        if others:
            raise UnsafeOperationError(
                f"WRITE to ID {servo_id} refused: it is the secondary ID of HLS servo(s) {sorted(others)}, "
                "which would act on it too"
            )


def _open_serial(port_name: str, baudrate: int) -> Any:
    # exclusive: a second process (CLI next to the GUI) must not interleave packets on the bus.
    return serial.Serial(port=port_name, baudrate=baudrate, bytesize=serial.EIGHTBITS, timeout=0, exclusive=True)


class TrafficLoggingPortHandler(PortHandler):
    """SDK PortHandler that records raw traffic and re-validates every outgoing frame."""

    def __init__(
        self,
        port_name: str,
        latency_ms: float,
        tx_validator: Callable[[Sequence[int]], None],
        serial_factory: Callable[[str, int], Any] | None = None,
    ) -> None:
        super().__init__(port_name)
        self.latency_ms = latency_ms
        self._tx_validator = tx_validator
        self._serial_factory = serial_factory or _open_serial
        self.events: list[tuple[str, bytes]] = []

    def getCFlagBaud(self, baudrate: int) -> int:
        return baudrate if baudrate in SUPPORTED_BAUDRATES else -1

    def setupPort(self, cflag_baud: int) -> bool:
        if self.is_open:
            self.closePort()
        self.ser = self._serial_factory(self.port_name, self.baudrate)
        self.is_open = True
        self.ser.reset_input_buffer()
        self.tx_time_per_byte = (1000.0 / self.baudrate) * 10.0
        return True

    def setPacketTimeout(self, packet_length: int) -> None:
        self.packet_start_time = self.getCurrentTime()
        self.packet_timeout = self.tx_time_per_byte * (packet_length + 3.0) + self.latency_ms

    def writePort(self, packet: Sequence[int]) -> int:
        data = bytes(packet)
        try:
            self._tx_validator(list(data))
        except UnsafeOperationError:
            # txPacket already marked the port busy; release it so the bus stays usable.
            self.is_using = False
            raise
        self.events.append(("tx", data))
        return self.ser.write(data)

    def readPort(self, length: int) -> bytes:
        data = self.ser.read(length)
        if data:
            self.events.append(("rx", bytes(data)))
        return data


class GuardedPacketHandler(scscl):
    """The SDK handler; only its packet framing and transport are used (byte order is handled by the caller)."""

    def __init__(self, port_handler: PortHandler, validator: Callable[[Sequence[int]], None]) -> None:
        super().__init__(port_handler)
        self._validator = validator

    def txPacket(self, txpacket: list[int]) -> int:
        self._validator(txpacket)
        return super().txPacket(txpacket)


class CommResult(str, Enum):
    OK = "OK"
    TIMEOUT = "TIMEOUT"
    CORRUPT = "CORRUPT"
    FAIL = "FAIL"


_RESULT_MAP = {
    COMM_SUCCESS: CommResult.OK,
    COMM_RX_TIMEOUT: CommResult.TIMEOUT,
    COMM_RX_CORRUPT: CommResult.CORRUPT,
}


@dataclass
class Transaction:
    op: str
    servo_id: int
    address: int | None
    length: int | None
    result: CommResult
    error: int
    data: list[int]
    tx: bytes
    rx: bytes
    latency_ms: float
    detail: str = ""

    @property
    def ok(self) -> bool:
        return self.result is CommResult.OK

    def to_dict(self) -> dict[str, Any]:
        return {
            "op": self.op,
            "id": self.servo_id,
            "address": self.address,
            "length": self.length,
            "result": self.result.value,
            "error": self.error,
            "data": self.data,
            "tx": self.tx.hex(" "),
            "rx": self.rx.hex(" "),
            "latency_ms": round(self.latency_ms, 3),
            **({"detail": self.detail} if self.detail else {}),
        }


class SafeBus:
    """The only object the rest of the tool talks to. SDK write helpers are not exposed."""

    def __init__(
        self,
        port_name: str,
        baudrate: int,
        latency_ms: float = 20.0,
        serial_factory: Callable[[str, int], Any] | None = None,
        policy: WritePolicy = READ_ONLY,
    ) -> None:
        if baudrate not in SUPPORTED_BAUDRATES:
            raise ValueError(f"unsupported baudrate {baudrate}")
        self._policy = policy
        self._baudrate = baudrate
        self._port = TrafficLoggingPortHandler(
            port_name,
            latency_ms,
            tx_validator=self._validate,
            serial_factory=serial_factory,
        )
        self._handler = GuardedPacketHandler(self._port, self._validate)
        # None until `diagnostics.take_census` ran; {} means no HLS servo was found.
        self.census: Census | None = None
        self.history: list[Transaction] = []
        # Long monitor runs keep their own samples; per-transaction history would grow unbounded.
        self.keep_history = True

    def __enter__(self) -> SafeBus:
        self.open()
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    @property
    def port_name(self) -> str:
        return self._port.getPortName()

    @property
    def baudrate(self) -> int:
        return self._baudrate

    @property
    def policy(self) -> WritePolicy:
        return self._policy

    def set_policy(self, policy: WritePolicy) -> None:
        self._policy = policy

    def _validate(self, packet: Sequence[int]) -> None:
        validate_instruction_packet(packet, self._policy)
        check_census(packet, self.census)

    def open(self) -> None:
        if not self._port.setBaudRate(self._baudrate):
            raise ValueError(f"unsupported baudrate {self._baudrate}")

    def close(self) -> None:
        if self._port.is_open:
            self._port.closePort()

    def set_baudrate(self, baudrate: int) -> None:
        if baudrate not in SUPPORTED_BAUDRATES:
            raise ValueError(f"unsupported baudrate {baudrate}")
        self._baudrate = baudrate
        self._port.setBaudRate(baudrate)

    def listen(self, seconds: float) -> bytes:
        """Receive without transmitting anything; used to check the idle line."""
        self._port.clearPort()
        time.sleep(seconds)
        available = self._port.getBytesAvailable()
        data = bytes(self._port.readPort(available)) if available else b""
        self._port.events.clear()
        return data

    def ping(self, servo_id: int) -> Transaction:
        # Built by hand: the SDK's ping() follows up with a model-number READ, which
        # would double the traffic and blur which request a corrupt reply belongs to.
        packet = [0] * 6
        packet[PKT_ID] = servo_id
        packet[PKT_LENGTH] = 2
        packet[PKT_INSTRUCTION] = INST_PING

        def call() -> tuple[list[int], int, int]:
            _, result, error = self._handler.txRxPacket(packet)
            return [], result, error

        return self._transact("ping", servo_id, None, None, call)

    def read(self, servo_id: int, address: int, length: int) -> Transaction:
        return self._transact(
            "read", servo_id, address, length, lambda: self._handler.readTxRx(servo_id, address, length)
        )

    def write_byte(self, servo_id: int, address: int, value: int) -> Transaction:
        """Refused by the guard unless the active policy allows this register and value."""
        return self.write_block(servo_id, address, bytes([value]))

    def write_block(self, servo_id: int, address: int, data: bytes) -> Transaction:
        """WRITE with status reply; `data` is already in wire order (the target model's byte order)."""

        def call() -> tuple[list[int], int, int]:
            result, error = self._handler.writeTxRx(servo_id, address, len(data), list(data))
            return [], result, error

        return self._transact("write", servo_id, address, len(data), call)

    def sync_write(self, address: int, data_len: int, entries: dict[int, bytes]) -> Transaction:
        """One broadcast SYNC_WRITE packet; servos do not reply, so confirm by reading back."""
        param: list[int] = []
        for servo_id, data in entries.items():
            if len(data) != data_len:
                raise ValueError(f"entry for ID {servo_id} has {len(data)} bytes, expected {data_len}")
            param.append(servo_id)
            param.extend(data)

        def call() -> tuple[list[int], int, int]:
            # The SDK's GroupSyncWrite keeps hidden queued state; build the packet directly instead.
            result = self._handler.syncWriteTxOnly(address, data_len, param, len(param))
            return [], result, 0

        return self._transact("sync_write", BROADCAST_ID, address, data_len, call)

    @contextmanager
    def id_assignment_mode(self, model: ServoModel = SCS0009, servo_id: int | None = None) -> Iterator[None]:
        with self.use_policy(id_assignment_policy(model, servo_id)):
            yield

    @contextmanager
    def use_policy(self, policy: WritePolicy) -> Iterator[None]:
        """Temporarily switch policy; the previous one is always restored."""
        previous = self._policy
        self._policy = policy
        try:
            yield
        finally:
            self._policy = previous

    def _transact(
        self,
        op: str,
        servo_id: int,
        address: int | None,
        length: int | None,
        call: Callable[[], tuple[list[int], int, int]],
    ) -> Transaction:
        self._port.events.clear()
        start = time.perf_counter()
        detail = ""
        try:
            data, code, error = call()
        except (serial.SerialException, OSError) as exc:
            # USB unplug and similar: report a failed exchange instead of killing the caller.
            data, code, error, detail = [], None, 0, f"{type(exc).__name__}: {exc}"
        finally:
            # The SDK's busy flag is a plain bool; an exception mid-exchange would leave it set
            # and every later packet (E-stop included) would be refused as "port busy".
            self._port.is_using = False
        latency_ms = (time.perf_counter() - start) * 1000.0
        tx = b"".join(chunk for kind, chunk in self._port.events if kind == "tx")
        rx = b"".join(chunk for kind, chunk in self._port.events if kind == "rx")
        self._port.events.clear()
        transaction = Transaction(
            op=op,
            servo_id=servo_id,
            address=address,
            length=length,
            result=_RESULT_MAP.get(code, CommResult.FAIL),
            error=error,
            data=list(data),
            tx=tx,
            rx=rx,
            latency_ms=latency_ms,
            detail=detail,
        )
        if self.keep_history:
            self.history.append(transaction)
        return transaction
