"""Read-only diagnose stages S0-S5 (see README for the rationale of each stage)."""

from __future__ import annotations

import statistics
from collections.abc import Callable, Iterable
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Any

from serial.tools import list_ports

from .config import BRIDGE_PRODUCT, ESP32_USB_VID, WCH_USB_VID, BringupConfig
from .models import SCS0009, ServoModel, identify
from .registers import (
    BAUD_CODES,
    DUMP_END,
    IDENTITY_LEN,
    MAX_SERVO_ID,
    PRESENT_BLOCK_ADDR,
    PRESENT_BLOCK_LEN,
    REG_ID,
    DecodedValue,
)
from .safe_bus import Census, CommResult, SafeBus, Transaction

ProgressCallback = Callable[[str, int, int], None]


class Verdict(str, Enum):
    HEALTHY_SINGLE = "HEALTHY_SINGLE"
    SUSPECTED_ID_CONFLICT = "SUSPECTED_ID_CONFLICT"
    UNRELIABLE = "UNRELIABLE"
    NO_RESPONSE = "NO_RESPONSE"


class Severity(str, Enum):
    PASS = "pass"
    WARN = "warn"
    FAIL = "fail"


@dataclass
class StageResult:
    name: str
    severity: Severity
    message: str
    details: dict[str, Any] = field(default_factory=dict)


@dataclass
class Check:
    name: str
    severity: Severity
    detail: str


# ---------------------------------------------------------------- S0: port discovery


class PortKind(str, Enum):
    BRIDGE = "ESP32 bridge"  # firmware/urt_bridge in front of the URT-2
    WCH = "URT-2 USB"  # the URT-2's own USB-serial chip (also the DevKit's "UART" port on some boards)
    OTHER = "other"


@dataclass
class PortCandidate:
    device: str
    description: str
    vid: int | None
    pid: int | None
    kind: PortKind

    @property
    def is_adapter(self) -> bool:
        return self.kind is not PortKind.OTHER


class PortSelectionError(RuntimeError):
    pass


def port_kind(vid: int | None, product: str) -> PortKind:
    if vid == ESP32_USB_VID and BRIDGE_PRODUCT in product:
        return PortKind.BRIDGE
    if vid == WCH_USB_VID:
        return PortKind.WCH
    return PortKind.OTHER


def discover_ports() -> list[PortCandidate]:
    candidates: dict[str, PortCandidate] = {}
    for port in list_ports.comports():
        if port.vid is None:
            continue  # Bluetooth / debug-console pseudo ports
        # macOS: /dev/tty.* blocks on open waiting for DCD; /dev/cu.* is the right node.
        device = port.device.replace("/dev/tty.", "/dev/cu.", 1)
        candidates[device] = PortCandidate(
            device=device,
            description=port.description or "",
            vid=port.vid,
            pid=port.pid,
            kind=port_kind(port.vid, f"{port.product or ''} {port.description or ''}"),
        )
    return sorted(candidates.values(), key=lambda c: (not c.is_adapter, c.device))


def select_port(candidates: list[PortCandidate], explicit: str | None) -> str:
    """The only bus adapter (ESP32 bridge or URT-2 USB); never guesses between several."""
    if explicit:
        return explicit
    pool = [c for c in candidates if c.is_adapter] or candidates
    if len(pool) == 1:
        return pool[0].device
    if not pool:
        raise PortSelectionError(
            "No USB serial adapter found. Check the USB cable (URT-2 or ESP32 bridge) and that the "
            "USB-serial driver is loaded (`dexhand-servo list-ports`)."
        )
    names = ", ".join(f"{c.device} ({c.kind.value})" for c in pool)
    raise PortSelectionError(f"Several candidate ports ({names}); pass --port explicitly.")


# ---------------------------------------------------------------- S1/S2: link sanity


def check_idle_line(bus: SafeBus, seconds: float) -> StageResult:
    data = bus.listen(seconds)
    if not data:
        return StageResult("S1 idle line", Severity.PASS, f"no traffic in {seconds * 1000:.0f} ms")
    return StageResult(
        "S1 idle line",
        Severity.WARN,
        f"{len(data)} unsolicited bytes while idle: noise, wiring fault, or another bus master",
        {"bytes": data[:64].hex(" ")},
    )


def check_echo(bus: SafeBus, probe_id: int) -> StageResult:
    probe = bus.ping(probe_id)
    if probe.tx and probe.tx in probe.rx:
        return StageResult(
            "S2 echo",
            Severity.FAIL,
            "adapter echoes transmitted bytes; the SDK would mistake them for servo replies",
            {"tx": probe.tx.hex(" "), "rx": probe.rx.hex(" ")},
        )
    note = "no echo"
    if probe.result is not CommResult.TIMEOUT:
        note += f" (probe ID {probe_id} returned {probe.result.value}; a servo may already use that ID)"
    return StageResult("S2 echo", Severity.PASS, note, {"probe_result": probe.result.value})


# ---------------------------------------------------------------- S3: ID scan


@dataclass
class ScanEntry:
    servo_id: int
    outcome: CommResult
    attempts: int


def scan_ids(
    bus: SafeBus, ids: tuple[int, ...], retries: int, progress: ProgressCallback | None = None
) -> list[ScanEntry]:
    """Return entries for IDs that produced any reply (clean or corrupt)."""
    responders: list[ScanEntry] = []
    for index, servo_id in enumerate(ids):
        saw_corrupt = False
        outcome = CommResult.TIMEOUT
        attempts = 0
        for _ in range(1 + retries):
            attempts += 1
            result = bus.ping(servo_id).result
            if result is CommResult.OK:
                outcome = CommResult.OK
                break
            saw_corrupt |= result is CommResult.CORRUPT
        if outcome is not CommResult.OK and saw_corrupt:
            outcome = CommResult.CORRUPT
        if outcome is not CommResult.TIMEOUT:
            responders.append(ScanEntry(servo_id, outcome, attempts))
        if progress:
            progress("scan", index + 1, len(ids))
    return responders


# ---------------------------------------------------------------- S4: reliability / conflict


@dataclass
class OpStats:
    attempts: int = 0
    ok: int = 0
    timeout: int = 0
    corrupt: int = 0
    other: int = 0
    latency_p50_ms: float | None = None
    latency_p99_ms: float | None = None

    @property
    def success_rate(self) -> float:
        return self.ok / self.attempts if self.attempts else 0.0

    @classmethod
    def from_transactions(cls, transactions: list[Transaction]) -> OpStats:
        stats = cls(attempts=len(transactions))
        latencies = []
        for t in transactions:
            if t.result is CommResult.OK:
                stats.ok += 1
                latencies.append(t.latency_ms)
            elif t.result is CommResult.TIMEOUT:
                stats.timeout += 1
            elif t.result is CommResult.CORRUPT:
                stats.corrupt += 1
            else:
                stats.other += 1
        if latencies:
            latencies.sort()
            stats.latency_p50_ms = round(statistics.median(latencies), 3)
            stats.latency_p99_ms = round(latencies[min(len(latencies) - 1, int(len(latencies) * 0.99))], 3)
        return stats

    def to_dict(self) -> dict[str, Any]:
        return {
            "attempts": self.attempts,
            "ok": self.ok,
            "timeout": self.timeout,
            "corrupt": self.corrupt,
            "other": self.other,
            "success_rate": round(self.success_rate, 4),
            "latency_p50_ms": self.latency_p50_ms,
            "latency_p99_ms": self.latency_p99_ms,
        }


@dataclass
class LinkAssessment:
    servo_id: int
    verdict: Verdict
    reasons: list[str]
    ping: OpStats
    read: OpStats
    id_register_values: list[int]
    positions: list[int]
    error_bytes: list[int]
    aborted_early: bool
    last_present: dict[str, DecodedValue]

    @property
    def position_spread(self) -> int | None:
        return max(self.positions) - min(self.positions) if self.positions else None


def assess_link(
    bus: SafeBus,
    servo_id: int,
    cfg: BringupConfig,
    read_trials: int | None = None,
    model: ServoModel = SCS0009,
) -> LinkAssessment:
    ping_txns: list[Transaction] = []
    read_txns: list[Transaction] = []
    corrupt = 0
    aborted = False

    def note(t: Transaction) -> bool:
        nonlocal corrupt, aborted
        if t.result is CommResult.CORRUPT:
            corrupt += 1
            if corrupt >= cfg.max_corrupt_frames:
                aborted = True
        return aborted

    # Reads go first: present position differs from servo to servo, so overlapping
    # replies from same-ID servos come out corrupt here, while identical PING replies
    # can overlap cleanly. Detecting a conflict early limits bus contention.
    positions: list[int] = []
    last_present: dict[str, DecodedValue] = {}
    for _ in range(cfg.read_trials if read_trials is None else read_trials):
        t = bus.read(servo_id, PRESENT_BLOCK_ADDR, PRESENT_BLOCK_LEN)
        read_txns.append(t)
        if t.ok and len(t.data) == PRESENT_BLOCK_LEN:
            last_present = model.decode(t.data, base_addr=PRESENT_BLOCK_ADDR)
            positions.append(int(last_present["present_position"].value))
        if note(t):
            break

    id_values: list[int] = []
    if not aborted:
        for _ in range(cfg.id_register_trials):
            t = bus.read(servo_id, REG_ID, 1)
            read_txns.append(t)
            if t.ok and t.data:
                id_values.append(t.data[0])
            if note(t):
                break

    if not aborted:
        for _ in range(cfg.ping_trials):
            t = bus.ping(servo_id)
            ping_txns.append(t)
            if note(t):
                break

    ping_stats = OpStats.from_transactions(ping_txns)
    read_stats = OpStats.from_transactions(read_txns)
    error_bytes = sorted({t.error for t in ping_txns + read_txns if t.ok and t.error})

    reasons: list[str] = []
    if corrupt:
        reasons.append(f"{corrupt} corrupt frame(s): overlapping replies from several servos on one ID, or poor signal integrity")
    if any(v != servo_id for v in id_values):
        reasons.append(f"ID register read back {sorted(set(id_values))}, expected {servo_id}")
    # Position spread alone is not evidence of a conflict: overlapping replies with
    # differing positions cannot all pass the checksum, so a real conflict always
    # shows corrupt frames. Clean frames with spread mean pot noise or back-driving.
    spread = max(positions) - min(positions) if positions else 0

    total_ok = ping_stats.ok + read_stats.ok
    total = ping_stats.attempts + read_stats.attempts
    if reasons:
        verdict = Verdict.SUSPECTED_ID_CONFLICT
    elif total_ok == 0:
        verdict = Verdict.NO_RESPONSE
        reasons.append("no clean reply during the reliability test")
    elif total_ok / total < cfg.healthy_success_rate:
        verdict = Verdict.UNRELIABLE
        reasons.append(
            f"success rate {total_ok}/{total}; timeouts without corruption usually mean supply or wiring trouble"
        )
    else:
        verdict = Verdict.HEALTHY_SINGLE
        if total_ok < total:
            reasons.append(f"{total - total_ok} timeout(s) out of {total} transactions")
        if spread > cfg.position_spread_max:
            reasons.append(f"present position varies by {spread} steps across clean reads (see sanity checks)")

    return LinkAssessment(
        servo_id=servo_id,
        verdict=verdict,
        reasons=reasons,
        ping=ping_stats,
        read=read_stats,
        id_register_values=id_values,
        positions=positions,
        error_bytes=error_bytes,
        aborted_early=aborted,
        last_present=last_present,
    )


# ---------------------------------------------------------------- S5: register dump + checks


def read_identity(bus: SafeBus, servo_id: int) -> tuple[ServoModel | None, list[int] | None]:
    """(model, registers 0..4); the model is None for an unsupported servo, both are None without a clean reply."""
    t = bus.read(servo_id, 0, IDENTITY_LEN)
    if not (t.ok and len(t.data) == IDENTITY_LEN):
        return None, None
    return identify(t.data), list(t.data)


class CensusError(RuntimeError):
    """The bus census could not be completed; writes stay refused."""


CENSUS_READ_LEN = 8  # registers 0..7: identity plus the HLS secondary ID


def take_census(bus: SafeBus, ids: Iterable[int] | None = None, retries: int = 1) -> Census:
    """Find every HLS secondary ID on the bus and store the result in `bus.census`.

    PINGs every ID (0..253 by default) and reads registers 0..7 of each responder. Fails,
    leaving `bus.census` unchanged, when a responder cannot be read cleanly or is not a
    supported model: its secondary IDs would then be unknown. Transactions are not kept
    in `bus.history`.
    """
    secondaries: dict[int, set[int]] = {}
    keep_history = bus.keep_history
    bus.keep_history = False
    try:
        for servo_id in range(MAX_SERVO_ID + 1) if ids is None else ids:
            if all(bus.ping(servo_id).result is CommResult.TIMEOUT for _ in range(retries + 1)):
                continue
            t = bus.read(servo_id, 0, CENSUS_READ_LEN)
            if not (t.ok and len(t.data) == CENSUS_READ_LEN):
                raise CensusError(f"ID {servo_id} answers PING but registers 0..7 do not read cleanly "
                                  f"({t.result.value}; several servos on one ID?); its secondary ID is unknown")
            model = identify(t.data)
            if model is None:
                raise CensusError(f"ID {servo_id} is an unsupported servo (registers 0..4 = {t.data[:5]}); "
                                  "whether it answers other IDs is unknown")
            if model.reg_secondary_id is not None:
                secondaries.setdefault(t.data[model.reg_secondary_id], set()).add(servo_id)
    finally:
        bus.keep_history = keep_history
    census = {secondary: frozenset(primaries) for secondary, primaries in secondaries.items()}
    bus.census = census
    return census


def ensure_census(bus: SafeBus) -> Census:
    return bus.census if bus.census is not None else take_census(bus)


@dataclass
class RegisterDump:
    servo_id: int
    raw: list[int | None]
    failed_chunks: list[tuple[int, int, str]]
    decoded: dict[str, DecodedValue]
    model: ServoModel | None = None


def dump_registers(bus: SafeBus, servo_id: int, chunk: int = 16) -> RegisterDump:
    """Read the whole memory table of one servo; the model is identified from the first chunk.

    An unidentified servo gets the SCS0009 range (0..70) as a raw dump with nothing decoded.
    """
    raw: list[int | None] = [None] * DUMP_END
    failed: list[tuple[int, int, str]] = []
    model: ServoModel | None = None
    end = DUMP_END
    start = 0
    while start < end:
        length = min(chunk, end - start)
        t = bus.read(servo_id, start, length)
        if t.ok and len(t.data) == length:
            raw[start : start + length] = t.data
        else:
            failed.append((start, length, t.result.value))
        if start == 0:
            model = identify(raw[:IDENTITY_LEN])
            if model is not None and model.dump_end != end:
                end = model.dump_end
                raw = (raw + [None] * end)[:end]
        start += length
    return RegisterDump(servo_id, raw, failed, model.decode(raw) if model else {}, model)


def run_checks(
    dump: RegisterDump, assessment: LinkAssessment, baudrate: int, cfg: BringupConfig
) -> list[Check]:
    d = dump.decoded
    model = dump.model
    checks: list[Check] = []
    joint = cfg.joints.get(dump.servo_id)

    def add(name: str, ok: bool, pass_detail: str, fail_detail: str, severity: Severity = Severity.WARN) -> None:
        checks.append(Check(name, Severity.PASS if ok else severity, pass_detail if ok else fail_detail))

    def value(name: str) -> Any:
        return d[name].value if name in d else None

    if dump.failed_chunks:
        spans = ", ".join(f"{s}..{s + n - 1} ({r})" for s, n, r in dump.failed_chunks)
        checks.append(Check("register dump", Severity.WARN, f"unreadable spans: {spans}"))

    if model is None:
        identity = dump.raw[:IDENTITY_LEN]
        checks.append(Check("servo model", Severity.WARN,
                            f"unsupported servo (registers 0..4 = {identity}); raw dump only, nothing decoded"))
        add("status packet error byte", not assessment.error_bytes, "always 0",
            f"non-zero error bytes seen: {[hex(e) for e in assessment.error_bytes]}")
        return checks

    identity = f"{model.label} (END={model.end_flag}, {model.byteorder}-endian)"
    if joint is None:
        add("servo model", True, identity, "")
    else:
        add("servo model", joint.model == model.key, f"{identity}, as configured for {joint.name}",
            f"servo is {identity} but config.py JOINTS says {joint.model} for {joint.name}", Severity.FAIL)

    servo_id = value("id")
    add("id register", servo_id == dump.servo_id, f"ID register = {servo_id}",
        f"ID register = {servo_id}, addressed as {dump.servo_id}", Severity.FAIL)

    baud_code = value("baud_code")
    add("baud register", BAUD_CODES.get(baud_code) == baudrate, f"code {baud_code} = {baudrate} bps",
        f"code {baud_code} does not match the {baudrate} bps link")

    voltage = value("present_voltage")
    lo, hi = cfg.supply_voltage_window
    add("supply voltage", voltage is not None and lo <= voltage <= hi, f"{voltage} V",
        f"{voltage} V outside expected {lo}-{hi} V")

    temperature = value("present_temperature")
    add("temperature", temperature is not None and temperature < cfg.temperature_max_c, f"{temperature} degC",
        f"{temperature} degC (limit {cfg.temperature_max_c})")

    torque = value("torque_enable")
    add("torque at power-up", torque == 0, "torque off (free)",
        f"torque_enable = {torque}; left unchanged by this tool")

    position, lower, upper = value("present_position"), value("min_angle_limit"), value("max_angle_limit")
    multi_turn = model.is_hls and lower == 0 and upper == 0
    if multi_turn:
        add("servo angle limits", False, "",
            "0/0: multi-turn mode, the servo does not limit travel itself; measure the joint, then write its "
            "travel with `hls-setup --limits-from-config` (README, HLS bring-up)")
    else:
        add("within EEPROM angle limits",
            None not in (position, lower, upper) and lower <= position <= upper,
            f"{position} in [{lower}, {upper}]",
            f"{position} vs limits [{lower}, {upper}]")

    if joint is not None and joint.has_travel_limits:
        travel_lo, travel_hi = joint.min_position, joint.max_position
        add("within joint travel",
            position is not None and travel_lo <= position <= travel_hi,
            f"{position} in configured [{travel_lo}, {travel_hi}] ({joint.name})",
            f"{position} outside configured [{travel_lo}, {travel_hi}] for {joint.name}: "
            "check the limits, the ID-to-joint mapping, or the joint assembly")
        if not multi_turn:
            add("travel inside EEPROM limits",
                None not in (lower, upper) and lower <= travel_lo and travel_hi <= upper,
                f"[{travel_lo}, {travel_hi}] inside [{lower}, {upper}]",
                f"configured [{travel_lo}, {travel_hi}] exceeds the servo's angle limits [{lower}, {upper}]")

    if model.is_hls:
        mode = value("mode")
        add("operating mode", mode == 0, "0 = position servo",
            f"mode = {mode}; only position-servo mode 0 is supported (motion is refused)")
        resolution = value("angle_resolution")
        add("angle resolution", resolution == 1, "1 (4096 steps per turn)",
            f"angle resolution = {resolution}; step sizes assume 1 (motion is refused)")
        offset = value("position_offset")
        expected = joint.position_offset if joint is not None else 0
        add("position offset", offset == expected, f"{offset} steps",
            f"{offset} steps, config expects {expected}: travel limits measured with another offset "
            "do not match this servo (motion is refused)")
        if position is not None and position < 0:
            checks.append(Check("position sign", Severity.WARN,
                                f"present position {position} is negative (full-angle feedback crossed the zero "
                                "seam); power-cycle the servo with the joint inside its travel"))

    spread = assessment.position_spread
    spread_max = model.scs_steps(cfg.position_spread_max)
    sensor = "encoder noise" if model.is_hls else "potentiometer noise"
    add("position stability", spread is not None and spread <= spread_max,
        f"spread {spread} steps over {len(assessment.positions)} reads",
        f"spread {spread} steps over {len(assessment.positions)} reads with torque off: "
        f"{sensor}, supply ripple, or the joint being back-driven (tendon / return spring)")

    status = value("status")
    add("status register", status == 0, "0", f"status = 0x{(status or 0):02X}")

    add("status packet error byte", not assessment.error_bytes, "always 0",
        f"non-zero error bytes seen: {[hex(e) for e in assessment.error_bytes]}")

    moving = value("moving")
    add("moving flag", moving == 0, "stationary", f"moving = {moving} although nothing was commanded")
    return checks


def secondary_id_collisions(report_servos: list[ServoReport], joint_ids: set[int]) -> list[str]:
    """HLS servos whose secondary ID equals another servo's or joint's ID: a WRITE meant for that ID reaches both."""
    present = {s.assessment.servo_id for s in report_servos}
    problems = []
    for servo in report_servos:
        dump = servo.dump
        if dump is None or dump.model is None or "secondary_id" not in dump.decoded:
            continue
        secondary = int(dump.decoded["secondary_id"].value)
        if secondary != dump.servo_id and secondary in present | joint_ids:
            role = "a responding servo" if secondary in present else "a configured joint"
            problems.append(f"ID {dump.servo_id} has secondary ID {secondary}, which is also {role}: "
                            f"a WRITE meant for ID {secondary} would also reach ID {dump.servo_id}")
    return problems


# ---------------------------------------------------------------- orchestration


@dataclass
class ServoReport:
    assessment: LinkAssessment
    dump: RegisterDump | None = None
    checks: list[Check] = field(default_factory=list)


@dataclass
class DiagnoseReport:
    started_at: str
    port: str
    baudrate: int
    scanned_ids: tuple[int, ...]
    stages: list[StageResult] = field(default_factory=list)
    responders: list[ScanEntry] = field(default_factory=list)
    servos: list[ServoReport] = field(default_factory=list)
    fleet_checks: list[Check] = field(default_factory=list)
    exit_code: int = 0
    summary: str = ""


def run_diagnose(
    bus: SafeBus,
    cfg: BringupConfig,
    ids: tuple[int, ...] | None = None,
    baud_sweep: bool = False,
    progress: ProgressCallback | None = None,
) -> DiagnoseReport:
    ids = ids if ids is not None else cfg.scan_ids
    report = DiagnoseReport(
        started_at=datetime.now().isoformat(timespec="seconds"),
        port=bus.port_name,
        baudrate=bus.baudrate,
        scanned_ids=ids,
    )

    report.stages.append(check_idle_line(bus, cfg.idle_listen_s))
    echo = check_echo(bus, cfg.echo_probe_id)
    report.stages.append(echo)
    if echo.severity is Severity.FAIL:
        report.exit_code = 2
        report.summary = "Aborted: adapter echo makes replies ambiguous."
        return report

    responders = scan_ids(bus, ids, cfg.scan_retries, progress)
    if not responders and baud_sweep:
        default_baudrate = bus.baudrate
        for baudrate in BAUD_CODES.values():
            if baudrate == bus.baudrate:
                continue
            bus.set_baudrate(baudrate)
            responders = scan_ids(bus, ids, cfg.scan_retries, progress)
            if responders:
                report.stages.append(
                    StageResult("S3 baud sweep", Severity.WARN, f"servos answer at {baudrate} bps, not the default")
                )
                break
        else:
            bus.set_baudrate(default_baudrate)
    report.baudrate = bus.baudrate
    report.responders = responders
    scan_msg = f"{len(responders)} responding ID(s) among {len(ids)} scanned at {bus.baudrate} bps"
    report.stages.append(StageResult("S3 ID scan", Severity.PASS if responders else Severity.FAIL, scan_msg))
    if not responders:
        report.exit_code = 2
        report.summary = "No servo replied. Check servo power, the 3-pin wiring order and the baud rate."
        return report

    for index, entry in enumerate(responders):
        model, _ = read_identity(bus, entry.servo_id)
        servo = ServoReport(assess_link(bus, entry.servo_id, cfg, model=model or SCS0009))
        if servo.assessment.verdict is Verdict.HEALTHY_SINGLE:
            servo.dump = dump_registers(bus, entry.servo_id)
            servo.checks = run_checks(servo.dump, servo.assessment, bus.baudrate, cfg)
        report.servos.append(servo)
        if progress:
            progress("assess", index + 1, len(responders))

    for problem in secondary_id_collisions(report.servos, set(cfg.joints)):
        report.fleet_checks.append(Check("secondary ID", Severity.FAIL, problem))
    healthy_ids = {s.assessment.servo_id for s in report.servos if s.assessment.verdict is Verdict.HEALTHY_SINGLE}
    missing = [i for i in cfg.expected_ids if i in ids and i not in healthy_ids]
    if missing:
        report.fleet_checks.append(Check("expected IDs", Severity.WARN, f"not healthy/absent: {missing}"))
    unmapped = sorted(i for i in healthy_ids if i not in cfg.joints)
    if unmapped:
        report.fleet_checks.append(Check("joint map", Severity.WARN, f"IDs {unmapped} are not in config JOINTS"))
    no_travel = sorted(i for i in healthy_ids if i in cfg.joints and not cfg.joints[i].has_travel_limits)
    if no_travel:
        report.fleet_checks.append(
            Check("joint travel limits", Severity.WARN, f"not configured yet for IDs {no_travel} (required before motion)")
        )

    unhealthy = [s for s in report.servos if s.assessment.verdict is not Verdict.HEALTHY_SINGLE]
    warnings = [c for s in report.servos for c in s.checks if c.severity is not Severity.PASS]
    warnings += [c for c in report.fleet_checks if c.severity is not Severity.PASS]
    stage_warnings = [s for s in report.stages if s.severity is not Severity.PASS]
    report.exit_code = 1 if (unhealthy or warnings or stage_warnings) else 0
    if any(s.assessment.verdict is Verdict.SUSPECTED_ID_CONFLICT for s in report.servos):
        report.summary = (
            "Link is up, but at least one ID has several servos behind it. Isolate servos "
            "one at a time and run `assign-id` (README step 3)."
        )
    elif report.exit_code == 0:
        report.summary = f"All {len(report.servos)} servo(s) individually reachable; registers read."
    else:
        report.summary = "Link is up; review the warnings above."
    return report
