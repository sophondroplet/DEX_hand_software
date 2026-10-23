"""Terminal rendering (rich) and JSON archiving of diagnose results."""

from __future__ import annotations

import json
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from typing import Any

from rich.console import Console
from rich.table import Table

from . import __version__
from .config import JOINT_MAP, BringupConfig
from .diagnostics import Check, DiagnoseReport, ServoReport, Severity, Verdict
from .safe_bus import Transaction

SEVERITY_STYLE = {Severity.PASS: "green", Severity.WARN: "yellow", Severity.FAIL: "red"}
VERDICT_STYLE = {
    Verdict.HEALTHY_SINGLE: "green",
    Verdict.SUSPECTED_ID_CONFLICT: "red",
    Verdict.UNRELIABLE: "yellow",
    Verdict.NO_RESPONSE: "red",
}


def _present(servo: ServoReport, name: str) -> str:
    source = servo.dump.decoded if servo.dump else servo.assessment.last_present
    return str(source[name].value) if name in source else "-"


def _model_label(servo: ServoReport) -> str:
    if servo.dump is None:
        return "-"
    return servo.dump.model.label if servo.dump.model else "unknown"


def render_diagnose(console: Console, report: DiagnoseReport) -> None:
    console.rule(f"[bold]Diagnose {report.port} @ {report.baudrate} bps")
    for stage in report.stages:
        style = SEVERITY_STYLE[stage.severity]
        console.print(f"[{style}]{stage.severity.value.upper():>4}[/] {stage.name}: {stage.message}")
        for key, value in stage.details.items():
            console.print(f"      {key}: {value}")

    if report.servos:
        table = Table(title="Per-ID link assessment", show_lines=False)
        for column in ("ID", "Joint", "Model", "Verdict", "Ping ok", "Read ok", "Corrupt", "p50/p99 ms",
                       "Pos", "Spread", "V", "degC", "Torque"):
            table.add_column(column)
        for servo in report.servos:
            a = servo.assessment
            style = VERDICT_STYLE[a.verdict]
            latency = f"{a.ping.latency_p50_ms}/{a.ping.latency_p99_ms}" if a.ping.latency_p50_ms else "-"
            table.add_row(
                str(a.servo_id),
                JOINT_MAP.get(a.servo_id, "-"),
                _model_label(servo),
                f"[{style}]{a.verdict.value}[/]",
                f"{a.ping.ok}/{a.ping.attempts}",
                f"{a.read.ok}/{a.read.attempts}",
                str(a.ping.corrupt + a.read.corrupt),
                latency,
                _present(servo, "present_position"),
                str(a.position_spread) if a.position_spread is not None else "-",
                _present(servo, "present_voltage"),
                _present(servo, "present_temperature"),
                _present(servo, "torque_enable") if servo.dump else "-",
            )
        console.print(table)

        for servo in report.servos:
            a = servo.assessment
            if a.reasons:
                console.print(f"[bold]ID {a.servo_id}[/] notes:")
                for reason in a.reasons:
                    console.print(f"  - {reason}")
            if servo.dump:
                _render_registers(console, servo)
            if servo.checks:
                _render_checks(console, f"ID {a.servo_id} sanity checks", servo.checks)

    if report.fleet_checks:
        _render_checks(console, "Fleet checks", report.fleet_checks)

    style = "green" if report.exit_code == 0 else ("yellow" if report.exit_code == 1 else "red")
    console.print(f"\n[{style} bold]{report.summary}[/] (exit {report.exit_code})")


def _render_registers(console: Console, servo: ServoReport) -> None:
    dump = servo.dump
    assert dump is not None
    if dump.model is not None:
        table = Table(title=f"ID {servo.assessment.servo_id} registers ({dump.model.label}, decoded, "
                            "verify against datasheet)")
        for column in ("Addr", "Name", "Area", "Raw", "Value", "Unit"):
            table.add_column(column)
        for reg in dump.model.registers:
            decoded = dump.decoded.get(reg.name)
            if decoded is None:
                continue
            table.add_row(str(reg.addr), reg.name, reg.area, str(decoded.raw), str(decoded.value), reg.unit)
        console.print(table)
    raw = dump.raw
    console.print(f"[dim]raw 0..{len(raw) - 1}:[/]")
    for start in range(0, len(raw), 16):
        row = " ".join("--" if b is None else f"{b:02X}" for b in raw[start : start + 16])
        console.print(f"[dim]  {start:3d}: {row}[/]")


def _render_checks(console: Console, title: str, checks: list[Check]) -> None:
    console.print(f"[bold]{title}[/]")
    for check in checks:
        style = SEVERITY_STYLE[check.severity]
        console.print(f"  [{style}]{check.severity.value.upper():>4}[/] {check.name}: {check.detail}")


# ---------------------------------------------------------------- JSON


def _check_dict(check: Check) -> dict[str, str]:
    return {"name": check.name, "severity": check.severity.value, "detail": check.detail}


def _servo_dict(servo: ServoReport) -> dict[str, Any]:
    a = servo.assessment
    out: dict[str, Any] = {
        "id": a.servo_id,
        "joint": JOINT_MAP.get(a.servo_id),
        "verdict": a.verdict.value,
        "reasons": a.reasons,
        "ping": a.ping.to_dict(),
        "read": a.read.to_dict(),
        "id_register_values": a.id_register_values,
        "positions": a.positions,
        "position_spread": a.position_spread,
        "error_bytes": a.error_bytes,
        "aborted_early": a.aborted_early,
    }
    if servo.dump:
        out["model"] = servo.dump.model.key if servo.dump.model else None
        out["registers_raw"] = servo.dump.raw
        out["registers_failed_chunks"] = servo.dump.failed_chunks
        out["registers"] = {
            name: {"addr": v.addr, "raw": v.raw, "value": v.value, "unit": v.unit}
            for name, v in servo.dump.decoded.items()
        }
        out["checks"] = [_check_dict(c) for c in servo.checks]
    return out


def _config_dict(cfg: BringupConfig) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for key, value in cfg.__dict__.items():
        if key == "scan_ids":
            continue
        if key == "joints":
            value = {str(servo_id): asdict(joint) for servo_id, joint in value.items()}
        elif isinstance(value, tuple):
            value = list(value)
        out[key] = value
    return out


def diagnose_to_dict(
    report: DiagnoseReport, cfg: BringupConfig, transactions: list[Transaction], all_traffic: bool
) -> dict[str, Any]:
    # Failed exchanges carry the diagnostic value; full logs on request only.
    traffic = transactions if all_traffic else [t for t in transactions if not t.ok]
    return {
        "tool_version": __version__,
        "started_at": report.started_at,
        "port": report.port,
        "baudrate": report.baudrate,
        "config": _config_dict(cfg),
        "scanned_ids": [min(report.scanned_ids), max(report.scanned_ids)] if report.scanned_ids else [],
        "stages": [
            {"name": s.name, "severity": s.severity.value, "message": s.message, "details": s.details}
            for s in report.stages
        ],
        "responders": [
            {"id": r.servo_id, "outcome": r.outcome.value, "attempts": r.attempts} for r in report.responders
        ],
        "servos": [_servo_dict(s) for s in report.servos],
        "fleet_checks": [_check_dict(c) for c in report.fleet_checks],
        "summary": report.summary,
        "exit_code": report.exit_code,
        "traffic_scope": "all" if all_traffic else "non-OK only",
        "transaction_count": len(transactions),
        "traffic": [t.to_dict() for t in traffic],
    }


def write_json(data: dict[str, Any], directory: Path, prefix: str) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{prefix}_{datetime.now():%Y%m%d_%H%M%S}.json"
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
    return path
