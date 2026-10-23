"""Poses and teach recordings as JSON under `data/`.

Every file stores the ID -> joint-name map and the ID -> servo-model map it was made
with. Loading refuses a file whose maps disagree with the current `JOINTS` (files
written before models were recorded count as SCS0009), or whose positions fall outside
the current travel limits - so a re-wired, re-measured or re-motored hand never replays
stale positions (positions are raw steps of the servo model they were recorded on).
"""

from __future__ import annotations

import json
import re
from collections.abc import Mapping, Sequence
from datetime import datetime
from pathlib import Path
from typing import Any

from ..config import JointConfig
from ..models import SCS0009
from .trajectory import Keyframe

SCHEMA_VERSION = 1
DATA_DIR = Path(__file__).resolve().parents[2] / "data"
_SAFE_NAME = re.compile(r"[^A-Za-z0-9_-]+")


class StorageError(RuntimeError):
    pass


def safe_file_stem(name: str) -> str:
    """ASCII-only file stem (spaces and punctuation become '-'), never empty."""
    stem = _SAFE_NAME.sub("-", name.strip()).strip("-")
    if not stem:
        raise StorageError(f"'{name}' has no usable ASCII characters for a file name")
    return stem


def _joint_map(joints: Mapping[int, JointConfig]) -> dict[str, str]:
    return {str(servo_id): joint.name for servo_id, joint in joints.items()}


def _model_map(joints: Mapping[int, JointConfig]) -> dict[str, str]:
    return {str(servo_id): joint.model for servo_id, joint in joints.items()}


def _check_positions(positions: Mapping[int, int], joints: Mapping[int, JointConfig], where: str) -> None:
    for servo_id, value in positions.items():
        joint = joints.get(servo_id)
        if joint is None or not joint.has_travel_limits:
            raise StorageError(f"{where}: ID {servo_id} is not a configured joint with travel limits")
        assert joint.min_position is not None and joint.max_position is not None
        if not isinstance(value, int) or not joint.min_position <= value <= joint.max_position:
            raise StorageError(
                f"{where}: ID {servo_id} position {value} outside [{joint.min_position}, {joint.max_position}]"
            )


def _check_header(data: dict[str, Any], joints: Mapping[int, JointConfig], where: str) -> None:
    if data.get("schema_version") != SCHEMA_VERSION:
        raise StorageError(f"{where}: unsupported schema_version {data.get('schema_version')}")
    stored = data.get("joints", {})
    current = _joint_map(joints)
    mismatched = sorted(k for k in stored if current.get(k) != stored[k])
    if mismatched:
        raise StorageError(f"{where}: joint map differs from config for IDs {mismatched} "
                           "(hand re-wired or renamed?)")
    stored_models = data.get("models", {})
    current_models = _model_map(joints)
    changed = sorted(k for k in stored
                     if k in current_models and stored_models.get(k, SCS0009.key) != current_models[k])
    if changed:
        raise StorageError(f"{where}: servo model differs from config for IDs {changed} "
                           "(positions were recorded in another servo's steps)")


def builtin_poses(joints: Mapping[int, JointConfig]) -> dict[str, dict[int, int]]:
    """'center': every joint at the middle of its travel (always valid by construction)."""
    center = {
        servo_id: (joint.min_position + joint.max_position) // 2
        for servo_id, joint in joints.items()
        if joint.min_position is not None and joint.max_position is not None
    }
    return {"center": center}


class PoseLibrary:
    def __init__(self, path: Path, joints: Mapping[int, JointConfig]) -> None:
        self.path = path
        self.joints = joints

    def load(self) -> dict[str, dict[int, int]]:
        if not self.path.exists():
            return {}
        data = json.loads(self.path.read_text(encoding="utf-8"))
        _check_header(data, self.joints, self.path.name)
        poses = {}
        for name, raw in data.get("poses", {}).items():
            positions = {int(k): v for k, v in raw.items()}
            _check_positions(positions, self.joints, f"{self.path.name} pose '{name}'")
            poses[name] = positions
        return poses

    def save(self, name: str, positions: Mapping[int, int]) -> None:
        name = name.strip()
        if not name:
            raise StorageError("pose name is empty")
        if name in builtin_poses(self.joints):
            raise StorageError(f"'{name}' is a built-in pose name")
        clean = {int(k): int(v) for k, v in positions.items()}
        _check_positions(clean, self.joints, f"pose '{name}'")
        poses = self.load()
        poses[name] = clean
        self._write(poses)

    def delete(self, name: str) -> None:
        poses = self.load()
        poses.pop(name, None)
        self._write(poses)

    def _write(self, poses: Mapping[str, Mapping[int, int]]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        data = {
            "schema_version": SCHEMA_VERSION,
            "joints": _joint_map(self.joints),
            "models": _model_map(self.joints),
            "poses": {name: {str(k): v for k, v in sorted(p.items())} for name, p in sorted(poses.items())},
        }
        self.path.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")


def save_recording(
    directory: Path, name: str, keyframes: Sequence[Keyframe], joints: Mapping[int, JointConfig]
) -> Path:
    if not keyframes:
        raise StorageError("recording is empty")
    for index, keyframe in enumerate(keyframes):
        _check_positions(keyframe.positions, joints, f"keyframe {index} (t={keyframe.t:.2f} s)")
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{safe_file_stem(name)}.json"
    data = {
        "schema_version": SCHEMA_VERSION,
        "name": name,
        "created": datetime.now().isoformat(timespec="seconds"),
        "joints": _joint_map(joints),
        "models": _model_map(joints),
        "keyframes": [{"t": kf.t, "positions": {str(k): v for k, v in kf.positions.items()}} for kf in keyframes],
    }
    path.write_text(json.dumps(data, indent=1, ensure_ascii=False), encoding="utf-8")
    return path


def load_recording(path: Path, joints: Mapping[int, JointConfig]) -> list[Keyframe]:
    data = json.loads(path.read_text(encoding="utf-8"))
    _check_header(data, joints, path.name)
    keyframes = []
    for index, raw in enumerate(data.get("keyframes", [])):
        positions = {int(k): v for k, v in raw["positions"].items()}
        _check_positions(positions, joints, f"{path.name} keyframe {index}")
        keyframes.append(Keyframe(float(raw["t"]), positions))
    if not keyframes:
        raise StorageError(f"{path.name}: no keyframes")
    if any(b.t < a.t for a, b in zip(keyframes, keyframes[1:], strict=False)):
        raise StorageError(f"{path.name}: keyframe times are not increasing")
    return keyframes


def list_recordings(directory: Path) -> list[Path]:
    return sorted(directory.glob("*.json")) if directory.exists() else []
