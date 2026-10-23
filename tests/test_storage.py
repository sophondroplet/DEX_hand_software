from __future__ import annotations

import json

import pytest

from dexhand_servo.config import JointConfig
from dexhand_servo.control.storage import (
    PoseLibrary,
    StorageError,
    builtin_poses,
    load_recording,
    safe_file_stem,
    save_recording,
)
from dexhand_servo.control.trajectory import Keyframe

JOINTS = {1: JointConfig("a", 300, 700), 2: JointConfig("b", 400, 600)}


def test_pose_roundtrip_and_validation(tmp_path) -> None:
    lib = PoseLibrary(tmp_path / "poses.json", JOINTS)
    lib.save("fist", {1: 650, 2: 590})
    assert lib.load() == {"fist": {1: 650, 2: 590}}
    with pytest.raises(StorageError, match="outside"):
        lib.save("bad", {1: 800})
    with pytest.raises(StorageError, match="built-in"):
        lib.save("center", {1: 500})
    lib.delete("fist")
    assert lib.load() == {}


def test_pose_file_from_differently_wired_hand_is_refused(tmp_path) -> None:
    lib = PoseLibrary(tmp_path / "poses.json", JOINTS)
    lib.save("open", {1: 400})
    rewired = {1: JointConfig("thumb", 300, 700), 2: JOINTS[2]}
    with pytest.raises(StorageError, match="joint map"):
        PoseLibrary(tmp_path / "poses.json", rewired).load()


def test_pose_outside_new_limits_is_refused(tmp_path) -> None:
    PoseLibrary(tmp_path / "poses.json", JOINTS).save("wide", {1: 680})
    narrower = {1: JointConfig("a", 300, 650), 2: JOINTS[2]}
    with pytest.raises(StorageError, match="outside"):
        PoseLibrary(tmp_path / "poses.json", narrower).load()


def test_recording_roundtrip(tmp_path) -> None:
    keyframes = [Keyframe(0.0, {1: 500, 2: 500}), Keyframe(0.5, {1: 520, 2: 480})]
    path = save_recording(tmp_path, "wave 1 / test", keyframes, JOINTS)
    assert path.name == "wave-1-test.json"
    assert load_recording(path, JOINTS) == keyframes
    data = json.loads(path.read_text(encoding="utf-8"))
    data["keyframes"][1]["positions"]["1"] = 900
    path.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(StorageError, match="outside"):
        load_recording(path, JOINTS)


def test_recording_with_out_of_range_frame_cannot_be_saved(tmp_path) -> None:
    with pytest.raises(StorageError, match="keyframe 1"):
        save_recording(tmp_path, "x", [Keyframe(0, {1: 500}), Keyframe(1, {1: 750})], JOINTS)


def test_helpers() -> None:
    assert builtin_poses(JOINTS) == {"center": {1: 500, 2: 500}}
    with pytest.raises(StorageError):
        safe_file_stem("握拳")


def test_files_from_another_servo_model_are_refused(tmp_path) -> None:
    lib = PoseLibrary(tmp_path / "poses.json", JOINTS)
    lib.save("open", {1: 400})
    assert json.loads((tmp_path / "poses.json").read_text(encoding="utf-8"))["models"] == {"1": "scs0009",
                                                                                         "2": "scs0009"}
    remotored = {1: JointConfig("a", 300, 3000, model="hls3606"), 2: JOINTS[2]}
    with pytest.raises(StorageError, match="servo model"):
        PoseLibrary(tmp_path / "poses.json", remotored).load()
    data = json.loads((tmp_path / "poses.json").read_text(encoding="utf-8"))
    del data["models"]  # files written before models were recorded count as SCS0009
    (tmp_path / "poses.json").write_text(json.dumps(data), encoding="utf-8")
    assert PoseLibrary(tmp_path / "poses.json", JOINTS).load() == {"open": {1: 400}}
    with pytest.raises(StorageError, match="servo model"):
        PoseLibrary(tmp_path / "poses.json", remotored).load()
