from __future__ import annotations

from dexhand_servo.registers import BAUD_CODES, DUMP_END, decode, word_be


def test_word_is_big_endian() -> None:
    assert word_be(0x01, 0xF4) == 500


def test_decode_present_block() -> None:
    # position 500, speed -10 (bit 15 sign), load -20 % raw 200 (bit 10 sign), 5.0 V, 31 degC
    block = [0x01, 0xF4, 0x80, 0x0A, 0x04, 0xC8, 50, 31]
    d = decode(block, base_addr=56)
    assert d["present_position"].value == 500
    assert d["present_speed"].value == -10
    assert d["present_load"].value == -20.0
    assert d["present_voltage"].value == 5.0
    assert d["present_temperature"].value == 31


def test_decode_skips_unread_bytes() -> None:
    memory: list[int | None] = [0] * DUMP_END
    memory[56] = None
    d = decode(memory)
    assert "present_position" not in d
    assert "present_voltage" in d


def test_baud_codes() -> None:
    assert BAUD_CODES[0] == 1_000_000
    assert len(BAUD_CODES) == 8


def test_real_hls_dump_decodes_per_manual() -> None:
    from hls_fixture import REAL_HLS_DUMP

    from dexhand_servo.models import HLS3606, SCS0009, identify

    assert len(REAL_HLS_DUMP) == 71
    assert identify(REAL_HLS_DUMP[:5]) is HLS3606
    assert identify(bytes([3, 6, 1, 5, 9])) is SCS0009
    assert identify(bytes([3, 6, 0, 9, 9])) is None, "little-endian but not an HLS"
    d = HLS3606.decode(list(REAL_HLS_DUMP))
    expected = {
        "end_flag": 0, "servo_major_version": 10, "id": 1, "secondary_id": 253, "min_angle_limit": 0,
        "max_angle_limit": 0, "max_torque": 100.0, "phase": 118, "p_gain": 32, "protection_current": 5200.0,
        "angle_resolution": 1, "position_offset": 0, "mode": 0, "goal_current": 5200.0, "goal_speed": 250,
        "torque_limit": 100.0, "lock": 1, "present_position": 4, "present_voltage": 4.5,
        "present_temperature": 29, "present_goal": 4, "present_current": -32.5,
    }
    assert {k: d[k].value for k in expected} == expected
    assert d["present_current"].raw == 0x8005


def test_model_goal_block_and_speed_conversion() -> None:
    from dexhand_servo.models import HLS3606, SCS0009

    assert SCS0009.goal_block(500, 150) == bytes([0x01, 0xF4, 0, 0, 0x00, 0x96])
    assert HLS3606.goal_block(2048, 3, 800) == bytes([0x00, 0x08, 0x20, 0x03, 0x03, 0x00])
    assert HLS3606.parse_goal_block(HLS3606.goal_block(2048, 3, 800)) == (2048, 800, 3)
    assert SCS0009.speed_register(150) == 150 and SCS0009.scs_steps(5) == 5
    assert round(HLS3606.steps_per_speed_unit, 2) == 49.97
    assert HLS3606.speed_register(100) == 3 and HLS3606.speed_register(100, floor=True) == 2
    assert HLS3606.speed_register(10) == 1 and HLS3606.speed_register(10, floor=True) == 0
    assert HLS3606.speed_register(1e6) == HLS3606.max_speed_register
    assert HLS3606.scs_steps(5) == 17 and round(HLS3606.native_speed(50)) == 167
