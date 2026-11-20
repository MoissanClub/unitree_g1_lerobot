import os
from types import SimpleNamespace

import pytest

from tools.g1_physical_replay import (
    KeyboardPause,
    PauseState,
    VoiceAlarm,
    approach_duration,
    validate_staging_contract,
)


def test_voice_alarm_selects_english_spd_say_language():
    calls = []

    def run(command, *, check):
        calls.append((command, check))

    VoiceAlarm("spd-say", "en", -35, runner=run).speak("Robot arms are moving.")

    assert calls == [
        (
            ["spd-say", "-w", "-l", "en", "-r", "-35", "Robot arms are moving."],
            True,
        )
    ]


def test_pause_state_freezes_plan_time_until_explicit_resume():
    pause = PauseState()

    assert pause.elapsed(10.0, 12.0) == 2.0
    assert pause.update("pause", 12.0) == "paused"
    assert pause.elapsed(10.0, 15.0) == 2.0
    assert pause.update("resume", 15.0) == "resumed"
    assert pause.elapsed(10.0, 17.0) == 4.0


def test_keyboard_pause_requires_an_interactive_terminal():
    stream = SimpleNamespace(isatty=lambda: False)

    with pytest.raises(RuntimeError, match="interactive terminal"), KeyboardPause(stream):
        pass


def test_keyboard_pause_accepts_uppercase_q():
    read_fd, write_fd = os.pipe()
    keyboard = KeyboardPause()
    keyboard.fd = read_fd
    try:
        os.write(write_fd, b"Q")
        assert keyboard.poll() == "pause"
    finally:
        os.close(read_fd)
        os.close(write_fd)


def test_requested_seven_second_approach_is_separate_from_velocity_ceiling():
    cfg = SimpleNamespace(period_s=0.02, max_velocity=0.1, max_acceleration=0.5)

    assert approach_duration(0.05, cfg, 7.0) == 7.0
    assert approach_duration(1.0, cfg, 7.0) == 20.0


def test_initial_positioning_requires_matching_reviewed_staging_envelope():
    args = SimpleNamespace(
        stage_from_current=True,
        label="calibration-down-inward",
        staging_duration_s=10.0,
        approach_duration_s=7.0,
    )
    cfg = SimpleNamespace(max_displacement=0.4, gravity_urdf="g1_29.urdf")
    contract = {
        "physical_replay_staging": {
            "reviewed": True,
            "reviewer": "Lab reviewer",
            "label": "calibration-down-inward",
            "max_start_delta_rad": 0.4,
            "minimum_duration_s": 10.0,
            "path_and_clearance_reviewed": True,
            "lower_body_mode": "stock_balance",
            "live_waist_gravity_reviewed": True,
        }
    }

    assert validate_staging_contract(contract, args, 0.3634, cfg) == 10.0

    cfg.max_displacement = 0.05
    with pytest.raises(ValueError, match="max_displacement"):
        validate_staging_contract(contract, args, 0.3634, cfg)


def test_stock_balance_staging_requires_live_waist_gravity_model():
    args = SimpleNamespace(
        stage_from_current=True,
        label="calibration-down-inward",
        staging_duration_s=10.0,
        approach_duration_s=7.0,
    )
    cfg = SimpleNamespace(max_displacement=0.4, gravity_urdf=None)
    contract = {
        "physical_replay_staging": {
            "reviewed": True,
            "reviewer": "Lab reviewer",
            "label": "calibration-down-inward",
            "max_start_delta_rad": 0.4,
            "minimum_duration_s": 10.0,
            "path_and_clearance_reviewed": True,
            "lower_body_mode": "stock_balance",
            "live_waist_gravity_reviewed": True,
        }
    }

    with pytest.raises(ValueError, match="live-waist gravity"):
        validate_staging_contract(contract, args, 0.3634, cfg)


def test_normal_05a_keeps_the_standard_approach_without_staging_review():
    args = SimpleNamespace(
        stage_from_current=False,
        approach_duration_s=7.0,
    )

    assert validate_staging_contract({}, args, 0.01, SimpleNamespace()) == 7.0
