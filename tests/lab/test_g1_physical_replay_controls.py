import os
from types import SimpleNamespace

import pytest

from tools.g1_physical_replay import (
    KeyboardPause,
    PauseState,
    VoiceAlarm,
    approach_duration,
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
