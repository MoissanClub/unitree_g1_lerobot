"""Deterministic joint-target tests shared by both motor comparisons."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class MotorTest:
    name: str
    seconds: float
    payload_kg: float = 0.0
    delay_s: float = 0.0
    kind: str = "legacy"
    axis: str = "shoulder_pitch"
    amplitude_rad: float = 0.0
    frequency_hz: float = 0.0
    move_seconds: float = 0.0


SUITE = (
    MotorTest("pose_hold", 4),
    MotorTest("shoulder_pitch_step", 6),
    MotorTest("shoulder_roll_step", 6),
    MotorTest("raise_lower", 6),
    MotorTest("forward_back", 6),
    MotorTest("left_right", 6),
    MotorTest("wrist_rotation", 6),
    MotorTest("stop_and_hold", 6),
    MotorTest("payload_hold", 4, payload_kg=0.5),
    MotorTest("delayed_sweep", 6, delay_s=0.020),
) + tuple(
    MotorTest(f"{axis}_{degrees}deg_step", 5, kind="step", axis=f"shoulder_{axis}",
              amplitude_rad=float(np.deg2rad(degrees)))
    for axis in ("pitch", "roll") for degrees in (5, 10, 20)
) + tuple(
    MotorTest(f"{axis}_{hz:g}hz_sweep", 8, kind="fast_sweep", axis=f"shoulder_{axis}",
              amplitude_rad=float(np.deg2rad(10)), frequency_hz=hz)
    for axis in ("pitch", "roll") for hz in (0.5, 1.0, 2.0)
) + tuple(
    MotorTest(f"{axis}_{duration:g}s_move", 5, kind="fast_move", axis=f"shoulder_{axis}",
              amplitude_rad=float(np.deg2rad(20)), move_seconds=duration)
    for axis in ("pitch", "roll") for duration in (0.8, 0.4, 0.2)
) + tuple(
    MotorTest(f"{axis}_rapid_reversal", 5, kind="reversal", axis=f"shoulder_{axis}",
              amplitude_rad=float(np.deg2rad(20)), move_seconds=0.2)
    for axis in ("pitch", "roll")
) + tuple(
    MotorTest(f"extended_hold_{kg:g}kg", 5, payload_kg=kg, kind="extended_hold")
    for kg in (0.0, 0.25, 0.5, 1.0)
)


def smooth_move(t: float, duration: float) -> float:
    u = np.clip(t / duration, 0.0, 1.0)
    return float(u ** 3 * (10 - 15 * u + 6 * u ** 2))


def ready(joints: list[str]) -> np.ndarray:
    values = []
    for joint in joints:
        if "shoulder_pitch" in joint:
            value = -0.35
        elif "shoulder_roll" in joint:
            value = 0.12 if joint.startswith("left") else -0.12
        elif "elbow" in joint:
            value = 0.8
        else:
            value = 0.0
        values.append(value)
    return np.array(values)


def target(test: MotorTest, t: float, joints: list[str]) -> np.ndarray:
    q = ready(joints)
    t = max(0.0, t)
    envelope = min(1.0, t) * min(1.0, max(0.0, test.seconds - t))
    wave = envelope * np.sin(2 * np.pi * 0.35 * t)
    if test.kind != "legacy":
        if test.kind == "extended_hold":
            for i, joint in enumerate(joints):
                if "shoulder_pitch" in joint:
                    q[i] = -1.2
                elif "shoulder_roll" in joint:
                    q[i] = 0.05 if joint.startswith("left") else -0.05
                elif "elbow" in joint:
                    q[i] = 1.0
                elif "shoulder_yaw" in joint:
                    q[i] = 0.6 if joint.startswith("left") else -0.6
            return q
        if test.kind == "step":
            displacement = float(t >= 1)
        elif test.kind == "fast_sweep":
            displacement = envelope * np.sin(2 * np.pi * test.frequency_hz * t)
        else:
            displacement = smooth_move(t - 1, test.move_seconds)
            if test.kind == "reversal":
                displacement -= smooth_move(t - 1 - test.move_seconds, test.move_seconds)
        for i, joint in enumerate(joints):
            if test.axis in joint:
                direction = -1 if test.axis == "shoulder_pitch" or joint.startswith("right") else 1
                q[i] += direction * test.amplitude_rad * displacement
        return q
    for i, joint in enumerate(joints):
        side = 1 if joint.startswith("left") else -1
        if test.name == "shoulder_pitch_step" and "shoulder_pitch" in joint:
            q[i] += -0.20 if t >= 1 else 0
        elif test.name == "shoulder_roll_step" and "shoulder_roll" in joint:
            q[i] += side * 0.18 if t >= 1 else 0
        elif test.name in {"raise_lower", "delayed_sweep"} and "shoulder_pitch" in joint:
            q[i] += 0.35 * wave
        elif test.name == "forward_back" and "elbow" in joint:
            q[i] += 0.40 * wave
        elif test.name == "left_right" and "shoulder_roll" in joint:
            q[i] += side * 0.22 * wave
        elif test.name == "wrist_rotation" and "wrist_roll" in joint:
            q[i] += side * 0.65 * wave
        elif test.name == "stop_and_hold" and "shoulder_pitch" in joint:
            q[i] += 0.30 * np.sin(np.pi * min(t, 2.0))
    return q
