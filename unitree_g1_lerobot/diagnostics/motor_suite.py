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
)


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
