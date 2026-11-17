"""Human-readable validation and motion statistics for recorded XR traces."""

from __future__ import annotations

import math
from collections import Counter
from pathlib import Path

from lerobot_teleoperator_g1_vr import TraceReader

EXPECTATIONS = ("any", "idle", "right", "left", "bimanual", "buttons-head", "reconnect")


def _maximum_position_displacement(controllers) -> float:
    positions = [
        controller.pose.position_m for controller in controllers if controller.pose
    ]
    if not positions:
        return 0.0
    origin = positions[0]
    return max(math.dist(origin, position) for position in positions)


def _maximum_rotation_radians(controllers) -> float:
    quaternions = [
        controller.pose.quaternion_xyzw for controller in controllers if controller.pose
    ]
    if not quaternions:
        return 0.0
    origin = quaternions[0]
    return max(
        2.0
        * math.acos(
            min(1.0, abs(sum(a * b for a, b in zip(origin, value, strict=True))))
        )
        for value in quaternions
    )


def _side_stats(controllers) -> dict[str, object]:
    count = len(controllers)
    buttons: dict[str, float] = {}
    for controller in controllers:
        for name, value in controller.buttons.items():
            buttons[name] = max(buttons.get(name, 0.0), value)
    return {
        "tracked_frames": sum(controller.tracked for controller in controllers),
        "tracked_ratio": sum(controller.tracked for controller in controllers) / count,
        "squeeze_min": min(controller.squeeze for controller in controllers),
        "squeeze_max": max(controller.squeeze for controller in controllers),
        "squeeze_engaged_frames": sum(
            controller.squeeze >= 0.5 for controller in controllers
        ),
        "trigger_max": max(controller.trigger for controller in controllers),
        "position_displacement_m": _maximum_position_displacement(controllers),
        "rotation_displacement_rad": _maximum_rotation_radians(controllers),
        "button_maxima": buttons,
    }


def validate_trace(
    trace_path: str | Path,
    *,
    expectation: str = "any",
    expected_rate_hz: float = 60.0,
) -> dict[str, object]:
    """Verify trace integrity and return measured acceptance statistics."""

    if expectation not in EXPECTATIONS:
        raise ValueError(f"expectation must be one of {EXPECTATIONS}")
    if not math.isfinite(expected_rate_hz) or expected_rate_hz <= 0:
        raise ValueError("expected_rate_hz must be positive and finite")
    reader = TraceReader(trace_path)
    frames = reader.frames
    samples = [frame.sample for frame in frames]
    duration_s = (frames[-1].elapsed_ns - frames[0].elapsed_ns) / 1_000_000_000
    effective_rate_hz = (len(frames) - 1) / duration_s if duration_s > 0 else 0.0
    states = Counter(sample.status.state.value for sample in samples)
    left = _side_stats([sample.left for sample in samples])
    right = _side_stats([sample.right for sample in samples])
    head_tracked = sum(sample.head.tracked for sample in samples)
    issues: list[str] = []

    if len(frames) < 2 or duration_s <= 0:
        issues.append("trace must contain at least two samples over positive time")
    if effective_rate_hz < expected_rate_hz * 0.5:
        issues.append(
            f"effective sample rate {effective_rate_hz:.1f} Hz is below half of expected"
        )
    if states.get("ready", 0) == 0:
        issues.append("trace contains no ready source sample")
    minimum_tracking_ratio = 0.3 if expectation == "reconnect" else 0.8
    for side, stats in (("left", left), ("right", right)):
        if stats["tracked_ratio"] < minimum_tracking_ratio:
            issues.append(
                f"{side} tracking ratio is below {minimum_tracking_ratio:.0%}"
            )

    motion_threshold_m = 0.02
    rotation_threshold_rad = 0.10

    def moved(stats: dict[str, object]) -> bool:
        return bool(
            stats["position_displacement_m"] >= motion_threshold_m
            or stats["rotation_displacement_rad"] >= rotation_threshold_rad
        )

    if expectation == "idle":
        if left["squeeze_max"] >= 0.5 or right["squeeze_max"] >= 0.5:
            issues.append("idle trace contains an engaged grip")
    elif expectation == "right":
        if right["squeeze_max"] < 0.5:
            issues.append("right grip never engaged")
        if not moved(right):
            issues.append("right controller did not move enough")
        if left["squeeze_max"] >= 0.5:
            issues.append("left grip engaged during right-only trace")
    elif expectation == "left":
        if left["squeeze_max"] < 0.5:
            issues.append("left grip never engaged")
        if not moved(left):
            issues.append("left controller did not move enough")
        if right["squeeze_max"] >= 0.5:
            issues.append("right grip engaged during left-only trace")
    elif expectation == "bimanual":
        for side, stats in (("left", left), ("right", right)):
            if stats["squeeze_max"] < 0.5:
                issues.append(f"{side} grip never engaged")
            if not moved(stats):
                issues.append(f"{side} controller did not move enough")
    elif expectation == "buttons-head":
        if head_tracked / len(samples) < 0.8:
            issues.append("head tracking ratio is below 80%")
        button_or_trigger = max(
            left["trigger_max"],
            right["trigger_max"],
            *left["button_maxima"].values(),
            *right["button_maxima"].values(),
        )
        if button_or_trigger < 0.5:
            issues.append("no trigger or button activation was recorded")
    elif expectation == "reconnect":
        if not ({"stale", "disconnected"} & states.keys()):
            issues.append("reconnect trace has no stale or disconnected state")
        if states.get("ready", 0) == 0:
            issues.append("reconnect trace never returned to ready")

    return {
        "trace_path": str(Path(trace_path).resolve()),
        "expectation": expectation,
        "passed": not issues,
        "issues": issues,
        "manifest": dict(reader.manifest),
        "measurements": {
            "sample_count": len(frames),
            "duration_s": duration_s,
            "effective_rate_hz": effective_rate_hz,
            "sequence_first": samples[0].sequence,
            "sequence_last": samples[-1].sequence,
            "source_states": dict(sorted(states.items())),
            "source_timestamp_advanced_frames": sum(
                sample.source_timestamp_advanced is True for sample in samples
            ),
            "head_tracked_frames": head_tracked,
            "head_tracked_ratio": head_tracked / len(samples),
            "left": left,
            "right": right,
        },
    }
