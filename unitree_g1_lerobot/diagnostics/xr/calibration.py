"""Plan and quality gates for a repeatable XR controller calibration suite."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from pathlib import Path
from statistics import median

from lerobot_teleoperator_g1_vr import TraceReader
from unitree_g1_lerobot.diagnostics.xr.trace_validation import validate_trace


@dataclass(frozen=True)
class CalibrationSegment:
    """One explicitly instructed static pose or single-arm movement."""

    label: str
    expectation: str
    duration_s: float
    preparation: str
    instruction: str
    phase: str = "motion_validation"
    position_group: str | None = None
    position_anchor: bool = False
    active_side: str | None = None
    minimum_motion_m: float | None = None
    maximum_static_motion_m: float | None = None
    maximum_static_rotation_rad: float | None = None

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def _calibration_capture(
    label: str,
    position_group: str,
    preparation: str,
    instruction: str,
    *,
    position_anchor: bool = False,
) -> CalibrationSegment:
    return CalibrationSegment(
        label=label,
        expectation="idle",
        duration_s=5.0,
        preparation=preparation,
        instruction=instruction,
        phase="calibration",
        position_group=position_group,
        position_anchor=position_anchor,
        maximum_static_motion_m=0.06,
        maximum_static_rotation_rad=0.25,
    )


_FORWARD_PREPARATION = (
    "Extend both arms straight forward at shoulder height as far as is comfortable. "
    "Keep both elbows and wrists straight."
)
_HORIZONTAL_PREPARATION = (
    "Open both arms straight out to the sides at shoulder height to make a horizontal "
    "T shape. Keep both elbows and wrists straight."
)


CALIBRATION_PLAN_VERSION = 4
CALIBRATION_PLAN = (
    _calibration_capture(
        "calibration-down-inward",
        "down",
        "Face straight ahead and keep your head level. Completely relax both arms "
        "and let them hang naturally at your sides.",
        "Keep both palms facing inward, keep your wrists straight, and hold still.",
        position_anchor=True,
    ),
    _calibration_capture(
        "calibration-forward-up",
        "forward",
        _FORWARD_PREPARATION,
        "Turn both palms upward toward the ceiling, and hold still.",
    ),
    _calibration_capture(
        "calibration-forward-inward",
        "forward",
        _FORWARD_PREPARATION,
        "Turn both palms inward toward each other with thumbs upward, and hold still.",
        position_anchor=True,
    ),
    _calibration_capture(
        "calibration-forward-down",
        "forward",
        _FORWARD_PREPARATION,
        "Turn both palms downward toward the floor, and hold still.",
    ),
    _calibration_capture(
        "calibration-forward-outward",
        "forward",
        _FORWARD_PREPARATION,
        "Turn both palms outward away from each other, and hold still.",
    ),
    _calibration_capture(
        "calibration-horizontal-up",
        "horizontal",
        _HORIZONTAL_PREPARATION,
        "Turn both palms upward toward the ceiling, and hold still.",
    ),
    _calibration_capture(
        "calibration-horizontal-forward",
        "horizontal",
        _HORIZONTAL_PREPARATION,
        "Rotate both palms forward with thumbs upward, and hold still.",
    ),
    _calibration_capture(
        "calibration-horizontal-down",
        "horizontal",
        _HORIZONTAL_PREPARATION,
        "Turn both palms down toward the floor with thumbs forward, and hold still.",
        position_anchor=True,
    ),
    _calibration_capture(
        "calibration-horizontal-backward",
        "horizontal",
        _HORIZONTAL_PREPARATION,
        "Rotate both palms backward with thumbs downward, and hold still.",
    ),
    _calibration_capture(
        "calibration-up-inward",
        "up",
        "Raise both arms straight upward over your head as high as is comfortable. "
        "Keep both elbows and wrists straight.",
        "Keep both palms facing inward toward each other, and hold still.",
        position_anchor=True,
    ),
    CalibrationSegment(
        label="right-forward",
        expectation="right",
        duration_s=10.0,
        preparation=(
            "Return to the 90 degree forearm pose. Keep your left arm still and "
            "release the left grip."
        ),
        instruction=(
            "Press and hold only the right grip. Slowly move your right hand "
            "straight forward until your right arm is comfortably fully extended. "
            "Hold for one second, then slowly return to the 90 degree pose."
        ),
        phase="motion_validation",
        active_side="right",
        minimum_motion_m=0.15,
    ),
    CalibrationSegment(
        label="right-up",
        expectation="right",
        duration_s=10.0,
        preparation=(
            "Return to the 90 degree forearm pose. Keep your left arm still and "
            "release the left grip."
        ),
        instruction=(
            "Press and hold only the right grip. Slowly raise your right hand "
            "straight upward as high as is comfortable. Hold for one second, then "
            "slowly return to the 90 degree pose."
        ),
        phase="motion_validation",
        active_side="right",
        minimum_motion_m=0.15,
    ),
    CalibrationSegment(
        label="right-outward",
        expectation="right",
        duration_s=10.0,
        preparation=(
            "Return to the 90 degree forearm pose. Keep your left arm still and "
            "release the left grip."
        ),
        instruction=(
            "Press and hold only the right grip. Slowly move your right hand "
            "directly out to your right side as far as is comfortable. Hold for one "
            "second, then slowly return to the 90 degree pose."
        ),
        phase="motion_validation",
        active_side="right",
        minimum_motion_m=0.15,
    ),
    CalibrationSegment(
        label="left-forward",
        expectation="left",
        duration_s=10.0,
        preparation=(
            "Return to the 90 degree forearm pose. Keep your right arm still and "
            "release the right grip."
        ),
        instruction=(
            "Press and hold only the left grip. Slowly move your left hand straight "
            "forward until your left arm is comfortably fully extended. Hold for one "
            "second, then slowly return to the 90 degree pose."
        ),
        phase="motion_validation",
        active_side="left",
        minimum_motion_m=0.15,
    ),
    CalibrationSegment(
        label="left-up",
        expectation="left",
        duration_s=10.0,
        preparation=(
            "Return to the 90 degree forearm pose. Keep your right arm still and "
            "release the right grip."
        ),
        instruction=(
            "Press and hold only the left grip. Slowly raise your left hand straight "
            "upward as high as is comfortable. Hold for one second, then slowly "
            "return to the 90 degree pose."
        ),
        phase="motion_validation",
        active_side="left",
        minimum_motion_m=0.15,
    ),
    CalibrationSegment(
        label="left-outward",
        expectation="left",
        duration_s=10.0,
        preparation=(
            "Return to the 90 degree forearm pose. Keep your right arm still and "
            "release the right grip."
        ),
        instruction=(
            "Press and hold only the left grip. Slowly move your left hand directly "
            "out to your left side as far as is comfortable. Hold for one second, "
            "then slowly return to the 90 degree pose."
        ),
        phase="motion_validation",
        active_side="left",
        minimum_motion_m=0.15,
    ),
)


def _representative_pose(tracked_poses) -> dict[str, object] | None:
    poses = [tracked.pose for tracked in tracked_poses if tracked.pose is not None]
    if not poses:
        return None
    position = [median(pose.position_m[index] for pose in poses) for index in range(3)]
    reference = poses[0].quaternion_xyzw
    aligned = []
    for pose in poses:
        quaternion = pose.quaternion_xyzw
        if sum(a * b for a, b in zip(reference, quaternion, strict=True)) < 0:
            quaternion = tuple(-value for value in quaternion)
        aligned.append(quaternion)
    quaternion = [sum(value[index] for value in aligned) for index in range(4)]
    norm = math.sqrt(sum(value * value for value in quaternion))
    return {
        "tracked_samples": len(poses),
        "position_m": position,
        "quaternion_xyzw": [value / norm for value in quaternion],
    }


def assess_calibration_trace(
    trace_path: str | Path,
    segment: CalibrationSegment,
) -> dict[str, object]:
    """Apply generic trace checks plus segment-specific calibration quality gates."""

    report = validate_trace(trace_path, expectation=segment.expectation)
    issues = list(report["issues"])
    measurements = report["measurements"]

    if segment.active_side is not None:
        stats = measurements[segment.active_side]
        if (
            segment.minimum_motion_m is not None
            and stats["position_displacement_m"] < segment.minimum_motion_m
        ):
            issues.append(
                f"{segment.active_side} controller translation is below "
                f"{segment.minimum_motion_m:.2f} m"
            )
        engaged_ratio = stats["squeeze_engaged_frames"] / measurements["sample_count"]
        if engaged_ratio < 0.5:
            issues.append(
                f"{segment.active_side} grip was held for less than half of the trace"
            )

    if segment.maximum_static_motion_m is not None:
        for side in ("left", "right"):
            if (
                measurements[side]["position_displacement_m"]
                > segment.maximum_static_motion_m
            ):
                issues.append(
                    f"{side} controller moved more than the "
                    f"{segment.maximum_static_motion_m:.2f} m static-pose limit"
                )

    if segment.maximum_static_rotation_rad is not None:
        for side in ("left", "right"):
            if (
                measurements[side]["rotation_displacement_rad"]
                > segment.maximum_static_rotation_rad
            ):
                issues.append(
                    f"{side} controller rotated more than the "
                    f"{segment.maximum_static_rotation_rad:.2f} rad static-pose limit"
                )

    report["issues"] = issues
    report["passed"] = not issues
    report["calibration_segment"] = segment.to_dict()
    if segment.phase != "motion_validation":
        samples = [frame.sample for frame in TraceReader(trace_path).frames]
        report["representative_poses"] = {
            "head": _representative_pose(sample.head for sample in samples),
            "left": _representative_pose(sample.left for sample in samples),
            "right": _representative_pose(sample.right for sample in samples),
        }
    return report


def _pose_distance(
    reports: dict[str, dict[str, object]],
    first: str,
    second: str,
    side: str,
) -> float:
    first_position = reports[first]["representative_poses"][side]["position_m"]
    second_position = reports[second]["representative_poses"][side]["position_m"]
    return math.dist(first_position, second_position)


def _orientation_distance(
    reports: dict[str, dict[str, object]],
    first: str,
    second: str,
    side: str,
) -> float:
    first_quaternion = reports[first]["representative_poses"][side]["quaternion_xyzw"]
    second_quaternion = reports[second]["representative_poses"][side]["quaternion_xyzw"]
    dot = abs(
        sum(a * b for a, b in zip(first_quaternion, second_quaternion, strict=True))
    )
    return 2.0 * math.acos(min(1.0, dot))


def assess_calibration_unit(
    position_group: str,
    reports: dict[str, dict[str, object]],
) -> dict[str, object]:
    """Validate one position unit, including any palm-orientation subphases."""

    unit_segments = tuple(
        segment
        for segment in CALIBRATION_PLAN
        if segment.position_group == position_group
    )
    if not unit_segments:
        raise ValueError(f"unknown calibration position group: {position_group}")
    issues = [
        f"{segment.label}: {issue}"
        for segment in unit_segments
        for issue in reports[segment.label]["issues"]
    ]
    measurements: dict[str, object] = {}
    if not issues and position_group == "forward":
        values = {
            side: _pose_distance(
                reports,
                "calibration-down-inward",
                "calibration-forward-inward",
                side,
            )
            for side in ("left", "right")
        }
        measurements["down_to_forward_m"] = values
        for side, value in values.items():
            if value < 0.25:
                issues.append(f"{side} down to forward position change is below 0.25 m")
        for label in (
            "calibration-forward-up",
            "calibration-forward-down",
            "calibration-forward-outward",
        ):
            position_change = {
                side: _pose_distance(reports, "calibration-forward-inward", label, side)
                for side in ("left", "right")
            }
            measurements[f"{label}_position_change_m"] = position_change
            for side, value in position_change.items():
                if value > 0.15:
                    issues.append(f"{side} hand moved more than 0.15 m during {label}")
        for name, first, second in (
            (
                "up_to_down",
                "calibration-forward-up",
                "calibration-forward-down",
            ),
            (
                "inward_to_outward",
                "calibration-forward-inward",
                "calibration-forward-outward",
            ),
        ):
            orientation_change = {
                side: _orientation_distance(reports, first, second, side)
                for side in ("left", "right")
            }
            measurements[f"{name}_orientation_change_rad"] = orientation_change
            for side, value in orientation_change.items():
                if value < 1.50:
                    issues.append(
                        f"{side} {name.replace('_', ' ')} wrist change is below 1.50 rad"
                    )
    elif not issues and position_group == "horizontal":
        values = {
            side: _pose_distance(
                reports,
                "calibration-forward-inward",
                "calibration-horizontal-down",
                side,
            )
            for side in ("left", "right")
        }
        measurements["forward_to_horizontal_m"] = values
        for side, value in values.items():
            if value < 0.20:
                issues.append(
                    f"{side} forward to horizontal position change is below 0.20 m"
                )

        for label in (
            "calibration-horizontal-up",
            "calibration-horizontal-forward",
            "calibration-horizontal-backward",
        ):
            position_change = {
                side: _pose_distance(
                    reports, "calibration-horizontal-down", label, side
                )
                for side in ("left", "right")
            }
            measurements[f"{label}_position_change_m"] = position_change
            for side, value in position_change.items():
                if value > 0.15:
                    issues.append(f"{side} hand moved more than 0.15 m during {label}")
        for name, first, second in (
            (
                "up_to_down",
                "calibration-horizontal-up",
                "calibration-horizontal-down",
            ),
            (
                "forward_to_backward",
                "calibration-horizontal-forward",
                "calibration-horizontal-backward",
            ),
        ):
            orientation_change = {
                side: _orientation_distance(reports, first, second, side)
                for side in ("left", "right")
            }
            measurements[f"{name}_orientation_change_rad"] = orientation_change
            for side, value in orientation_change.items():
                if value < 1.50:
                    issues.append(
                        f"{side} {name.replace('_', ' ')} wrist change is below 1.50 rad"
                    )
    elif not issues and position_group == "up":
        values = {
            side: _pose_distance(
                reports,
                "calibration-down-inward",
                "calibration-up-inward",
                side,
            )
            for side in ("left", "right")
        }
        measurements["down_to_up_m"] = values
        for side, value in values.items():
            if value < 0.40:
                issues.append(f"{side} down to up position change is below 0.40 m")

    return {
        "schema_version": 1,
        "plan_version": CALIBRATION_PLAN_VERSION,
        "position_group": position_group,
        "passed": not issues,
        "issues": issues,
        "measurements": measurements,
    }


def pending_segments(
    manifest: dict[str, object] | None,
) -> tuple[CalibrationSegment, ...]:
    """Return plan entries that do not already have a selected passing trace."""

    recorded = {} if manifest is None else manifest.get("segments", {})
    return tuple(
        segment
        for segment in CALIBRATION_PLAN
        if recorded.get(segment.label, {}).get("status") != "passed"
    )
