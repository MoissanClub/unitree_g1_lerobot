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
    active_side: str | None = None
    minimum_motion_m: float | None = None
    maximum_static_motion_m: float | None = None
    maximum_static_rotation_rad: float | None = None

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


CALIBRATION_PLAN_VERSION = 2
CALIBRATION_PLAN = (
    CalibrationSegment(
        label="position-natural-down",
        expectation="idle",
        duration_s=6.0,
        preparation=(
            "Face straight ahead and keep your head level. Hold one controller in "
            "each hand and release both grip buttons."
        ),
        instruction=(
            "Completely relax both of your arms. Let both arms hang naturally at "
            "your sides with palms facing inward. Keep both grips released and hold still."
        ),
        phase="position_calibration",
        maximum_static_motion_m=0.06,
        maximum_static_rotation_rad=0.25,
    ),
    CalibrationSegment(
        label="position-full-up",
        expectation="idle",
        duration_s=6.0,
        preparation=(
            "Raise both arms straight upward over your head as high as is comfortable. "
            "Keep both elbows straight and palms facing inward."
        ),
        instruction=(
            "Hold both arms fully upward. Look straight ahead, keep both grips "
            "released, and hold still."
        ),
        phase="position_calibration",
        maximum_static_motion_m=0.06,
        maximum_static_rotation_rad=0.25,
    ),
    CalibrationSegment(
        label="position-full-forward",
        expectation="idle",
        duration_s=6.0,
        preparation=(
            "Extend both arms straight forward at shoulder height as far as is "
            "comfortable. Keep elbows straight, thumbs upward, and palms facing inward."
        ),
        instruction=(
            "Hold both arms fully forward at shoulder height. Look straight ahead, "
            "keep both grips released, and hold still."
        ),
        phase="position_calibration",
        maximum_static_motion_m=0.06,
        maximum_static_rotation_rad=0.25,
    ),
    CalibrationSegment(
        label="position-full-open",
        expectation="idle",
        duration_s=6.0,
        preparation=(
            "Open both arms straight out to the sides at shoulder height, making a "
            "horizontal T shape. Keep elbows straight and palms facing down."
        ),
        instruction=(
            "Hold the full horizontal open T pose. Look straight ahead, keep both "
            "grips released, and hold still."
        ),
        phase="position_calibration",
        maximum_static_motion_m=0.06,
        maximum_static_rotation_rad=0.25,
    ),
    CalibrationSegment(
        label="palms-up",
        expectation="idle",
        duration_s=5.0,
        preparation=(
            "Bring both hands in front of your chest about shoulder width apart. "
            "Bend your elbows and keep both hands at the same height."
        ),
        instruction=(
            "Turn both palms to face the ceiling. Keep the hand positions fixed, "
            "release both grips, and hold still."
        ),
        phase="orientation_calibration",
        maximum_static_motion_m=0.06,
        maximum_static_rotation_rad=0.25,
    ),
    CalibrationSegment(
        label="palms-down",
        expectation="idle",
        duration_s=5.0,
        preparation=(
            "Keep both hands in front of your chest about shoulder width apart and "
            "at the same height."
        ),
        instruction=(
            "Turn both palms to face the floor. Keep the hand positions fixed, "
            "release both grips, and hold still."
        ),
        phase="orientation_calibration",
        maximum_static_motion_m=0.06,
        maximum_static_rotation_rad=0.25,
    ),
    CalibrationSegment(
        label="palms-inward",
        expectation="idle",
        duration_s=5.0,
        preparation=(
            "Keep both hands in front of your chest about shoulder width apart and "
            "at the same height."
        ),
        instruction=(
            "Turn both palms inward so they face each other. Keep the hand positions "
            "fixed, release both grips, and hold still."
        ),
        phase="orientation_calibration",
        maximum_static_motion_m=0.06,
        maximum_static_rotation_rad=0.25,
    ),
    CalibrationSegment(
        label="palms-outward",
        expectation="idle",
        duration_s=5.0,
        preparation=(
            "Keep both hands in front of your chest about shoulder width apart and "
            "at the same height."
        ),
        instruction=(
            "Turn both palms outward, away from each other. Keep the hand positions "
            "fixed, release both grips, and hold still."
        ),
        phase="orientation_calibration",
        maximum_static_motion_m=0.06,
        maximum_static_rotation_rad=0.25,
    ),
    CalibrationSegment(
        label="palms-forward",
        expectation="idle",
        duration_s=5.0,
        preparation=(
            "Keep both hands in front of your chest about shoulder width apart and "
            "at the same height."
        ),
        instruction=(
            "Turn both palms forward, away from your chest. Keep the hand positions "
            "fixed, release both grips, and hold still."
        ),
        phase="orientation_calibration",
        maximum_static_motion_m=0.06,
        maximum_static_rotation_rad=0.25,
    ),
    CalibrationSegment(
        label="palms-backward",
        expectation="idle",
        duration_s=5.0,
        preparation=(
            "Keep both hands in front of your chest about shoulder width apart and "
            "at the same height."
        ),
        instruction=(
            "Turn both palms backward, toward your chest. Keep the hand positions "
            "fixed, release both grips, and hold still."
        ),
        phase="orientation_calibration",
        maximum_static_motion_m=0.06,
        maximum_static_rotation_rad=0.25,
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
