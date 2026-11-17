from __future__ import annotations

from lerobot_teleoperator_g1_vr import (
    TraceManifestSeed,
    TraceRecorder,
    XRController,
    XRPose,
    XRSample,
    XRSourceState,
    XRSourceStatus,
)

from unitree_g1_lerobot.diagnostics.xr.trace_validation import validate_trace
from unitree_g1_lerobot.diagnostics.xr.calibration import (
    CALIBRATION_PLAN,
    CalibrationSegment,
    assess_calibration_trace,
    pending_segments,
)


def controller(x, *, squeeze=0.0):
    return XRController(
        True,
        XRPose((x, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        squeeze=squeeze,
    )


def sample(
    sequence,
    captured_at_s,
    *,
    left_x=0.0,
    right_x=0.0,
    left_squeeze=0.0,
    right_squeeze=0.0,
):
    return XRSample(
        sequence,
        captured_at_s,
        controller(left_x, squeeze=left_squeeze),
        controller(right_x, squeeze=right_squeeze),
        XRSourceStatus(XRSourceState.READY),
        source_timestamp_ns=sequence + 1,
        source_timestamp_advanced=True,
    )


def make_right_trace(path):
    seed = TraceManifestSeed("right", "right", "test", "revision", "test frame")
    with TraceRecorder(path, seed) as recorder:
        recorder.record(sample(0, 10.00), {})
        recorder.record(sample(1, 10.02, right_x=0.01, right_squeeze=1.0), {})
        recorder.record(sample(2, 10.04, right_x=0.04, right_squeeze=1.0), {})


def test_right_trace_validation_reports_motion_and_grip(tmp_path):
    path = tmp_path / "right"
    make_right_trace(path)

    report = validate_trace(path, expectation="right")

    assert report["passed"]
    measurements = report["measurements"]
    assert measurements["sample_count"] == 3
    assert measurements["right"]["squeeze_max"] == 1.0
    assert measurements["right"]["position_displacement_m"] == 0.04


def test_behavior_expectation_failure_is_reported_without_hiding_integrity(tmp_path):
    path = tmp_path / "right"
    make_right_trace(path)

    report = validate_trace(path, expectation="idle")

    assert not report["passed"]
    assert "idle trace contains an engaged grip" in report["issues"]


def test_calibration_plan_has_explicit_static_and_axis_prompts():
    by_label = {segment.label: segment for segment in CALIBRATION_PLAN}

    assert len(by_label) == len(CALIBRATION_PLAN) == 16
    assert (
        "Completely relax both of your arms"
        in by_label["position-natural-down"].instruction
    )
    assert "fully upward" in by_label["position-full-up"].instruction
    assert "fully forward" in by_label["position-full-forward"].instruction
    assert "horizontal open T pose" in by_label["position-full-open"].instruction
    assert "face the ceiling" in by_label["palms-up"].instruction
    assert "face the floor" in by_label["palms-down"].instruction
    assert "face each other" in by_label["palms-inward"].instruction
    assert "away from each other" in by_label["palms-outward"].instruction
    assert "away from your chest" in by_label["palms-forward"].instruction
    assert "toward your chest" in by_label["palms-backward"].instruction
    assert "straight forward" in by_label["right-forward"].instruction
    assert "straight upward" in by_label["right-up"].instruction
    assert "directly out to your right side" in by_label["right-outward"].instruction
    for segment in CALIBRATION_PLAN:
        if segment.phase != "motion_validation":
            assert "grip" not in segment.preparation.lower()
            assert "grip" not in segment.instruction.lower()


def test_calibration_motion_requires_translation_and_sustained_grip(tmp_path):
    path = tmp_path / "right-calibration"
    seed = TraceManifestSeed("right", "right", "test", "revision", "test frame")
    with TraceRecorder(path, seed) as recorder:
        recorder.record(sample(0, 10.00), {})
        recorder.record(sample(1, 10.02, right_x=0.10, right_squeeze=1.0), {})
        recorder.record(sample(2, 10.04, right_x=0.20, right_squeeze=1.0), {})
    segment = CalibrationSegment(
        "right-forward",
        "right",
        10.0,
        "prepare",
        "move",
        active_side="right",
        minimum_motion_m=0.15,
    )

    report = assess_calibration_trace(path, segment)

    assert report["passed"]


def test_calibration_static_pose_rejects_excess_motion(tmp_path):
    path = tmp_path / "static"
    seed = TraceManifestSeed("static", "static", "test", "revision", "test frame")
    with TraceRecorder(path, seed) as recorder:
        recorder.record(sample(0, 10.00), {})
        recorder.record(sample(1, 10.02, left_x=0.05), {})
        recorder.record(sample(2, 10.04, left_x=0.10), {})
    segment = CalibrationSegment(
        "relaxed-arms",
        "idle",
        8.0,
        "prepare",
        "relax",
        phase="position_calibration",
        maximum_static_motion_m=0.08,
    )

    report = assess_calibration_trace(path, segment)

    assert not report["passed"]
    assert any("static-pose limit" in issue for issue in report["issues"])
    assert report["representative_poses"]["left"]["position_m"] == [0.05, 0.0, 0.0]


def test_resume_skips_only_segments_with_selected_passes():
    manifest = {
        "segments": {
            "position-natural-down": {"status": "passed"},
            "position-full-up": {"status": "failed"},
        }
    }

    labels = [segment.label for segment in pending_segments(manifest)]

    assert "position-natural-down" not in labels
    assert "position-full-up" in labels
    assert "right-forward" in labels
