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
    assess_calibration_unit,
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
    assert "relax both arms" in by_label["calibration-down-inward"].preparation
    assert "straight forward" in by_label["calibration-forward-up"].preparation
    assert "horizontal T shape" in by_label["calibration-horizontal-down"].preparation
    assert "palms forward" in by_label["calibration-horizontal-forward"].instruction
    assert "straight upward" in by_label["calibration-up-inward"].preparation
    calibration = [
        segment for segment in CALIBRATION_PLAN if segment.phase == "calibration"
    ]
    assert len(calibration) == 10
    assert {segment.position_group for segment in calibration} == {
        "down",
        "forward",
        "horizontal",
        "up",
    }
    assert sum(segment.position_anchor for segment in calibration) == 4
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
        phase="calibration",
        maximum_static_motion_m=0.08,
    )

    report = assess_calibration_trace(path, segment)

    assert not report["passed"]
    assert any("static-pose limit" in issue for issue in report["issues"])
    assert report["representative_poses"]["left"]["position_m"] == [0.05, 0.0, 0.0]


def test_resume_skips_only_segments_with_selected_passes():
    manifest = {
        "segments": {
            "calibration-down-inward": {"status": "passed"},
            "calibration-up-inward": {"status": "failed"},
        }
    }

    labels = [segment.label for segment in pending_segments(manifest)]

    assert "calibration-down-inward" not in labels
    assert "calibration-up-inward" in labels
    assert "right-forward" in labels


def test_horizontal_calibration_unit_checks_position_and_wrist_change():
    def report(position, quaternion):
        pose = {"position_m": position, "quaternion_xyzw": quaternion}
        return {
            "passed": True,
            "issues": [],
            "representative_poses": {"left": pose, "right": pose},
        }

    reports = {
        "calibration-forward-inward": report([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        "calibration-horizontal-up": report([0.4, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        "calibration-horizontal-down": report([0.4, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]),
        "calibration-horizontal-forward": report(
            [0.4, 0.0, 0.0], [0.70710678, 0.0, 0.0, 0.70710678]
        ),
        "calibration-horizontal-backward": report(
            [0.4, 0.0, 0.0], [-0.70710678, 0.0, 0.0, 0.70710678]
        ),
    }

    passed = assess_calibration_unit("horizontal", reports)
    reports["calibration-horizontal-backward"] = report(
        [0.4, 0.0, 0.0], [0.70710678, 0.0, 0.0, 0.70710678]
    )
    failed = assess_calibration_unit("horizontal", reports)

    assert passed["passed"]
    assert not failed["passed"]
    assert any("wrist change" in issue for issue in failed["issues"])


def test_forward_calibration_unit_checks_opposed_orientations():
    def report(position, quaternion):
        pose = {"position_m": position, "quaternion_xyzw": quaternion}
        return {
            "passed": True,
            "issues": [],
            "representative_poses": {"left": pose, "right": pose},
        }

    reports = {
        "calibration-down-inward": report([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        "calibration-forward-up": report([0.4, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        "calibration-forward-inward": report([0.4, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        "calibration-forward-down": report([0.4, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]),
        "calibration-forward-outward": report([0.4, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]),
    }

    passed = assess_calibration_unit("forward", reports)
    reports["calibration-forward-outward"] = report(
        [0.4, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]
    )
    failed = assess_calibration_unit("forward", reports)

    assert passed["passed"]
    assert not failed["passed"]
    assert any("inward to outward" in issue for issue in failed["issues"])
