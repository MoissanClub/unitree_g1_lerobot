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


def controller(x, *, squeeze=0.0):
    return XRController(
        True,
        XRPose((x, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        squeeze=squeeze,
    )


def sample(sequence, captured_at_s, *, right_x=0.0, right_squeeze=0.0):
    return XRSample(
        sequence,
        captured_at_s,
        controller(0.0),
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
