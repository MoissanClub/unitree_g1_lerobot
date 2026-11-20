"""Regression tests for calibrated motion with bent elbows and tracking interruptions."""
from dataclasses import replace

import numpy as np
from lerobot_teleoperator_g1_vr import XRController, XRPose, XRSample, XRSourceState, XRSourceStatus, XRTrackedPose
from lerobot_teleoperator_g1_vr.calibration import CalibrationProfile


def test_calibrated_sample_maps_axes_scales_and_preserves_real_grips():
    basis = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
    profile = CalibrationProfile(basis, None, (), {"left": (None, 0.5, None), "right": (None, 0.3, None)},
                                 {}, "", "")
    pose = XRPose((2, 3, 1), (0, 0, 0, 1))
    sample = XRSample(1, 1., XRController(True, pose, squeeze=1.),
                      XRController(True, pose), XRSourceStatus(XRSourceState.READY),
                      head=XRTrackedPose(True, XRPose((2, 2, 2), (0, 0, 0, 1))))
    mapped = profile.map_sample(sample)
    np.testing.assert_allclose(mapped.left.pose.position_m, [0.5, 0, 0.5])
    np.testing.assert_allclose(mapped.right.pose.position_m, [0.3, 0, 0.3])
    assert (mapped.left.squeeze, mapped.right.squeeze) == (1., 0.)
    assert profile.heading_matches(sample)
    missing = replace(sample, head=XRTrackedPose(False, None))
    assert not profile.heading_matches(missing)
    assert profile.map_sample(missing).status.state == XRSourceState.STALE
