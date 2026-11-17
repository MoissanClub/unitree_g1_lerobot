"""Opt-in end-to-end trace -> real G1 IK -> native MuJoCo simulation gate."""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config
from lerobot_teleoperator_g1_vr import (
    DeterministicClock,
    G1RetargeterConfig,
    G129Retargeter,
    TraceManifestSeed,
    TraceRecorder,
    XRController,
    XRInteractionGuard,
    XRPose,
    XRSample,
    XRSourceSelection,
    XRSourceState,
    XRSourceStatus,
    build_xr_source,
)
from unitree_g1_lerobot.xr import G1VRSimulationSupervisor

URDF = os.environ.get("G1_29_URDF")
pytestmark = pytest.mark.skipif(not URDF, reason="Set G1_29_URDF to the pinned model")


def controller(*, x=0.0, z=0.0, squeeze=0.0):
    return XRController(True, XRPose((x, 0, z), (0, 0, 0, 1)), squeeze=squeeze)


def test_recorded_trace_drives_native_g1_29_simulation(tmp_path):
    assert Path(URDF).is_file()
    clock = DeterministicClock()
    trace = tmp_path / "g1-29"
    samples = (
        XRSample(0, 0.00, controller(), controller(), XRSourceStatus(XRSourceState.READY)),
        XRSample(
            1,
            0.02,
            controller(),
            controller(squeeze=1.0),
            XRSourceStatus(XRSourceState.READY),
        ),
        XRSample(
            2,
            0.04,
            controller(),
            controller(x=0.02, z=0.02, squeeze=1.0),
            XRSourceStatus(XRSourceState.READY),
        ),
        XRSample(3, 0.06, controller(), controller(), XRSourceStatus(XRSourceState.READY)),
    )
    with TraceRecorder(
        trace, TraceManifestSeed("pinned", "pinned", "synthetic", "test", "test")
    ) as recorder:
        for sample in samples:
            recorder.record(sample)

    source = build_xr_source(XRSourceSelection("trace", trace_path=trace), clock=clock)
    retargeter = G129Retargeter(G1RetargeterConfig(URDF), clock=clock)
    robot = UnitreeG1(
        UnitreeG1Config(
            id="test-g1-29-vr",
            embodiment="g1_29",
            simulation_urdf=URDF,
            gravity_compensation=True,
        )
    )
    records = G1VRSimulationSupervisor(
        source, XRInteractionGuard(clock=clock), retargeter, robot
    ).run()

    assert len(records) == 4
    assert records[0].held
    assert records[1].right_engaged
    assert records[2].right_engaged
    assert records[2].maximum_command_delta_rad > 1e-4
    assert records[3].held
    assert not robot.is_connected
