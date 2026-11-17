"""Supervisor tests use fake sources/retargeting/targets and no vendor runtime."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from lerobot_teleoperator_g1_vr import (
    DeterministicClock,
    G1RetargetDiagnostics,
    G1RetargetResult,
    XRController,
    XRInteractionGuard,
    XRPose,
    XRSample,
    XRSourceState,
    XRSourceStatus,
    XRTraceExhaustedError,
)
from unitree_g1_lerobot.xr import G1VRSimulationSupervisor


KEYS = tuple(f"joint_{index}.q" for index in range(14))


class FakeSource:
    def __init__(self, samples, *, connect_error=None):
        self.samples = iter(samples)
        self.connect_error = connect_error
        self.is_connected = False
        self.status = XRSourceStatus(XRSourceState.DISCONNECTED)
        self.disconnect_calls = 0

    def connect(self):
        if self.connect_error:
            raise self.connect_error
        self.is_connected = True

    def sample(self):
        try:
            return next(self.samples)
        except StopIteration as exc:
            raise XRTraceExhaustedError from exc

    def disconnect(self):
        self.disconnect_calls += 1
        self.is_connected = False


class FakeRetargeter:
    embodiment = "g1_29"
    action_features = dict.fromkeys(KEYS, float)

    def reset(self):
        pass

    def retarget(self, sample, measured):
        engaged = (sample.left.squeeze >= 0.5, sample.right.squeeze >= 0.5)
        action = dict(zip(KEYS, measured, strict=True))
        if engaged[0]:
            action[KEYS[0]] += 0.1
        if engaged[1]:
            action[KEYS[7]] -= 0.2
        return G1RetargetResult(
            action,
            G1RetargetDiagnostics("g1_29", sample.status.state, *engaged, not any(engaged)),
        )


class FakeTarget:
    def __init__(self):
        self.is_connected = False
        self.q = [0.0] * 14
        self.actions = []
        self.disconnect_calls = 0

    def connect(self):
        self.is_connected = True

    def get_observation(self):
        return dict(zip(KEYS, self.q, strict=True))

    def send_action(self, action):
        self.actions.append(dict(action))
        self.q = [action[key] for key in KEYS]

    def disconnect(self):
        self.disconnect_calls += 1
        self.is_connected = False


def sample(sequence, *, left=0.0, right=0.0, state=XRSourceState.READY):
    def controller(squeeze):
        return XRController(True, XRPose((0, 0, 0), (0, 0, 0, 1)), squeeze=squeeze)

    return XRSample(
        sequence,
        10.0,
        controller(left),
        controller(right),
        XRSourceStatus(state),
    )


def supervisor(samples, *, source=None, target=None):
    clock = DeterministicClock(10.0)
    return G1VRSimulationSupervisor(
        source or FakeSource(samples),
        XRInteractionGuard(clock=clock),
        FakeRetargeter(),
        target or FakeTarget(),
    )


def test_trace_sequence_drives_both_sides_and_records_comparable_telemetry():
    target = FakeTarget()
    records = supervisor(
        [sample(0), sample(1, left=1.0), sample(2, right=1.0)], target=target
    ).run()
    assert len(records) == 3
    assert records[0].held
    assert records[1].left_engaged and records[1].maximum_command_delta_rad == pytest.approx(0.1)
    assert records[2].right_engaged and records[2].maximum_command_delta_rad == pytest.approx(0.2)
    assert target.disconnect_calls == 1


def test_stale_input_is_held_and_requires_release_before_regrip():
    records = supervisor(
        [
            sample(0, left=1.0, state=XRSourceState.STALE),
            sample(1, left=1.0),
            sample(2, left=0.0),
            sample(3, left=1.0),
        ]
    ).run()
    assert [record.left_engaged for record in records] == [False, False, False, True]
    assert records[0].source_state == "stale"


def test_source_connect_failure_still_disconnects_started_target():
    target = FakeTarget()
    source = FakeSource([], connect_error=RuntimeError("no source"))
    with pytest.raises(RuntimeError, match="no source"):
        supervisor([], source=source, target=target).run()
    assert source.disconnect_calls == 0
    assert target.disconnect_calls == 1


def test_missing_observation_fails_and_cleans_both_lifecycles():
    target = FakeTarget()
    target.get_observation = lambda: {}
    source = FakeSource([sample(0)])
    with pytest.raises(RuntimeError, match="missing finite"):
        supervisor([], source=source, target=target).run()
    assert source.disconnect_calls == 1
    assert target.disconnect_calls == 1


def test_supervisor_rejects_non_g1_29_retargeter():
    retargeter = SimpleNamespace(embodiment="g1_23", action_features={})
    with pytest.raises(ValueError, match="g1_29"):
        G1VRSimulationSupervisor(FakeSource([]), XRInteractionGuard(), retargeter, FakeTarget())
