"""Source-neutral, simulation-only supervised G1-29 VR control loop."""

from __future__ import annotations

import math
import time
from collections.abc import Callable, Mapping
from dataclasses import asdict, dataclass, replace
from typing import Protocol

from lerobot_teleoperator_g1_vr import (
    G1Retargeter,
    XRInputSource,
    XRInteractionGuard,
    XRTraceExhaustedError, XRSourceState, XRSourceStatus,
)


class G1SimulationTarget(Protocol):
    """Narrow LeRobot robot lifecycle used by the supervisor."""

    @property
    def is_connected(self) -> bool: ...

    def connect(self) -> None: ...

    def get_observation(self) -> Mapping[str, object]: ...

    def send_action(self, action: Mapping[str, float]) -> object: ...

    def disconnect(self) -> None: ...


@dataclass(frozen=True, slots=True)
class G1VRTelemetry:
    sequence: int
    source_state: str
    left_engaged: bool
    right_engaged: bool
    held: bool
    maximum_command_delta_rad: float
    ik_status: str | None
    seed_clipped: bool
    maximum_seed_limit_violation_rad: float
    action: dict
    measured: tuple
    targets: list | None
    captured_at_s: float


class G1VRSimulationSupervisor:
    """Own source/target cleanup and apply guard -> retarget -> simulated robot."""

    def __init__(
        self,
        source: XRInputSource,
        guard: XRInteractionGuard,
        retargeter: G1Retargeter,
        target: G1SimulationTarget,
        profile=None,
        check_heading=False,
    ) -> None:
        if retargeter.embodiment != "g1_29":
            raise ValueError("supervisor supports the g1_29 simulation embodiment only")
        if len(retargeter.action_features) != 14:
            raise ValueError("G1-29 retargeter must expose 14 arm actions")
        self.source = source
        self.guard = guard
        self.retargeter = retargeter
        self.target = target
        self.profile = profile
        self.check_heading = check_heading

    def _measured_arm_q(self) -> tuple[float, ...]:
        observation = self.target.get_observation()
        try:
            values = tuple(
                float(observation[key]) for key in self.retargeter.action_features
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(
                "simulation observation is missing finite G1-29 arm positions"
            ) from exc
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError(
                "simulation observation contains nonfinite G1-29 arm positions"
            )
        return values

    def run(
        self,
        *,
        max_steps: int | None = None,
        max_duration_s: float | None = None,
        rate_hz: float | None = None,
        ready_hook: Callable[[], None] | None = None,
        record_hook: Callable[[G1VRTelemetry], None] | None = None,
    ) -> tuple[G1VRTelemetry, ...]:
        if max_steps is not None and (isinstance(max_steps, bool) or max_steps <= 0):
            raise ValueError("max_steps must be a positive integer or None")
        if max_duration_s is not None and (
            isinstance(max_duration_s, bool)
            or not math.isfinite(max_duration_s)
            or max_duration_s <= 0
        ):
            raise ValueError("max_duration_s must be a positive finite number or None")
        if rate_hz is not None and (
            isinstance(rate_hz, bool) or not math.isfinite(rate_hz) or rate_hz <= 0
        ):
            raise ValueError("rate_hz must be a positive finite number or None")
        heading_ready = not self.check_heading
        records: list[G1VRTelemetry] = []
        target_started = source_started = False
        self.guard.reset()
        self.retargeter.reset()
        try:
            self.target.connect()
            target_started = True
            self.source.connect()
            source_started = True
            if ready_hook is not None:
                ready_hook()
            started_at = time.monotonic()
            while (max_steps is None or len(records) < max_steps) and (
                max_duration_s is None or time.monotonic() - started_at < max_duration_s
            ):
                cycle_started_at = time.monotonic()
                try:
                    sample = self.source.sample()
                except XRTraceExhaustedError:
                    break
                if self.profile is not None:
                    if self.check_heading and (not sample.head.tracked or sample.status.state != XRSourceState.READY):
                        heading_ready = False
                    if not heading_ready:
                        heading_ready = (sample.status.state == XRSourceState.READY
                                         and self.profile.heading_matches(sample)
                                         and max(sample.left.squeeze, sample.right.squeeze) < 0.5)
                        if not heading_ready:
                            sample = replace(sample, status=XRSourceStatus(
                                XRSourceState.STALE, "Face calibration-forward and release both grips"))
                    sample = self.profile.map_sample(sample)
                guarded = self.guard.process(sample)
                measured = self._measured_arm_q()
                result = self.retargeter.retarget(guarded, measured)
                action = dict(result.action)
                maximum_delta = max(
                    abs(action[key] - measured[index])
                    for index, key in enumerate(self.retargeter.action_features)
                )
                self.target.send_action(action)
                diagnostics = result.diagnostics
                record = G1VRTelemetry(
                    sequence=sample.sequence,
                    action=action, measured=measured, captured_at_s=sample.captured_at_s,
                    targets=([t.tolist() for t in self.retargeter.control.targets]
                             if self.profile is not None else None),
                    source_state=guarded.status.state.value,
                    left_engaged=diagnostics.left_engaged,
                    right_engaged=diagnostics.right_engaged,
                    held=diagnostics.held,
                    maximum_command_delta_rad=maximum_delta,
                    ik_status=diagnostics.ik_status,
                    seed_clipped=getattr(diagnostics, "seed_clipped", False),
                    maximum_seed_limit_violation_rad=getattr(
                        diagnostics, "maximum_seed_limit_violation_rad", 0.0
                    ),
                )
                records.append(record)
                if record_hook is not None:
                    record_hook(record)
                if rate_hz is not None:
                    time.sleep(
                        max(0.0, 1.0 / rate_hz - (time.monotonic() - cycle_started_at))
                    )
        finally:
            if source_started:
                self.source.disconnect()
            if target_started:
                self.target.disconnect()
        return tuple(records)


def telemetry_dict(record: G1VRTelemetry) -> dict[str, object]:
    """Return stable JSON-ready telemetry for live/trace comparisons."""

    return asdict(record)
