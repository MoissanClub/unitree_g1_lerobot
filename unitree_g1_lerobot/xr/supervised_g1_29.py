"""Source-neutral, simulation-only supervised G1-29 VR control loop."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import asdict, dataclass
from typing import Protocol

from lerobot_teleoperator_g1_vr import (
    G1Retargeter,
    XRInputSource,
    XRInteractionGuard,
    XRTraceExhaustedError,
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


class G1VRSimulationSupervisor:
    """Own source/target cleanup and apply guard -> retarget -> simulated robot."""

    def __init__(
        self,
        source: XRInputSource,
        guard: XRInteractionGuard,
        retargeter: G1Retargeter,
        target: G1SimulationTarget,
    ) -> None:
        if retargeter.embodiment != "g1_29":
            raise ValueError("supervisor supports the g1_29 simulation embodiment only")
        if len(retargeter.action_features) != 14:
            raise ValueError("G1-29 retargeter must expose 14 arm actions")
        self.source = source
        self.guard = guard
        self.retargeter = retargeter
        self.target = target

    def _measured_arm_q(self) -> tuple[float, ...]:
        observation = self.target.get_observation()
        try:
            values = tuple(float(observation[key]) for key in self.retargeter.action_features)
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError("simulation observation is missing finite G1-29 arm positions") from exc
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError("simulation observation contains nonfinite G1-29 arm positions")
        return values

    def run(self, *, max_steps: int | None = None) -> tuple[G1VRTelemetry, ...]:
        if max_steps is not None and (isinstance(max_steps, bool) or max_steps <= 0):
            raise ValueError("max_steps must be a positive integer or None")
        records: list[G1VRTelemetry] = []
        target_started = source_started = False
        self.guard.reset()
        self.retargeter.reset()
        try:
            self.target.connect()
            target_started = True
            self.source.connect()
            source_started = True
            while max_steps is None or len(records) < max_steps:
                try:
                    sample = self.source.sample()
                except XRTraceExhaustedError:
                    break
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
                records.append(
                    G1VRTelemetry(
                        sequence=sample.sequence,
                        source_state=guarded.status.state.value,
                        left_engaged=diagnostics.left_engaged,
                        right_engaged=diagnostics.right_engaged,
                        held=diagnostics.held,
                        maximum_command_delta_rad=maximum_delta,
                        ik_status=diagnostics.ik_status,
                    )
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
