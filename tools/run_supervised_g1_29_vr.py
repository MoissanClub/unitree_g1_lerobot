#!/usr/bin/env python3
"""Run normalized XR input against LeRobot's local native G1-29 simulation."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

from lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config
from lerobot_teleoperator_g1_vr import (
    CloudXRSourceConfig,
    DeterministicClock,
    G1RetargeterConfig,
    G129Retargeter,
    ReplayMode,
    SystemClock,
    XRInteractionGuard,
    XRoboToolkitSourceConfig,
    XRSourceKind,
    XRSourceSelection,
    build_xr_source,
)

from unitree_g1_lerobot.xr import (
    G1VRSimulationSupervisor,
    QuestVideoSidecar,
    QuestVideoSidecarConfig,
    telemetry_dict,
)


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument(
        "--source", choices=[kind.value for kind in XRSourceKind], required=True
    )
    result.add_argument("--trace", type=Path)
    result.add_argument("--urdf", type=Path, required=True)
    result.add_argument("--mesh-dir", type=Path)
    result.add_argument("--sdk-site-packages", type=Path)
    result.add_argument("--steps", type=int)
    result.add_argument("--duration-s", type=float)
    result.add_argument("--rate-hz", type=float)
    result.add_argument("--telemetry", type=Path)
    result.add_argument("--viewer", action="store_true")
    result.add_argument("--ready-file", type=Path)
    result.add_argument("--start-gate-file", type=Path)
    result.add_argument("--gate-timeout-s", type=float, default=600.0)
    result.add_argument("--quest-video", action="store_true")
    result.add_argument(
        "--video-sender",
        type=Path,
        default=Path(__file__).with_name("quest_video_sender.py"),
    )
    result.add_argument("--video-evidence-root", type=Path, default=Path("outputs"))
    result.add_argument("--video-listen", default="0.0.0.0:13579")
    result.add_argument("--video-display", default=":1")
    return result


class ViewerSimulationTarget:
    """Display the exact native simulation controlled by the supervisor."""

    def __init__(self, target: UnitreeG1) -> None:
        self.target = target
        self.viewer: Any | None = None

    @property
    def is_connected(self) -> bool:
        return self.target.is_connected

    def connect(self) -> None:
        self.target.connect()
        try:
            import mujoco.viewer

            native = self.target._native
            if native is None:
                raise RuntimeError("native simulation was not initialized")
            self.viewer = mujoco.viewer.launch_passive(native.model, native.data)
            self.viewer.cam.lookat[:] = [0.0, 0.0, -0.1]
            self.viewer.cam.distance = 2.2
            self.viewer.cam.azimuth = -135
            self.viewer.cam.elevation = -10
            self.viewer.sync()
        except BaseException:
            self.target.disconnect()
            raise

    def get_observation(self):
        return self.target.get_observation()

    def send_action(self, action):
        result = self.target.send_action(action)
        if self.viewer is not None and self.viewer.is_running():
            self.viewer.sync()
        return result

    def disconnect(self) -> None:
        try:
            if self.viewer is not None:
                self.viewer.close()
                self.viewer = None
        finally:
            self.target.disconnect()


def _ready_gate_hook(ready_file: Path, start_gate_file: Path, timeout_s: float):
    """Return a hook that announces desktop readiness and waits for the operator gate."""

    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise SystemExit("--gate-timeout-s must be a positive finite number")

    def wait_for_gate() -> None:
        ready_file.parent.mkdir(parents=True, exist_ok=True)
        if ready_file.exists() or start_gate_file.exists():
            raise RuntimeError("ready/start gate path already exists")
        temporary = ready_file.with_suffix(ready_file.suffix + ".tmp")
        temporary.write_text(
            json.dumps({"ready_at_monotonic_s": time.monotonic()}, sort_keys=True)
            + "\n"
        )
        temporary.replace(ready_file)
        deadline = time.monotonic() + timeout_s
        while not start_gate_file.is_file():
            if time.monotonic() >= deadline:
                raise TimeoutError("operator start gate timed out")
            time.sleep(0.05)

    return wait_for_gate


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    kind = XRSourceKind(args.source)
    if (kind is XRSourceKind.TRACE) != (args.trace is not None):
        raise SystemExit("--trace is required only when --source=trace")
    if (args.ready_file is None) != (args.start_gate_file is None):
        raise SystemExit("--ready-file and --start-gate-file must be supplied together")
    if args.duration_s is not None and (
        not math.isfinite(args.duration_s) or args.duration_s <= 0
    ):
        raise SystemExit("--duration-s must be a positive finite number")
    if args.rate_hz is not None and (
        not math.isfinite(args.rate_hz) or args.rate_hz <= 0
    ):
        raise SystemExit("--rate-hz must be a positive finite number")
    if args.viewer and args.mesh_dir is None:
        raise SystemExit("--viewer requires --mesh-dir")
    clock = DeterministicClock() if kind is XRSourceKind.TRACE else SystemClock()
    selection = XRSourceSelection(
        kind,
        cloudxr=CloudXRSourceConfig() if kind is XRSourceKind.CLOUDXR else None,
        xrobotoolkit=(
            XRoboToolkitSourceConfig(sdk_site_packages=args.sdk_site_packages)
            if kind is XRSourceKind.XROBOTOOLKIT
            else None
        ),
        trace_path=args.trace,
        replay_mode=ReplayMode.DETERMINISTIC,
    )
    source = build_xr_source(selection, clock=clock)
    retargeter = G129Retargeter(G1RetargeterConfig(args.urdf), clock=clock)
    guard = XRInteractionGuard(clock=clock)
    robot = UnitreeG1(
        UnitreeG1Config(
            id="g1-29-vr-simulation",
            embodiment="g1_29",
            is_simulation=True,
            simulation_urdf=str(args.urdf),
            simulation_mesh_dir=str(args.mesh_dir) if args.mesh_dir else None,
            gravity_compensation=True,
        )
    )
    target = ViewerSimulationTarget(robot) if args.viewer else robot
    sidecar = None
    if args.quest_video:
        host, separator, port = args.video_listen.rpartition(":")
        if not separator:
            raise SystemExit("--video-listen must be HOST:PORT")
        sidecar = QuestVideoSidecar(
            QuestVideoSidecarConfig(
                sender_script=args.video_sender,
                evidence_root=args.video_evidence_root,
                listen_host=host,
                listen_port=int(port),
                display=args.video_display,
            )
        )
        status = sidecar.start()
        print(f"Quest video: {status.state}: {status.detail}")
    telemetry_stream = None
    if args.telemetry:
        args.telemetry.parent.mkdir(parents=True, exist_ok=True)
        telemetry_stream = args.telemetry.with_suffix(".jsonl").open("x")

    def record_hook(record) -> None:
        if telemetry_stream is not None:
            telemetry_stream.write(
                json.dumps(telemetry_dict(record), sort_keys=True) + "\n"
            )
            telemetry_stream.flush()

    try:
        ready_hook = (
            _ready_gate_hook(args.ready_file, args.start_gate_file, args.gate_timeout_s)
            if args.ready_file is not None and args.start_gate_file is not None
            else None
        )
        records = G1VRSimulationSupervisor(source, guard, retargeter, target).run(
            max_steps=args.steps,
            max_duration_s=args.duration_s,
            rate_hz=args.rate_hz,
            ready_hook=ready_hook,
            record_hook=record_hook,
        )
    finally:
        if telemetry_stream is not None:
            telemetry_stream.close()
        if sidecar is not None:
            status = sidecar.stop()
            print(f"Quest video: {status.state}: {status.detail}")
    payload = [telemetry_dict(record) for record in records]
    text = json.dumps(payload, indent=2) + "\n"
    if args.telemetry:
        args.telemetry.write_text(text)
    else:
        print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
