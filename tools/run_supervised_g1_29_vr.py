#!/usr/bin/env python3
"""Run normalized XR input against LeRobot's local native G1-29 simulation."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config
from lerobot_teleoperator_g1_vr import (
    CloudXRSourceConfig,
    DeterministicClock,
    G1RetargeterConfig,
    G129Retargeter,
    ReplayMode,
    SystemClock,
    XRInteractionGuard,
    XRSourceKind,
    XRSourceSelection,
    XRoboToolkitSourceConfig,
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
    result.add_argument("--source", choices=[kind.value for kind in XRSourceKind], required=True)
    result.add_argument("--trace", type=Path)
    result.add_argument("--urdf", type=Path, required=True)
    result.add_argument("--mesh-dir", type=Path)
    result.add_argument("--sdk-site-packages", type=Path)
    result.add_argument("--steps", type=int)
    result.add_argument("--telemetry", type=Path)
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


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    kind = XRSourceKind(args.source)
    if (kind is XRSourceKind.TRACE) != (args.trace is not None):
        raise SystemExit("--trace is required only when --source=trace")
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
    try:
        records = G1VRSimulationSupervisor(source, guard, retargeter, robot).run(
            max_steps=args.steps
        )
    finally:
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
