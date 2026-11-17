#!/usr/bin/env python3
"""Create a deterministic artificial XR trace for headset-free lab-script tests."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from lerobot_teleoperator_g1_vr import (
    TraceManifestSeed,
    TraceRecorder,
    XRController,
    XRPose,
    XRSample,
    XRSourceState,
    XRSourceStatus,
    XRTrackedPose,
)

SCENARIOS = ("idle", "right", "left", "bimanual", "reconnect")


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("output", type=Path)
    result.add_argument("--scenario", choices=SCENARIOS, default="bimanual")
    result.add_argument("--duration-s", type=float, default=4.0)
    result.add_argument("--rate-hz", type=float, default=60.0)
    return result


def controller(side: str, active: bool, tracked: bool, motion: float) -> XRController:
    if not tracked:
        return XRController(False, None)
    lateral = 0.22 if side == "left" else -0.22
    direction = 1.0 if side == "left" else -1.0
    position = (
        0.35 + (motion if active else 0.0),
        lateral,
        1.10 + direction * motion / 2,
    )
    angle = direction * motion if active else 0.0
    pose = XRPose(
        position,
        (0.0, math.sin(angle / 2), 0.0, math.cos(angle / 2)),
    )
    return XRController(True, pose, squeeze=1.0 if active else 0.0)


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    if not math.isfinite(args.duration_s) or args.duration_s <= 0:
        raise SystemExit("--duration-s must be a positive finite number")
    if not math.isfinite(args.rate_hz) or args.rate_hz <= 0:
        raise SystemExit("--rate-hz must be a positive finite number")
    count = max(2, round(args.duration_s * args.rate_hz))
    seed = TraceManifestSeed(
        trace_id=args.output.name,
        label=f"synthetic-{args.scenario}",
        source_type="synthetic",
        source_adapter_revision="deterministic-generator-v1",
        coordinate_frame="artificial normalized controller frame; metres; quaternion xyzw",
    )
    with TraceRecorder(args.output, seed) as recorder:
        for index in range(count):
            progress = index / (count - 1)
            engaged = progress >= 0.15
            left_active = args.scenario in ("left", "bimanual") and engaged
            right_active = args.scenario in ("right", "bimanual") and engaged
            state = XRSourceState.READY
            tracked = True
            timestamp_advanced = True
            if args.scenario == "reconnect" and 0.35 <= progress < 0.55:
                state = XRSourceState.STALE
                tracked = False
                timestamp_advanced = False
            motion = 0.12 * max(0.0, (progress - 0.15) / 0.85)

            head = XRTrackedPose(
                tracked,
                XRPose((0.0, 0.0, 1.65), (0.0, 0.0, 0.0, 1.0)) if tracked else None,
            )
            captured_at_s = 10.0 + index / args.rate_hz
            sample = XRSample(
                sequence=index,
                captured_at_s=captured_at_s,
                left=controller("left", left_active, tracked, motion),
                right=controller("right", right_active, tracked, motion),
                status=XRSourceStatus(state, "artificial test sample"),
                head=head,
                source_timestamp_ns=1_000_000_000 + round(index / args.rate_hz * 1e9),
                source_timestamp_advanced=timestamp_advanced,
            )
            recorder.record(
                sample,
                {"synthetic": True, "scenario": args.scenario, "progress": progress},
            )
    print(f"Synthetic {args.scenario} trace: {args.output} ({count} samples)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
