#!/usr/bin/env python3
"""Record an XRoboToolkit trace after a desktop-ready/operator-start handshake."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

from lerobot_teleoperator_g1_vr import (
    TraceManifestSeed,
    TraceRecorder,
    XRoboToolkitSource,
    XRoboToolkitSourceConfig,
)


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("output", type=Path)
    result.add_argument("--label", required=True)
    result.add_argument("--duration-s", type=float, default=10.0)
    result.add_argument("--rate-hz", type=float, default=60.0)
    result.add_argument("--sdk-site-packages", type=Path)
    result.add_argument("--adapter-revision", default="working-tree")
    result.add_argument("--ready-file", type=Path, required=True)
    result.add_argument("--start-gate-file", type=Path, required=True)
    result.add_argument("--gate-timeout-s", type=float, default=600.0)
    return result


def _validate_positive(value: float, name: str) -> None:
    if not math.isfinite(value) or value <= 0:
        raise SystemExit(f"{name} must be a positive finite number")


def _announce_ready_and_wait(args: argparse.Namespace) -> None:
    args.ready_file.parent.mkdir(parents=True, exist_ok=True)
    if args.ready_file.exists() or args.start_gate_file.exists():
        raise RuntimeError("ready/start gate path already exists")
    temporary = args.ready_file.with_suffix(args.ready_file.suffix + ".tmp")
    temporary.write_text(
        json.dumps({"ready_at_monotonic_s": time.monotonic()}, sort_keys=True) + "\n"
    )
    temporary.replace(args.ready_file)
    deadline = time.monotonic() + args.gate_timeout_s
    while not args.start_gate_file.is_file():
        if time.monotonic() >= deadline:
            raise TimeoutError("operator start gate timed out")
        time.sleep(0.05)


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    _validate_positive(args.duration_s, "--duration-s")
    _validate_positive(args.rate_hz, "--rate-hz")
    _validate_positive(args.gate_timeout_s, "--gate-timeout-s")
    source = XRoboToolkitSource(
        XRoboToolkitSourceConfig(sdk_site_packages=args.sdk_site_packages)
    )
    manifest = TraceManifestSeed(
        trace_id=args.output.name,
        label=args.label,
        source_type="xrobotoolkit",
        source_adapter_revision=args.adapter_revision,
        coordinate_frame="XRoboToolkit normalized controller frame; metres; quaternion xyzw",
    )
    source.connect()
    try:
        _announce_ready_and_wait(args)
        started = time.monotonic()
        with TraceRecorder(args.output, manifest) as recorder:
            while time.monotonic() - started < args.duration_s:
                cycle = time.monotonic()
                sample = source.sample()
                recorder.record(sample, source.last_raw_action)
                time.sleep(max(0.0, 1.0 / args.rate_hz - (time.monotonic() - cycle)))
    finally:
        source.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
