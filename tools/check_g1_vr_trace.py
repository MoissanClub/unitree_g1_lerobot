#!/usr/bin/env python3
"""Validate a recorded G1 VR trace and write a machine-readable report."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from unitree_g1_lerobot.diagnostics.xr.trace_validation import (
    EXPECTATIONS,
    validate_trace,
)


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("trace", type=Path)
    result.add_argument("--expect", choices=EXPECTATIONS, default="any")
    result.add_argument("--expected-rate-hz", type=float, default=60.0)
    result.add_argument("--output", type=Path)
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    report = validate_trace(
        args.trace,
        expectation=args.expect,
        expected_rate_hz=args.expected_rate_hz,
    )
    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered)
    measurements = report["measurements"]
    print(
        f"{'PASS' if report['passed'] else 'FAIL'} {args.trace}: "
        f"{measurements['sample_count']} samples, "
        f"{measurements['duration_s']:.3f} s, "
        f"{measurements['effective_rate_hz']:.1f} Hz"
    )
    for side in ("left", "right"):
        stats = measurements[side]
        print(
            f"  {side}: tracked={stats['tracked_ratio']:.1%}, "
            f"grip_max={stats['squeeze_max']:.2f}, "
            f"motion={stats['position_displacement_m']:.3f} m, "
            f"rotation={stats['rotation_displacement_rad']:.3f} rad"
        )
    print(f"  states: {measurements['source_states']}")
    for issue in report["issues"]:
        print(f"  issue: {issue}")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
