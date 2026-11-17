#!/usr/bin/env python3
"""Collect and immediately validate a resumable XR calibration suite."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

from lerobot_teleoperator_g1_vr import (
    TraceManifestSeed,
    TraceRecorder,
    XRoboToolkitSource,
    XRoboToolkitSourceConfig,
)

from unitree_g1_lerobot.diagnostics.xr.calibration import (
    CALIBRATION_PLAN,
    CALIBRATION_PLAN_VERSION,
    CalibrationSegment,
    assess_calibration_trace,
    pending_segments,
)


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("suite_dir", type=Path)
    result.add_argument("--max-attempts", type=int, default=3)
    result.add_argument("--rate-hz", type=float, default=60.0)
    result.add_argument("--prepare-s", type=float, default=7.0)
    result.add_argument("--sdk-site-packages", type=Path)
    result.add_argument("--adapter-revision", default="working-tree")
    result.add_argument("--ready-file", type=Path)
    result.add_argument("--start-gate-file", type=Path)
    result.add_argument("--gate-timeout-s", type=float, default=600.0)
    result.add_argument("--device-timeout-s", type=float, default=60.0)
    result.add_argument("--tts-bin", default="spd-say")
    result.add_argument("--voice-rate", default="-35")
    result.add_argument("--timeline", type=Path)
    result.add_argument(
        "--list-pending",
        action="store_true",
        help="print missing/failed labels without connecting to XRoboToolkit",
    )
    return result


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _load_manifest(path: Path) -> dict[str, object] | None:
    if not path.exists():
        return None
    manifest = json.loads(path.read_text())
    if manifest.get("plan_version") != CALIBRATION_PLAN_VERSION:
        raise RuntimeError(
            "existing suite uses a different calibration plan; use --new-suite"
        )
    return manifest


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    temporary.replace(path)


def _new_manifest(suite_dir: Path) -> dict[str, object]:
    return {
        "schema_version": 1,
        "plan_version": CALIBRATION_PLAN_VERSION,
        "suite_id": suite_dir.name,
        "created_at_utc": _utc_now(),
        "updated_at_utc": _utc_now(),
        "status": "in_progress",
        "segments": {},
    }


def _validate_args(args: argparse.Namespace) -> None:
    if args.max_attempts < 1:
        raise SystemExit("--max-attempts must be at least 1")
    for name in ("rate_hz", "prepare_s", "gate_timeout_s", "device_timeout_s"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            raise SystemExit(f"--{name.replace('_', '-')} must be positive and finite")


def _timeline(args: argparse.Namespace, message: str) -> None:
    if args.timeline is None:
        return
    args.timeline.parent.mkdir(parents=True, exist_ok=True)
    with args.timeline.open("a") as stream:
        stream.write(f"{datetime.now().astimezone().isoformat()}\t{message}\n")


def _voice(args: argparse.Namespace, message: str) -> None:
    print(f"VOICE: {message}", flush=True)
    _timeline(args, f"voice: {message}")
    subprocess.run(
        [args.tts_bin, "-w", "-r", str(args.voice_rate), message],
        check=False,
    )


def _announce_ready_and_wait(args: argparse.Namespace) -> None:
    if args.ready_file is None or args.start_gate_file is None:
        raise SystemExit("--ready-file and --start-gate-file are required for capture")
    args.ready_file.parent.mkdir(parents=True, exist_ok=True)
    if args.ready_file.exists() or args.start_gate_file.exists():
        raise RuntimeError("ready/start gate path already exists")
    _write_json(args.ready_file, {"ready_at_monotonic_s": time.monotonic()})
    deadline = time.monotonic() + args.gate_timeout_s
    while not args.start_gate_file.is_file():
        if time.monotonic() >= deadline:
            raise TimeoutError("operator start gate timed out")
        time.sleep(0.05)


def _wait_for_device(source, args: argparse.Namespace) -> bool:
    deadline = time.monotonic() + args.device_timeout_s
    warning_at = time.monotonic() + 5.0
    warned = False
    while time.monotonic() < deadline:
        sample = source.sample()
        if (
            sample.status.state.value == "ready"
            and sample.left.tracked
            and sample.right.tracked
        ):
            return True
        if not warned and time.monotonic() >= warning_at:
            _voice(
                args,
                "I am not receiving both controllers. In the headset, reconnect the "
                "XRoboToolkit app. I will keep waiting.",
            )
            warned = True
        time.sleep(0.05)
    return False


def _record_attempt(
    source,
    args: argparse.Namespace,
    segment: CalibrationSegment,
    trace_dir: Path,
) -> None:
    seed = TraceManifestSeed(
        trace_id=f"{segment.label}-{trace_dir.name}",
        label=segment.label,
        source_type="xrobotoolkit",
        source_adapter_revision=args.adapter_revision,
        coordinate_frame=(
            "XRoboToolkit normalized controller frame; metres; quaternion xyzw"
        ),
    )
    started = time.monotonic()
    with TraceRecorder(trace_dir, seed) as recorder:
        while time.monotonic() - started < segment.duration_s:
            cycle = time.monotonic()
            sample = source.sample()
            recorder.record(sample, source.last_raw_action)
            time.sleep(max(0.0, 1.0 / args.rate_hz - (time.monotonic() - cycle)))


def _quality_summary(report: dict[str, object]) -> str:
    issues = report["issues"]
    if not issues:
        return ""
    joined = " ".join(str(issue) for issue in issues[:3])
    return f"The quality check reported: {joined}."


def _artifact_paths(
    suite_dir: Path,
    segment: CalibrationSegment,
    attempt_number: int,
) -> tuple[Path, Path]:
    if segment.phase == "motion_validation":
        trace_root = suite_dir / "traces"
        report_root = suite_dir / "trace-reports"
    else:
        trace_root = suite_dir / "calibration" / "anchors"
        report_root = suite_dir / "calibration" / "reports"
    trace_dir = trace_root / segment.label / f"attempt-{attempt_number:02d}"
    report_path = report_root / f"{segment.label}-attempt-{attempt_number:02d}.json"
    return trace_dir, report_path


def _calibration_is_complete(manifest: dict[str, object]) -> bool:
    entries = manifest["segments"]
    return all(
        entries.get(segment.label, {}).get("status") == "passed"
        for segment in CALIBRATION_PLAN
        if segment.phase != "motion_validation"
    )


def _write_anchor_set(suite_dir: Path, manifest: dict[str, object]) -> Path:
    anchor_set_path = suite_dir / "calibration" / "anchor-set.json"
    if anchor_set_path.exists():
        digest = hashlib.sha256(anchor_set_path.read_bytes()).hexdigest()
        manifest["calibration_anchor_set"] = {
            "path": str(anchor_set_path.resolve()),
            "sha256": digest,
        }
        _write_json(suite_dir / "manifest.json", manifest)
        return anchor_set_path

    anchors = {}
    for segment in CALIBRATION_PLAN:
        if segment.phase == "motion_validation":
            continue
        entry = manifest["segments"][segment.label]
        selected = next(
            attempt
            for attempt in entry["attempts"]
            if attempt["passed"] and attempt["trace_path"] == entry["selected_trace"]
        )
        report = json.loads(Path(selected["report_path"]).read_text())
        anchors[segment.label] = {
            "phase": segment.phase,
            "selected_trace": selected["trace_path"],
            "report_path": selected["report_path"],
            "representative_poses": report["representative_poses"],
        }
    _write_json(
        anchor_set_path,
        {
            "schema_version": 1,
            "plan_version": CALIBRATION_PLAN_VERSION,
            "created_at_utc": _utc_now(),
            "coordinate_frame": (
                "XRoboToolkit normalized controller frame; metres; quaternion xyzw"
            ),
            "status": "complete_observation_set_transform_not_yet_solved",
            "anchors": anchors,
        },
    )
    digest = hashlib.sha256(anchor_set_path.read_bytes()).hexdigest()
    manifest["calibration_anchor_set"] = {
        "path": str(anchor_set_path.resolve()),
        "sha256": digest,
    }
    _write_json(suite_dir / "manifest.json", manifest)
    return anchor_set_path


def _attempt_segment(
    source,
    args: argparse.Namespace,
    suite_dir: Path,
    manifest: dict[str, object],
    segment: CalibrationSegment,
) -> bool:
    segments = manifest["segments"]
    entry = segments.setdefault(
        segment.label,
        {"status": "pending", "selected_trace": None, "attempts": []},
    )
    attempts = entry["attempts"]

    for local_attempt in range(1, args.max_attempts + 1):
        first_trace_dir, _ = _artifact_paths(suite_dir, segment, 1)
        existing_attempts = tuple(first_trace_dir.parent.glob("attempt-*"))
        attempt_number = max(len(attempts), len(existing_attempts)) + 1
        trace_dir, report_path = _artifact_paths(suite_dir, segment, attempt_number)
        _voice(
            args,
            f"Next trace: {segment.label.replace('-', ' ')}. {segment.preparation} "
            f"The recorded action will be: {segment.instruction} "
            f"You have {args.prepare_s:.0f} seconds to prepare. Do not begin until "
            "I say start now.",
        )
        time.sleep(args.prepare_s)
        if not _wait_for_device(source, args):
            report = {
                "passed": False,
                "issues": ["both controllers were not tracked before recording"],
                "calibration_segment": segment.to_dict(),
            }
        else:
            _voice(args, "Get ready. Recording starts in 3, 2, 1.")
            _voice(args, "Start now.")
            _timeline(
                args,
                f"calibration trace {segment.label} attempt {attempt_number} started",
            )
            _record_attempt(source, args, segment, trace_dir)
            _voice(
                args,
                "Recording stopped. Release both grip buttons and relax while I "
                "check the trace quality.",
            )
            report = assess_calibration_trace(trace_dir, segment)

        _write_json(report_path, report)
        attempt = {
            "attempt": attempt_number,
            "captured_at_utc": _utc_now(),
            "trace_path": str(trace_dir.resolve()) if trace_dir.exists() else None,
            "report_path": str(report_path.resolve()),
            "passed": bool(report["passed"]),
            "issues": list(report["issues"]),
        }
        if segment.phase == "motion_validation":
            attempt["calibration_anchor_set_sha256"] = manifest[
                "calibration_anchor_set"
            ]["sha256"]
        attempts.append(attempt)
        manifest["updated_at_utc"] = _utc_now()

        if report["passed"]:
            entry["status"] = "passed"
            entry["selected_trace"] = attempt["trace_path"]
            _write_json(suite_dir / "manifest.json", manifest)
            _timeline(args, f"calibration trace {segment.label} passed")
            _voice(
                args,
                f"The {segment.label.replace('-', ' ')} trace passed its quality "
                "check. Do not repeat it.",
            )
            return True

        entry["status"] = "failed"
        _write_json(suite_dir / "manifest.json", manifest)
        _timeline(
            args,
            f"calibration trace {segment.label} attempt {attempt_number} failed: "
            + "; ".join(report["issues"]),
        )
        if local_attempt < args.max_attempts:
            _voice(
                args,
                f"The {segment.label.replace('-', ' ')} trace did not pass. "
                f"Redo required. {_quality_summary(report)} Return to the starting "
                "pose. I will repeat the instructions.",
            )
        else:
            _voice(
                args,
                f"The {segment.label.replace('-', ' ')} trace did not pass after "
                f"{args.max_attempts} attempts in this run. I will mark it failed and "
                "continue. Run script 2 again later to retry only failed traces.",
            )
    return False


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    _validate_args(args)
    manifest_path = args.suite_dir / "manifest.json"
    manifest = _load_manifest(manifest_path)
    pending = pending_segments(manifest)
    if args.list_pending:
        print("\n".join(segment.label for segment in pending))
        return 0
    if not pending:
        print("Calibration suite is already complete; no headset run is needed.")
        return 0

    args.suite_dir.mkdir(parents=True, exist_ok=True)
    if manifest is None:
        manifest = _new_manifest(args.suite_dir)
        _write_json(manifest_path, manifest)

    source = XRoboToolkitSource(
        XRoboToolkitSourceConfig(sdk_site_packages=args.sdk_site_packages)
    )
    source.connect()
    try:
        _announce_ready_and_wait(args)
        _voice(
            args,
            f"Calibration collection is starting. I need {len(pending)} traces. "
            "Keep both controllers visible and follow each instruction exactly.",
        )
        current_phase = None
        for segment in pending:
            if segment.phase == "motion_validation" and not _calibration_is_complete(
                manifest
            ):
                _voice(
                    args,
                    "Calibration anchors are still incomplete. I will not record "
                    "motion traces against an incomplete calibration. Run script 2 "
                    "again to retry only the failed calibration anchors.",
                )
                break
            if segment.phase != current_phase:
                current_phase = segment.phase
                phase_name = segment.phase.replace("_", " ")
                _voice(args, f"Starting the {phase_name} phase.")
                if segment.phase == "motion_validation":
                    anchor_set_path = _write_anchor_set(args.suite_dir, manifest)
                    _voice(
                        args,
                        "Calibration anchor collection passed. The immutable anchor "
                        "set is saved. Starting separate motion trace recording.",
                    )
                    _timeline(args, f"calibration anchor set: {anchor_set_path}")
            _attempt_segment(source, args, args.suite_dir, manifest, segment)
    finally:
        source.disconnect()

    remaining = pending_segments(manifest)
    manifest["status"] = "complete" if not remaining else "partial"
    manifest["updated_at_utc"] = _utc_now()
    _write_json(manifest_path, manifest)
    if remaining:
        labels = ", ".join(segment.label.replace("-", " ") for segment in remaining)
        _voice(
            args,
            f"This calibration run is finished. These traces still need a retry: "
            f"{labels}. Run script 2 again to collect only those traces.",
        )
        return 3
    _voice(
        args,
        "All calibration traces passed their immediate quality checks. No redo is needed.",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
