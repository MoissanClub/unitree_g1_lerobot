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
    assess_calibration_unit,
    assess_calibration_trace,
    pending_segments,
)


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("suite_dir", type=Path)
    result.add_argument("--max-attempts", type=int, default=3)
    result.add_argument("--rate-hz", type=float, default=60.0)
    result.add_argument("--prepare-s", type=float, default=7.0)
    result.add_argument("--subphase-prepare-s", type=float, default=2.0)
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
        "--dry-run",
        action="store_true",
        help="speak an accelerated walkthrough without connecting or writing data",
    )
    result.add_argument(
        "--dry-run-only",
        choices=tuple(segment.label for segment in CALIBRATION_PLAN),
        help="walk through only one named segment",
    )
    result.add_argument("--dry-run-pause-s", type=float, default=0.5)
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
    for name in (
        "rate_hz",
        "prepare_s",
        "subphase_prepare_s",
        "gate_timeout_s",
        "device_timeout_s",
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            raise SystemExit(f"--{name.replace('_', '-')} must be positive and finite")
    if not math.isfinite(args.dry_run_pause_s) or args.dry_run_pause_s < 0:
        raise SystemExit("--dry-run-pause-s must be nonnegative and finite")


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


def _run_dry_walkthrough(args: argparse.Namespace) -> int:
    segments = tuple(
        segment
        for segment in CALIBRATION_PLAN
        if args.dry_run_only is None or segment.label == args.dry_run_only
    )
    _voice(
        args,
        "Practice mode. No headset connection, tracking, recording, calibration, "
        "or quality decision will occur.",
    )
    current_phase = None
    current_position_group = None
    for index, segment in enumerate(segments):
        if segment.phase != current_phase:
            current_phase = segment.phase
            _voice(args, f"Starting the {segment.phase.replace('_', ' ')} phase.")
        same_position = segment.position_group == current_position_group
        if segment.phase == "calibration":
            if not same_position:
                _voice(
                    args,
                    f"Next calibration position: {segment.position_group}. "
                    f"{segment.preparation} Hold this arm position through all palm "
                    f"directions. In the real run, you will have {args.prepare_s:.0f} "
                    "seconds to prepare.",
                )
                time.sleep(args.dry_run_pause_s)
            _voice(args, _palm_capture_prompt(segment))
            _voice(
                args,
                f"Practice mode skips the {segment.duration_s:.0f} second recording "
                "wait.",
            )
        else:
            prompt = (
                f"Next trace: {segment.label.replace('-', ' ')}. "
                f"{segment.preparation} The recorded action will be: "
                f"{segment.instruction}"
            )
            _voice(
                args,
                f"{prompt} In the real run, you will have {args.prepare_s:.0f} seconds "
                "to prepare. Do not begin until I say start now.",
            )
            time.sleep(args.dry_run_pause_s)
            _voice(args, "Get ready. Recording starts in 3, 2, 1.")
            _voice(args, "Start now.")
            _voice(
                args,
                f"Practice mode skips the {segment.duration_s:.0f} second recording wait.",
            )
            _voice(
                args,
                "Recording stopped. Release the active grip and relax while I check "
                "the trace quality.",
            )
        next_segment = segments[index + 1] if index + 1 < len(segments) else None
        unit_finished = segment.phase == "calibration" and (
            next_segment is None
            or next_segment.position_group != segment.position_group
        )
        if unit_finished:
            _voice(
                args,
                f"The {segment.position_group} position is captured. You may relax. "
                "In the real run, I will now evaluate the complete "
                f"{segment.position_group} position unit. If it fails, every "
                "subphase in this position is repeated.",
            )
        elif segment.phase == "motion_validation":
            _voice(
                args,
                "In the real run, I will now say either that this motion trace "
                "passed or that this trace needs a redo.",
            )
        current_position_group = segment.position_group
    _voice(
        args,
        "Practice walkthrough complete. No data or calibration artifacts were created.",
    )
    return 0


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


_PALM_COMMANDS = {
    "up": "Turn both palms upward",
    "inward": "Turn both palms inward",
    "down": "Turn both palms downward",
    "outward": "Turn both palms outward",
    "forward": "Turn both palms forward",
    "backward": "Turn both palms backward",
}


def _palm_capture_prompt(segment: CalibrationSegment) -> str:
    direction = segment.label.rsplit("-", maxsplit=1)[-1]
    command = _PALM_COMMANDS.get(direction)
    if command is None:
        raise ValueError(f"no concise palm command for {segment.label}")
    return f"{command}. 3, 2, 1. Start."


def _artifact_paths(
    suite_dir: Path,
    segment: CalibrationSegment,
    attempt_number: int,
) -> tuple[Path, Path]:
    if segment.phase == "motion_validation":
        trace_root = suite_dir / "traces"
        report_root = suite_dir / "trace-reports"
        trace_dir = trace_root / segment.label / f"attempt-{attempt_number:02d}"
        report_path = report_root / f"{segment.label}-attempt-{attempt_number:02d}.json"
    else:
        unit_root = (
            suite_dir
            / "calibration"
            / "units"
            / str(segment.position_group)
            / f"attempt-{attempt_number:02d}"
        )
        trace_dir = unit_root / "anchors" / segment.label
        report_path = unit_root / "reports" / f"{segment.label}.json"
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
            "position_group": segment.position_group,
            "position_anchor": segment.position_anchor,
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
            "calibration_scope": "human_vr",
            "consumer_boundary": "retargeting_before_robot_ik",
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


def _next_unit_attempt_number(suite_dir: Path, position_group: str) -> int:
    root = suite_dir / "calibration" / "units" / position_group
    numbers = []
    for path in root.glob("attempt-*"):
        try:
            numbers.append(int(path.name.removeprefix("attempt-")))
        except ValueError:
            continue
    return max(numbers, default=0) + 1


def _selected_calibration_reports(
    manifest: dict[str, object],
) -> dict[str, dict[str, object]]:
    reports = {}
    for segment in CALIBRATION_PLAN:
        if segment.phase != "calibration":
            continue
        entry = manifest["segments"].get(segment.label, {})
        if entry.get("status") != "passed":
            continue
        selected = next(
            attempt
            for attempt in entry["attempts"]
            if attempt["trace_path"] == entry["selected_trace"]
        )
        reports[segment.label] = json.loads(Path(selected["report_path"]).read_text())
    return reports


def _capture_calibration_unit(
    source,
    args: argparse.Namespace,
    suite_dir: Path,
    manifest: dict[str, object],
    position_group: str,
    unit_attempt: int,
) -> dict[str, object]:
    segments = tuple(
        segment
        for segment in CALIBRATION_PLAN
        if segment.position_group == position_group
    )
    unit_dir = (
        suite_dir
        / "calibration"
        / "units"
        / position_group
        / f"attempt-{unit_attempt:02d}"
    )
    reports = _selected_calibration_reports(manifest)

    for index, segment in enumerate(segments):
        trace_dir = unit_dir / "anchors" / segment.label
        report_path = unit_dir / "reports" / f"{segment.label}.json"
        if index == 0:
            _voice(
                args,
                f"Next calibration position: {position_group}. {segment.preparation} "
                f"Hold this arm position through all palm directions. You have "
                f"{args.prepare_s:.0f} seconds to prepare.",
            )
            time.sleep(args.prepare_s)
        if not _wait_for_device(source, args):
            report = {
                "passed": False,
                "issues": ["both controllers were not tracked before recording"],
                "calibration_segment": segment.to_dict(),
            }
        else:
            _voice(args, _palm_capture_prompt(segment))
            _timeline(
                args,
                f"calibration unit {position_group} attempt {unit_attempt} "
                f"anchor {segment.label} started",
            )
            _record_attempt(source, args, segment, trace_dir)
            report = assess_calibration_trace(trace_dir, segment)
        _write_json(report_path, report)
        reports[segment.label] = report

        entry = manifest["segments"].setdefault(
            segment.label,
            {"status": "pending", "selected_trace": None, "attempts": []},
        )
        entry["attempts"].append(
            {
                "attempt": len(entry["attempts"]) + 1,
                "calibration_unit": position_group,
                "unit_attempt": unit_attempt,
                "captured_at_utc": _utc_now(),
                "trace_path": str(trace_dir.resolve()) if trace_dir.exists() else None,
                "report_path": str(report_path.resolve()),
                "passed_individual_gate": bool(report["passed"]),
                "issues": list(report["issues"]),
            }
        )
        if index == len(segments) - 1:
            _voice(
                args,
                f"The {position_group} position is captured. You may relax while I "
                "check the complete unit.",
            )

    unit_report = assess_calibration_unit(position_group, reports)
    unit_report["unit_attempt"] = unit_attempt
    unit_report["unit_dir"] = str(unit_dir.resolve())
    unit_report_path = unit_dir / "unit-report.json"
    _write_json(unit_report_path, unit_report)

    for segment in segments:
        entry = manifest["segments"][segment.label]
        current_attempt = entry["attempts"][-1]
        current_attempt["passed"] = bool(unit_report["passed"])
        if unit_report["passed"]:
            entry["status"] = "passed"
            entry["selected_trace"] = current_attempt["trace_path"]
        else:
            entry["status"] = "failed"
            entry["selected_trace"] = None

    manifest.setdefault("calibration_units", []).append(
        {
            "position_group": position_group,
            "unit_attempt": unit_attempt,
            "captured_at_utc": _utc_now(),
            "passed": bool(unit_report["passed"]),
            "report_path": str(unit_report_path.resolve()),
            "issues": list(unit_report["issues"]),
        }
    )
    manifest["updated_at_utc"] = _utc_now()
    _write_json(suite_dir / "manifest.json", manifest)
    return unit_report


def _run_calibration_units(
    source,
    args: argparse.Namespace,
    suite_dir: Path,
    manifest: dict[str, object],
) -> bool:
    for position_group in ("down", "forward", "horizontal", "up"):
        segments = tuple(
            segment
            for segment in CALIBRATION_PLAN
            if segment.position_group == position_group
        )
        if all(
            manifest["segments"].get(segment.label, {}).get("status") == "passed"
            for segment in segments
        ):
            continue
        for local_attempt in range(1, args.max_attempts + 1):
            unit_attempt = _next_unit_attempt_number(suite_dir, position_group)
            _voice(
                args,
                f"Starting {position_group} calibration attempt {local_attempt} of "
                f"{args.max_attempts}.",
            )
            report = _capture_calibration_unit(
                source,
                args,
                suite_dir,
                manifest,
                position_group,
                unit_attempt,
            )
            if report["passed"]:
                _voice(
                    args,
                    f"The {position_group} calibration unit passed. Continue to the "
                    "next position.",
                )
                break
            summary = " ".join(str(issue) for issue in report["issues"][:3])
            if local_attempt < args.max_attempts:
                _voice(
                    args,
                    f"The {position_group} calibration unit did not pass. {summary} "
                    f"Redo the complete {position_group} position unit.",
                )
            else:
                _voice(
                    args,
                    f"The {position_group} calibration unit did not pass after the "
                    "allowed attempts. Motion traces will not be recorded. Run script "
                    "2 again to retry this position unit.",
                )
                return False
    return True


def _attempt_segment(
    source,
    args: argparse.Namespace,
    suite_dir: Path,
    manifest: dict[str, object],
    segment: CalibrationSegment,
    initial_prepare_s: float,
    same_position: bool,
) -> bool:
    segments = manifest["segments"]
    entry = segments.setdefault(
        segment.label,
        {"status": "pending", "selected_trace": None, "attempts": []},
    )
    attempts = entry["attempts"]

    for local_attempt in range(1, args.max_attempts + 1):
        preparation_s = (
            initial_prepare_s if local_attempt == 1 else args.subphase_prepare_s
        )
        first_trace_dir, _ = _artifact_paths(suite_dir, segment, 1)
        existing_attempts = tuple(first_trace_dir.parent.glob("attempt-*"))
        attempt_number = max(len(attempts), len(existing_attempts)) + 1
        trace_dir, report_path = _artifact_paths(suite_dir, segment, attempt_number)
        if segment.position_group is not None and (same_position or local_attempt > 1):
            prompt = (
                f"Next palm subphase: {segment.label.replace('-', ' ')}. "
                f"{segment.instruction}"
            )
        else:
            prompt = (
                f"Next trace: {segment.label.replace('-', ' ')}. "
                f"{segment.preparation} The recorded action will be: "
                f"{segment.instruction}"
            )
        _voice(
            args,
            f"{prompt} You have {preparation_s:.0f} seconds to prepare. Do not "
            "begin until I say start now.",
        )
        time.sleep(preparation_s)
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
            if segment.phase == "motion_validation":
                _voice(
                    args,
                    "Recording stopped. Release the active grip and relax while I "
                    "check the trace quality.",
                )
            else:
                _voice(
                    args,
                    "Recording stopped. Relax while I check the trace quality.",
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
    if args.dry_run:
        return _run_dry_walkthrough(args)
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
        calibration_passed = _calibration_is_complete(manifest)
        if not calibration_passed:
            _voice(
                args,
                "Starting four human VR calibration position units. Each position "
                "is checked and retried independently.",
            )
            calibration_passed = _run_calibration_units(
                source, args, args.suite_dir, manifest
            )
        if calibration_passed:
            anchor_set_path = _write_anchor_set(args.suite_dir, manifest)
            _voice(
                args,
                "Calibration passed and its immutable anchor set is saved. Starting "
                "the separate motion trace phase.",
            )
            _timeline(args, f"calibration anchor set: {anchor_set_path}")
            for segment in pending_segments(manifest):
                if segment.phase != "motion_validation":
                    continue
                _attempt_segment(
                    source,
                    args,
                    args.suite_dir,
                    manifest,
                    segment,
                    args.prepare_s,
                    False,
                )
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
