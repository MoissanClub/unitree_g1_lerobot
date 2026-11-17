from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

from tools.record_xrobotoolkit_calibration import _artifact_paths
from unitree_g1_lerobot.diagnostics.xr.calibration import CALIBRATION_PLAN

ROOT = Path(__file__).resolve().parents[2]
LAB = ROOT / "lab" / "g1_vr"
SCRIPTS = tuple(sorted(LAB.glob("0*.sh")))


def test_all_operator_scripts_pass_bash_syntax_check():
    subprocess.run(
        ["bash", "-n", str(LAB / "lib.sh"), *(str(path) for path in SCRIPTS)],
        check=True,
    )


def test_all_operator_scripts_expose_help_without_starting_hardware():
    for path in SCRIPTS:
        result = subprocess.run(
            [str(path), "--help"],
            check=False,
            capture_output=True,
            text=True,
        )
        assert result.returncode == 0, (path, result.stderr)
        assert "usage:" in result.stdout.lower()


def handoff_environment(tmp_path):
    session = tmp_path / "session"
    session.mkdir(exist_ok=True)
    return {
        **os.environ,
        "G1_VR_LAB_SESSION_DIR": str(session),
        "G1_VR_DONNING_SECONDS": "0",
        "G1_VR_TTS_BIN": "/bin/true",
    }


def test_headset_handoff_requires_exact_confirmation(tmp_path):
    command = f"source {LAB / 'lib.sh'}; confirm_headset_handoff"
    accepted = subprocess.run(
        ["bash", "-c", command],
        check=False,
        input="wearing the headset now\n",
        text=True,
        env=handoff_environment(tmp_path),
        capture_output=True,
    )
    rejected = subprocess.run(
        ["bash", "-c", command],
        check=False,
        input="ready\n",
        text=True,
        env=handoff_environment(tmp_path),
        capture_output=True,
    )

    assert accepted.returncode == 0
    assert rejected.returncode == 2
    assert "confirmation did not match" in rejected.stderr


def test_headset_removal_confirmation_can_run_without_real_audio(tmp_path):
    command = f"source {LAB / 'lib.sh'}; confirm_headset_removed"
    result = subprocess.run(
        ["bash", "-c", command],
        check=False,
        input="headset off\n",
        text=True,
        env=handoff_environment(tmp_path),
        capture_output=True,
    )

    assert result.returncode == 0
    assert "Remove the headset" in result.stdout


def test_synthetic_trace_cli_passes_bimanual_checker(tmp_path):
    trace = tmp_path / "synthetic-bimanual"
    report = tmp_path / "report.json"
    subprocess.run(
        [
            sys.executable,
            str(ROOT / "tools" / "synthesize_g1_vr_trace.py"),
            str(trace),
            "--scenario",
            "bimanual",
            "--duration-s",
            "1",
        ],
        check=True,
    )
    subprocess.run(
        [
            sys.executable,
            str(ROOT / "tools" / "check_g1_vr_trace.py"),
            str(trace),
            "--expect",
            "bimanual",
            "--output",
            str(report),
        ],
        check=True,
    )

    payload = json.loads(report.read_text())
    assert payload["passed"] is True
    assert payload["manifest"]["source_type"] == "synthetic"
    assert payload["measurements"]["left"]["squeeze_max"] == 1.0
    assert payload["measurements"]["right"]["squeeze_max"] == 1.0


def test_calibration_cli_lists_pending_without_connecting_to_headset(tmp_path):
    suite = tmp_path / "suite"
    result = subprocess.run(
        [
            sys.executable,
            str(ROOT / "tools" / "record_xrobotoolkit_calibration.py"),
            str(suite),
            "--list-pending",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    labels = result.stdout.splitlines()
    assert len(labels) == 16
    assert labels[:4] == [
        "position-natural-down",
        "position-full-up",
        "position-full-forward",
        "position-full-open",
    ]
    assert labels[-1] == "left-outward"


def test_calibration_and_motion_artifacts_are_stored_separately(tmp_path):
    by_label = {segment.label: segment for segment in CALIBRATION_PLAN}

    anchor, anchor_report = _artifact_paths(
        tmp_path, by_label["position-natural-down"], 1
    )
    trace, trace_report = _artifact_paths(tmp_path, by_label["right-forward"], 1)

    assert anchor == tmp_path / "calibration/anchors/position-natural-down/attempt-01"
    assert anchor_report == (
        tmp_path / "calibration/reports/position-natural-down-attempt-01.json"
    )
    assert trace == tmp_path / "traces/right-forward/attempt-01"
    assert trace_report == tmp_path / "trace-reports/right-forward-attempt-01.json"
