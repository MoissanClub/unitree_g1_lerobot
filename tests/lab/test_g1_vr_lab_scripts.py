from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

from tools.record_xrobotoolkit_calibration import (
    _artifact_paths,
    _palm_capture_prompt,
)
from unitree_g1_lerobot.diagnostics.xr.calibration import CALIBRATION_PLAN

ROOT = Path(__file__).resolve().parents[2]
LAB = ROOT / "lab" / "g1_vr"
SCRIPTS = tuple(sorted(LAB.glob("0*.sh")))


def test_operator_workflow_has_four_numbered_scripts():
    assert [path.name for path in SCRIPTS] == [
        "01_precheck.sh",
        "02_record_trace.sh",
        "03_replay_trace.sh",
        "04_live_e2e.sh",
    ]


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


def test_precheck_helper_starts_missing_xrobotoolkit_service(tmp_path):
    service = tmp_path / "service"
    service.mkdir()
    marker = tmp_path / "started"
    launcher = service / "runService.sh"
    launcher.write_text(f"#!/usr/bin/env bash\ntouch {marker}\n")
    launcher.chmod(0o755)
    process = service / "RoboticsServiceProcess"
    process.write_text("fixture\n")
    process.chmod(0o755)
    log = tmp_path / "service.log"
    command = f"""
        export G1_VR_XRBT_PC_SERVICE_DIR={service}
        source {LAB / 'lib.sh'}
        ss() {{
            if [[ -f {marker} ]]; then
                printf 'LISTEN 0 1 127.0.0.1:60061 0.0.0.0:*\\n'
                printf 'LISTEN 0 1 0.0.0.0:63901 0.0.0.0:*\\n'
            fi
        }}
        pgrep() {{ return 1; }}
        systemd-run() {{ touch {marker}; }}
        ensure_xrobotoolkit_pc_service {log}
    """

    result = subprocess.run(
        ["bash", "-c", command], check=False, capture_output=True, text=True
    )

    assert result.returncode == 0, result.stderr
    assert marker.exists()


def test_precheck_helper_refuses_duplicate_partial_service(tmp_path):
    command = f"""
        source {LAB / 'lib.sh'}
        ss() {{ printf 'LISTEN 0 1 127.0.0.1:60061 0.0.0.0:*\\n'; }}
        pgrep() {{ return 0; }}
        ensure_xrobotoolkit_pc_service {tmp_path / 'service.log'}
    """

    result = subprocess.run(
        ["bash", "-c", command], check=False, capture_output=True, text=True
    )

    assert result.returncode == 1
    assert "refusing to launch a duplicate" in result.stderr


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


def test_script_three_selects_all_current_suite_traces_by_default(tmp_path):
    right_trace = tmp_path / "right-forward"
    left_trace = tmp_path / "left-forward"
    session = tmp_path / "session"
    session.mkdir()
    (session / "precheck.passed").write_text("fixture\n")
    suite = session / "calibration-suites" / "suite-01"
    suite.mkdir(parents=True)
    (session / "calibration-suites" / "current-suite").write_text(
        f"{suite}\n"
    )
    for trace, scenario in ((right_trace, "right"), (left_trace, "left")):
        subprocess.run(
            [
                sys.executable,
                str(ROOT / "tools" / "synthesize_g1_vr_trace.py"),
                str(trace),
                "--scenario",
                scenario,
                "--duration-s",
                "1",
            ],
            check=True,
        )
    (suite / "manifest.json").write_text(
        json.dumps(
            {
                "status": "complete",
                "segments": {
                    "right-forward": {
                        "status": "passed",
                        "selected_trace": str(right_trace),
                    },
                    "left-forward": {
                        "status": "passed",
                        "selected_trace": str(left_trace),
                    },
                }
            }
        )
    )
    env = {
        **os.environ,
        "G1_VR_LAB_SESSION_DIR": str(session),
        "G1_VR_PYTHON": sys.executable,
        "PYTHONPATH": str(ROOT),
    }

    result = subprocess.run(
        [
            str(LAB / "03_replay_trace.sh"),
            "--validate-only",
        ],
        check=False,
        capture_output=True,
        text=True,
        env=env,
    )

    assert result.returncode == 0, result.stderr
    assert "Selected 2 accepted traces from the current suite" in result.stdout
    assert "Validation-only run complete for 2 trace(s)" in result.stdout
    assert (session / "checks" / "right-forward.json").exists()
    assert (session / "checks" / "left-forward.json").exists()
    assert not (session / "telemetry").exists()


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
    assert labels[:10] == [
        "calibration-down-inward",
        "calibration-forward-up",
        "calibration-forward-inward",
        "calibration-forward-down",
        "calibration-forward-outward",
        "calibration-horizontal-up",
        "calibration-horizontal-forward",
        "calibration-horizontal-down",
        "calibration-horizontal-backward",
        "calibration-up-inward",
    ]
    assert labels[-1] == "left-outward"


def test_calibration_and_motion_artifacts_are_stored_separately(tmp_path):
    by_label = {segment.label: segment for segment in CALIBRATION_PLAN}

    anchor, anchor_report = _artifact_paths(
        tmp_path, by_label["calibration-down-inward"], 1
    )
    trace, trace_report = _artifact_paths(tmp_path, by_label["right-forward"], 1)

    assert anchor == tmp_path / (
        "calibration/units/down/attempt-01/anchors/calibration-down-inward"
    )
    assert anchor_report == (
        tmp_path
        / "calibration/units/down/attempt-01/reports/calibration-down-inward.json"
    )
    assert trace == tmp_path / "traces/right-forward/attempt-01"
    assert trace_report == tmp_path / "trace-reports/right-forward-attempt-01.json"


def test_calibration_subphases_use_concise_continuous_prompts():
    by_label = {segment.label: segment for segment in CALIBRATION_PLAN}

    assert _palm_capture_prompt(by_label["calibration-forward-up"]) == (
        "Turn both palms upward. 3, 2, 1. Start."
    )
    assert _palm_capture_prompt(by_label["calibration-forward-inward"]) == (
        "Turn both palms inward. 3, 2, 1. Start."
    )
    assert _palm_capture_prompt(by_label["calibration-horizontal-backward"]) == (
        "Turn both palms backward. 3, 2, 1. Start."
    )


def test_script_two_dry_run_needs_no_precheck_headset_or_data_write(tmp_path):
    output_root = tmp_path / "outputs"
    env = {
        **os.environ,
        "G1_VR_LAB_OUTPUT_ROOT": str(output_root),
        "G1_VR_PYTHON": sys.executable,
        "G1_VR_TTS_BIN": "/bin/true",
    }

    result = subprocess.run(
        [
            str(LAB / "02_record_trace.sh"),
            "--dry-run",
            "--dry-run-only",
            "calibration-down-inward",
        ],
        check=False,
        capture_output=True,
        text=True,
        env=env,
    )

    assert result.returncode == 0, result.stderr
    assert "Practice mode" in result.stdout
    assert "Completely relax both arms" in result.stdout
    assert "Practice walkthrough complete" in result.stdout
    assert not output_root.exists()


def test_script_two_dry_run_speaks_short_palm_subphase_command(tmp_path):
    env = {
        **os.environ,
        "G1_VR_LAB_OUTPUT_ROOT": str(tmp_path / "outputs"),
        "G1_VR_PYTHON": sys.executable,
        "G1_VR_TTS_BIN": "/bin/true",
    }

    result = subprocess.run(
        [
            str(LAB / "02_record_trace.sh"),
            "--dry-run",
            "--dry-run-only",
            "calibration-forward-inward",
        ],
        check=False,
        capture_output=True,
        text=True,
        env=env,
    )

    assert result.returncode == 0, result.stderr
    assert "Turn both palms inward. 3, 2, 1. Start." in result.stdout
    assert "Next palm subphase" not in result.stdout
    assert "Get ready" not in result.stdout
    assert "2 seconds to settle" not in result.stdout
