"""Opt-in real native-simulation operator tests. No X, OpenXR, DDS, or hardware."""

import os
from pathlib import Path
import signal
import subprocess
import time

import pytest

ROOT = Path(__file__).resolve().parents[2]
pytestmark = pytest.mark.skipif(
    not os.environ.get("G1_VR_OPERATOR_TESTS"),
    reason="Requires installed VR environment/GPU",
)


@pytest.mark.parametrize("embodiment", ["g1_29", "g1_23"])
def test_replay_processes(embodiment):
    result = subprocess.run(
        [str(ROOT / "run_g1_vr_sim.sh"), "--embodiment", embodiment, "--headless"],
        text=True,
        capture_output=True,
        timeout=90,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert f"PASS {embodiment}: 100 frames" in result.stdout


def test_simulator_failure_stops_other_services():
    process = subprocess.Popen(
        [str(ROOT / "run_g1_vr_sim.sh"), "--headless", "--steps", "10000"],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    children = []
    try:
        deadline = time.monotonic() + 30
        simulator = None
        while time.monotonic() < deadline:
            ids = subprocess.run(
                ["pgrep", "-P", str(process.pid)], text=True, capture_output=True
            )
            children = [int(pid) for pid in ids.stdout.split()]
            if len(children) == 3:
                for pid in children:
                    cmd = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
                    if b"simulator" in cmd:
                        simulator = pid
                if simulator:
                    break
            assert process.poll() is None
            time.sleep(0.1)
        assert simulator is not None, "Simulator did not become ready"
        os.kill(simulator, signal.SIGTERM)
        output, _ = process.communicate(timeout=30)
        assert process.returncode != 0, output
        assert (
            "simulator stopped unexpectedly" in output or "Bridge failed" in output
        ), output
        assert all(not Path(f"/proc/{pid}").exists() for pid in children)
    finally:
        if process.poll() is None:
            process.terminate()
            process.communicate(timeout=30)
