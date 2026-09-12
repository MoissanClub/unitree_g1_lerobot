"""Opt-in real native-simulation operator tests. No X, OpenXR, DDS, or hardware."""

import os
import json
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


@pytest.mark.parametrize("embodiment", ["g1_29", "g1_23"])
def test_stalled_viewer_does_not_block_control_or_camera(tmp_path, embodiment):
    import zmq

    config = json.loads((ROOT / ".vr-sim.json").read_text())
    common = [
        "--run-dir",
        str(tmp_path),
        "--assets",
        config["assets"],
        "--embodiment",
        embodiment,
    ]
    env = dict(
        os.environ, PYTHONPATH=str(Path(config["lerobot"]) / "src"), MUJOCO_GL="egl"
    )
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.RCVTIMEO, 2000)
    viewer_pid = None
    with (tmp_path / "sim.log").open("w") as log:
        process = subprocess.Popen(
            [
                config["python"],
                str(ROOT / "tests/fixtures/stalled_vr_viewer.py"),
                "simulator",
                *common,
            ],
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
        )
        try:
            deadline = time.monotonic() + 20
            while not (tmp_path / "simulator.ready").exists():
                assert process.poll() is None, (tmp_path / "sim.log").read_text()
                assert time.monotonic() < deadline
                time.sleep(0.1)
            viewer_pid = int((tmp_path / "viewer.pid").read_text())
            socket.connect("ipc://" + str(tmp_path / "control.sock"))
            socket.send_json({})
            state = socket.recv_json()
            socket.send_json(
                {
                    "q": state["q"],
                    "sent_at": time.monotonic() - 1,
                    "embodiment": embodiment,
                }
            )
            assert socket.recv_json()["status"] == "expired_command"
            (tmp_path / "cloudxr.ready").write_text("{}")
            result = subprocess.run(
                [
                    config["python"],
                    str(ROOT / "tools/g1_vr_service.py"),
                    "bridge",
                    *common,
                    "--replay",
                    "--steps",
                    "100",
                ],
                env=env,
                capture_output=True,
                text=True,
                timeout=30,
            )
            assert result.returncode == 0, result.stdout + result.stderr
            assert f"PASS {embodiment}" in result.stdout
            assert Path(f"/proc/{viewer_pid}").exists(), (
                "Viewer should still be blocked"
            )
        finally:
            (tmp_path / "stop").touch()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            socket.close()
            context.term()
        assert process.returncode == 0, (tmp_path / "sim.log").read_text()
        assert not Path(f"/proc/{viewer_pid}").exists(), "Viewer child leaked"


@pytest.mark.parametrize("embodiment,delay", [("g1_29", 0.8), ("g1_23", 3.0)])
def test_bridge_recovers_after_simulator_stall(embodiment, delay):
    process = subprocess.Popen(
        [
            str(ROOT / "run_g1_vr_sim.sh"),
            "--embodiment",
            embodiment,
            "--headless",
            "--steps",
            "180",
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    simulator = None
    try:
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            ids = subprocess.run(
                ["pgrep", "-P", str(process.pid)], text=True, capture_output=True
            )
            for pid in ids.stdout.split():
                try:
                    cmd = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
                except FileNotFoundError:
                    continue
                if b"simulator" in cmd:
                    run_dir = Path(os.fsdecode(cmd[cmd.index(b"--run-dir") + 1]))
                    if (run_dir / "bridge.ready").exists():
                        simulator = int(pid)
            if simulator:
                break
            assert process.poll() is None
            time.sleep(0.02)
        assert simulator is not None
        os.kill(simulator, signal.SIGSTOP)
        time.sleep(delay)
        os.kill(simulator, signal.SIGCONT)
        output, _ = process.communicate(timeout=30)
        assert process.returncode == 0, output
        assert f"PASS {embodiment}" in output
        assert "discarded" in output, output
    finally:
        if simulator is not None and Path(f"/proc/{simulator}").exists():
            os.kill(simulator, signal.SIGCONT)
        if process.poll() is None:
            process.terminate()
            process.communicate(timeout=30)
