"""Owned, loopback-only simulator sessions for the live acceptance runner."""
import fcntl
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from ..shared.acceptance_metrics import LIMITS
from ..shared.reporting import revision, source_hashes, write_report

ROOT = Path(__file__).resolve().parents[3]


def run_matrix(args):
    with open(f"/tmp/lerobot-g1-sim-{os.getuid()}.lock", "a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    metadata = dict(project_revision=revision(ROOT), lerobot_revision=revision(ROOT.parent/"lerobot"),
                    diagnostic_sha256=hashlib.sha256((ROOT / "unitree_g1_lerobot/diagnostics/verify_live_control.py").read_bytes()).hexdigest(),
                    thresholds=LIMITS, python=sys.version, outcomes={})
    metadata["geometry_enabled"] = args.geometry
    metadata["diagnostic_source_sha256"] = source_hashes(ROOT, ROOT / "unitree_g1_lerobot/diagnostics")
    print(f"Reports: {output}", flush=True)
    env = dict(os.environ, HF_HUB_OFFLINE="1", PYTHONUNBUFFERED="1")
    env.pop("DISPLAY", None)
    for variant in args.embodiment or ("g1_29", "g1_23"):
        sim = None
        try:
            geometry_options = ["--geometry-log", str(output/f"{variant}-geometry.jsonl")] if args.geometry else []
            with (output/f"{variant}-sim.log").open("w") as log:
                sim = subprocess.Popen([str(ROOT/"run_g1_mujoco_dds_sim.sh"), "--embodiment", variant,
                                        "--headless", "--skip-startup-diagnostic", "--duration-s", "240", *geometry_options],
                                       cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            time.sleep(5)
            if sim.poll() is not None:
                raise RuntimeError(f"Simulator exited; inspect {variant}-sim.log")
            with (output/f"{variant}-control.log").open("w") as log:
                child = subprocess.run([sys.executable, "-m", "unitree_g1_lerobot.diagnostics.verify_live_control",
                                        "--child", variant, "--report", str(output/f"{variant}.json"), *geometry_options],
                                       cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=220)
            metadata["outcomes"][variant] = child.returncode
            if sim.poll() is not None:
                raise RuntimeError("Simulator stopped before acceptance completed")
            print(f"{variant}: {'PASS' if child.returncode == 0 else 'FAIL'}", flush=True)
        finally:
            if sim is not None and sim.poll() is None:
                os.killpg(sim.pid, signal.SIGINT)
                try:
                    sim.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(sim.pid, signal.SIGKILL)
                    sim.wait()
            if sim is not None:
                metadata.setdefault("simulator_exit_codes", {})[variant] = sim.returncode
            write_report(output/"manifest.json", metadata)
    return int(any(metadata["outcomes"].values()) or any(metadata["simulator_exit_codes"].values()))
