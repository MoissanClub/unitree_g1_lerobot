"""Supervise the native simulator, XR bridge, and CloudXR as separate processes."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]


def shutdown_services(run, children):
    """Keep camera and runtime alive until the XR consumer has finished teardown."""
    by_role = dict(children)
    problems = []
    for role in ("bridge", "simulator", "cloudxr"):
        child = by_role.get(role)
        if child is None:
            continue
        (run / f"stop.{role}").touch()
        try:
            child.wait(timeout=10)
        except subprocess.TimeoutExpired:
            problems.append(f"{role} required forced termination")
            os.killpg(child.pid, signal.SIGTERM)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        if child.returncode:
            problems.append(f"{role} exited with code {child.returncode}")
    return problems


def wait_ready(path, children, timeout=180):
    deadline = time.monotonic() + timeout
    while not path.exists():
        for name, child in children:
            if child.poll() is not None:
                raise RuntimeError(
                    f"{name} exited before readiness with code {child.returncode}"
                )
        if time.monotonic() > deadline:
            raise TimeoutError(f"Timed out waiting for {path.name}")
        time.sleep(0.1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--embodiment", choices=("g1_29", "g1_23"), default="g1_29")
    parser.add_argument("--config", type=Path, default=ROOT / ".vr-sim.json")
    parser.add_argument(
        "--headless",
        action="store_true",
        help="No Tk; synthetic input unless --live-xr",
    )
    parser.add_argument(
        "--live-xr", action="store_true", help="Use real CloudXR/input even without Tk"
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=0,
        help="Bridge frames; 0 continuous live, 100 replay",
    )
    parser.add_argument("--accept-cloudxr-eula", action="store_true")
    args = parser.parse_args()
    if args.steps < 0:
        parser.error("steps must be nonnegative")
    if not args.config.is_file():
        parser.error("Run ./install_g1_vr_sim.sh first.")
    config = json.loads(args.config.read_text())
    python = Path(config["python"])
    source = Path(config["lerobot"])
    assets = Path(config["assets"])
    for path in (python, source / "src/lerobot", assets / f"{args.embodiment}.urdf"):
        if not path.exists():
            parser.error(f"Missing installed resource: {path}; rerun installation.")
    if not args.headless and not os.environ.get("DISPLAY"):
        parser.error(
            "No DISPLAY. Use ssh -Y, or --headless --live-xr for headset-only operation."
        )
    replay = args.headless and not args.live_xr
    steps = args.steps or (100 if replay else 0)
    env = dict(
        os.environ,
        PYTHONPATH=str(source / "src"),
        MUJOCO_GL="egl",
        PYTHONUNBUFFERED="1",
    )
    children, logs = [], []
    (ROOT / "outputs").mkdir(exist_ok=True)
    logdir = Path(
        tempfile.mkdtemp(
            prefix=time.strftime("vr-sim-%Y%m%d-%H%M%S-"), dir=ROOT / "outputs"
        )
    )
    print(f"Logs: {logdir}", flush=True)
    with tempfile.TemporaryDirectory(prefix="g1-vr-") as folder:
        run = Path(folder)
        (run / "managed").touch()
        common = [
            "--run-dir",
            folder,
            "--assets",
            str(assets),
            "--embodiment",
            args.embodiment,
        ]

        def start(role, extra=()):
            stream = (logdir / f"{role}.log").open("w")
            logs.append(stream)
            child = subprocess.Popen(
                [
                    str(python),
                    str(ROOT / "tools/g1_vr_service.py"),
                    role,
                    *common,
                    *extra,
                ],
                env=env,
                stdout=stream,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            children.append((role, child))
            return child

        def interrupted(*_):
            raise KeyboardInterrupt

        signal.signal(signal.SIGTERM, interrupted)
        try:
            start("simulator", ["--headless"] if args.headless else [])
            wait_ready(run / "simulator.ready", children)
            # CloudXR exports its resolved runtime environment for the separate bridge.
            cloud = ["--replay"] if replay else []
            if args.accept_cloudxr_eula:
                cloud.append("--accept-cloudxr-eula")
            start("cloudxr", cloud)
            wait_ready(run / "cloudxr.ready", children)
            bridge = start(
                "bridge", ["--steps", str(steps), *(["--replay"] if replay else [])]
            )
            print(
                f"{args.embodiment}: simulator ready; CloudXR {'skipped (replay)' if replay else 'ready'}. "
                "Bridge starting. Ctrl+C stops all three processes.",
                flush=True,
            )
            if not replay:
                print(
                    "Connect headset at https://nvidia.github.io/IsaacTeleop/client using this host's IP. "
                    "Accept its TLS certificate at https://HOST:48322 if needed.",
                    flush=True,
                )
            announced = False
            while bridge.poll() is None:
                if not announced and (run / "bridge.ready").exists():
                    print(
                        "Steady-state listening: independent left/right controller clutches; video enabled."
                        if not replay
                        else "Synthetic dual-arm replay running.",
                        flush=True,
                    )
                    announced = True
                if (run / "stop").exists():
                    break
                for name, child in children[:-1]:
                    if child.poll() is not None:
                        raise RuntimeError(
                            f"{name} stopped unexpectedly ({child.returncode})"
                        )
                time.sleep(0.1)
            if bridge.returncode:
                raise RuntimeError(
                    f"Bridge failed ({bridge.returncode}); inspect {logdir}"
                )
        except KeyboardInterrupt:
            print("Stopping session.", flush=True)
        except Exception:
            for name, _ in children:
                print(
                    f"--- {name} log ---\n"
                    + (logdir / f"{name}.log").read_text()[-3000:],
                    flush=True,
                )
            raise
        finally:
            previous = {
                sig: signal.signal(sig, signal.SIG_IGN)
                for sig in (signal.SIGINT, signal.SIGTERM)
            }
            try:
                print(
                    "Stopping XR bridge/video, then simulator, then CloudXR...",
                    flush=True,
                )
                problems = shutdown_services(run, children)
            finally:
                for stream in logs:
                    stream.close()
                for sig, handler in previous.items():
                    signal.signal(sig, handler)
            bridge_log = logdir / "bridge.log"
            if bridge_log.exists():
                for line in bridge_log.read_text().splitlines():
                    if line.startswith(
                        (
                            "PASS ",
                            "STOPPED ",
                            "COMPLETED ",
                            "ERROR ",
                            "XR_ERROR_",
                            "Traceback",
                            "Expired command",
                            "Waiting for fresh",
                        )
                    ):
                        print(line, flush=True)
            if problems:
                raise RuntimeError(
                    f"Shutdown problems: {'; '.join(problems)}. Logs: {logdir}"
                )
            print(f"Session stopped. Logs: {logdir}", flush=True)


if __name__ == "__main__":
    main()
