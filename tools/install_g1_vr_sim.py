"""Install a private, pinned LeRobot/Isaac Teleop environment without changing DDS setup."""

import argparse
import json
import os
from pathlib import Path
import platform
import shlex
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
COMMIT = "d5e400bcefeccc93ba956ce876530e5283512df1"
BRANCH = "integration/g1-acceptance"
REPOSITORY = "https://github.com/MoissanClub/lerobot.git"


def print_next_steps(root):
    print(f"""
======================================================================
INSTALLATION SUCCESSFUL
======================================================================
LeRobot, dependencies, and model assets are installed.
Both G1-29/G1-23 headless checks and offscreen GPU video checks passed.
Headset operation and the desktop viewer still need your visual review.

NEXT STEPS

1. Open a terminal on the simulation workstation.
   For remote access, connect from a computer with a working X server:
     ssh -Y YOUR_USER@WORKSTATION_IP
   A local desktop terminal also works; SSH is not required locally.
   Then enter this checkout (no conda activation is needed):
     cd {shlex.quote(str(root))}

2. Review the NVIDIA CloudXR EULA before first live use.
   Only if you accept, start the first session with:
     ./run_g1_vr_sim.sh --embodiment g1_29 --accept-cloudxr-eula
   This flag accepts the EULA AND starts the session; it is not a
   separate acceptance-only command. Installation did not accept it.

3. After acceptance is saved, start either robot (one session at a time):
     ./run_g1_vr_sim.sh --embodiment g1_29
     ./run_g1_vr_sim.sh --embodiment g1_23
   The launcher starts MuJoCo, CloudXR, and the XR bridge automatically,
   with gravity compensation and camera streaming enabled.
   Do not start the three legacy DDS scripts alongside this launcher.

4. Wait for the launcher to report readiness, then connect your headset:
     https://nvidia.github.io/IsaacTeleop/client
   Enter WORKSTATION_IP. If prompted, accept the workstation certificate:
     https://WORKSTATION_IP:48322/
   Squeeze each controller to engage its arm. Ctrl+C stops the session.

Without an X display, use headset-only mode:
  ./run_g1_vr_sim.sh --embodiment g1_29 --headless --live-xr
  Add --accept-cloudxr-eula on first use only if you accept the EULA.
  --headless alone runs synthetic replay, NOT live headset control.

Ensure headset network/firewall access is configured. No firewall or
driver settings were changed. Logs: outputs/vr-sim-*
Full guide: docs/coworker-installation.md
======================================================================
""")


def run(*args, **kwargs):
    print("+", *map(str, args), flush=True)
    subprocess.run(list(map(str, args)), check=True, **kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prefix", type=Path, default=ROOT / ".vr-sim")
    parser.add_argument("--conda", default=os.environ.get("CONDA_EXE", "conda"))
    args = parser.parse_args()
    if platform.system() != "Linux" or platform.machine() != "x86_64":
        parser.error("This CloudXR setup targets Linux x86_64 with an NVIDIA GPU.")
    if not shutil.which(args.conda):
        parser.error("Install Miniforge/conda first, or supply --conda /path/to/conda.")
    if not shutil.which("git"):
        parser.error("git is required.")
    prefix = args.prefix.resolve()
    prefix.mkdir(parents=True, exist_ok=True)
    checkout, env, assets = prefix / "lerobot", prefix / "env", prefix / "assets"
    if not checkout.exists():
        run("git", "clone", "--branch", BRANCH, REPOSITORY, checkout)
        run("git", "-C", checkout, "checkout", "--detach", COMMIT)
    head = subprocess.check_output(
        ["git", "-C", str(checkout), "rev-parse", "HEAD"], text=True
    ).strip()
    dirty = subprocess.check_output(
        ["git", "-C", str(checkout), "status", "--porcelain"], text=True
    ).strip()
    if head != COMMIT or dirty:
        parser.error(
            f"Preserving existing checkout {checkout}: expected clean {COMMIT}; use a new --prefix."
        )
    if not env.exists():
        run(
            args.conda,
            "create",
            "--yes",
            "--prefix",
            env,
            "--override-channels",
            "-c",
            "conda-forge",
            "--strict-channel-priority",
            "python=3.12",
            "pinocchio=3.9.0",
            "casadi=3.7.2",
            "numpy=2.2",
            "pip",
            "tk",
        )
    python = env / "bin/python"
    if not python.is_file():
        parser.error(
            f"Incomplete environment at {env}; use another --prefix or repair it explicitly."
        )
    # Do not request LeRobot's kinematics extra: it installs a different Pinocchio ABI.
    run(
        python,
        "-m",
        "pip",
        "install",
        "-e",
        str(checkout),
        "mujoco==3.12.0",
        "pyzmq>=26,<28",
        "pytest",
        "meshcat",
        "torch==2.11.0+cu128",
        "torchvision==0.26.0+cu128",
        "numpy>=2.0,<2.3",
        "isaacteleop[cloudxr,retargeters-lite]==1.3.132rc1",
        "--extra-index-url",
        "https://pypi.nvidia.com",
        "--extra-index-url",
        "https://download.pytorch.org/whl/cu128",
    )
    environment = dict(os.environ, PYTHONPATH=str(checkout / "src"), MUJOCO_GL="egl")
    run(python, ROOT / "tools/verify_vr_graphics.py", env=environment)
    run(
        python,
        "-c",
        "import pinocchio; from pinocchio import casadi; import mujoco, tkinter, zmq; "
        "from lerobot.robots.unitree_g1 import UnitreeG1; "
        "from isaacteleop.cloudxr import CloudXRLauncher; "
        "from lerobot.teleoperators.xr_controllers.video_session import VideoControllerSession; "
        "assert pinocchio.__version__ == '3.9.0'; print('Runtime imports passed')",
        env=environment,
    )
    run(
        python,
        checkout / "examples/unitree_g1/prepare_cartesian_assets.py",
        "--output",
        assets,
        "--meshes",
        env=environment,
    )
    for embodiment in ("g1_29", "g1_23"):
        run(
            python,
            checkout / "examples/unitree_g1/run_xr_simulation.py",
            "--assets",
            assets,
            "--embodiment",
            embodiment,
            "--headless",
            "--steps",
            "100",
            env=environment,
        )
    config = {
        "python": str(python),
        "lerobot": str(checkout),
        "assets": str(assets),
        "commit": COMMIT,
    }
    target = ROOT / ".vr-sim.json"
    temporary = target.with_suffix(".tmp")
    temporary.write_text(json.dumps(config, indent=2) + "\n")
    temporary.replace(target)
    with (prefix / "pip-freeze.txt").open("w") as stream:
        run(python, "-m", "pip", "freeze", stdout=stream)
    print_next_steps(ROOT)


if __name__ == "__main__":
    main()
