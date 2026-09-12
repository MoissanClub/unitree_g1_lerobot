"""Install a private, pinned LeRobot/Isaac Teleop environment without changing DDS setup."""

import argparse
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
COMMIT = "d5e400bcefeccc93ba956ce876530e5283512df1"
BRANCH = "integration/g1-acceptance"
REPOSITORY = "https://github.com/MoissanClub/lerobot.git"


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
    print(
        "Installed and verified both embodiments. Run ./run_g1_vr_sim.sh --embodiment g1_29"
    )
    print(
        "For first live use, review CloudXR's EULA; pass --accept-cloudxr-eula only if you accept."
    )
    print(
        "NVIDIA driver/Vulkan support and network/firewall access must be configured by the operator."
    )


if __name__ == "__main__":
    main()
