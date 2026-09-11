#!/usr/bin/env python3
"""Clone the fork, test feature branches, then merge and retest each milestone.

No physical discovery, X windows, headset sessions, force pushes, or checkout
deletion. The destination must not exist. Reports survive a failed verification.
GPU verification is offscreen Vulkan/CUDA, not an OpenXR session.
"""

import argparse
import json
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path


BRANCHES = [
    "g1/embodiments",
    "g1/cartesian-control",
    "g1/simulation",
    "g1/xr",
    "g1/xr-video",
    "g1/hand-support",
    "g1/brainco-hands",
]
BASE = "b6ec0060779550c0a157ae34feb89e0cf86012a8"
SUITES = {
    1: [
        "tests/robots/test_unitree_g1.py",
        "tests/robots/test_unitree_g1_utils.py",
        "tests/robots/test_unitree_g1_embodiments.py",
        "tests/teleoperators/test_unitree_g1_teleoperator.py",
        "tests/robots/test_sonic_whole_body.py",
    ],
    2: [
        "tests/robots/test_unitree_g1_cartesian_control.py",
        "tests/robots/test_unitree_g1_kinematics.py",
    ],
    3: [
        "tests/robots/test_unitree_g1_simulation.py",
        "tests/integration/test_unitree_g1_mujoco_runtime.py",
    ],
    4: ["tests/teleoperators/test_unitree_g1_xr.py"],
    5: ["tests/teleoperators/test_unitree_g1_xr_video.py"],
    6: ["tests/robots/test_unitree_g1_hands.py"],
    7: ["tests/robots/test_unitree_g1_brainco_hands.py"],
}
LOCAL = {
    1: [1],
    2: [1, 2],
    3: [1, 2, 3],
    4: [1, 2, 4],
    5: [1, 2, 4, 5],
    6: [1, 2, 6],
    7: [1, 2, 6, 7],
}


class Verification:
    def __init__(self, args):
        self.args = args
        self.checkout = args.checkout.resolve()
        self.output = args.output.resolve()
        self.output.mkdir(parents=True, exist_ok=False)
        self.report = {
            "status": "running",
            "scope": "headless_nonphysical",
            "checkout": str(self.checkout),
            "remote": args.remote,
            "base": args.base,
            "commands": [],
            "stages": [],
            "gpu_requested": args.gpu,
            "manual_X_headset_hardware": "not_run",
        }
        self.env = dict(
            os.environ,
            PYTHONPATH=str(self.checkout / "src"),
            MUJOCO_GL="egl",
            G1_KINEMATICS_ASSETS=str(args.assets.resolve()),
            G1_RENDER_TESTS="1",
            G1_BRAINCO_SDK_TESTS="1",
            HF_LEROBOT_HOME=str(self.output / "cache"),
        )
        self.env.pop("G1_VIDEO_GPU_TESTS", None)
        self.env.pop("G1_INTEGRATION_TESTS", None)
        self.sdk_env = dict(self.env)
        if args.sdk_site_packages:
            self.sdk_env["PYTHONPATH"] += os.pathsep + str(
                args.sdk_site_packages.resolve()
            )

    def save(self):
        (self.output / "report.json").write_text(
            json.dumps(self.report, indent=2) + "\n"
        )

    def run(self, command, *, cwd=None, env=None):
        number = len(self.report["commands"])
        logfile = self.output / f"command-{number:03d}.log"
        start = time.monotonic()
        result = subprocess.run(
            [str(x) for x in command],
            cwd=cwd or self.checkout,
            env=env or self.env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=600,
        )
        logfile.write_text(result.stdout)
        self.report["commands"].append(
            {
                "argv": [str(x) for x in command],
                "cwd": str(cwd or self.checkout),
                "log": logfile.name,
                "exit_code": result.returncode,
                "seconds": round(time.monotonic() - start, 3),
            }
        )
        self.save()
        if result.returncode:
            raise RuntimeError(f"Command failed; {logfile}\n{result.stdout[-6000:]}")
        return result.stdout.strip()

    def junit(self, name, paths, *, env=None, extra=()):
        xml = self.output / f"{name}.xml"
        self.run(
            [
                self.args.python,
                "-m",
                "pytest",
                *paths,
                "-q",
                f"--junitxml={xml}",
                *extra,
            ],
            env=env,
        )
        cases = list(ET.parse(xml).getroot().iter("testcase"))
        skipped = [case for case in cases if case.find("skipped") is not None]
        unexpected = [
            case.get("classname", "")
            for case in skipped
            if "test_sonic_whole_body" not in case.get("classname", "")
            and case.get("name") != "tests.robots.test_sonic_whole_body"
        ]
        if unexpected or not cases:
            raise RuntimeError(
                f"Required tests were skipped or not collected: {unexpected}"
            )
        return {
            "passed": len(cases) - len(skipped),
            "optional_skipped": len(skipped),
            "junit": xml.name,
        }

    def verify(self, name, milestones, *, combined=False):
        print(f"Verifying {name}: milestones {milestones}", flush=True)
        commit = self.run(["git", "rev-parse", "HEAD"])
        changed = self.run(
            [
                "git",
                "diff",
                "--name-only",
                "--diff-filter=ACM",
                self.args.base,
                "HEAD",
                "--",
                "*.py",
            ]
        ).splitlines()
        if changed:
            self.run([self.args.python, "-m", "ruff", "check", *changed])
            self.run([self.args.python, "-m", "ruff", "format", "--check", *changed])
        if 5 in milestones:
            self.run(
                [
                    self.args.python,
                    "-m",
                    "mypy",
                    "--follow-imports=silent",
                    "src/lerobot/cameras/frame_channel.py",
                ]
            )
        self.run(
            [
                self.args.python,
                "-c",
                "from pathlib import Path; import lerobot.robots.unitree_g1 as m; "
                f"assert Path(m.__file__).resolve().is_relative_to(Path({str(self.checkout / 'src')!r})); print(m.__file__)",
            ]
        )
        env = dict(self.env)
        paths = [path for milestone in milestones for path in SUITES[milestone]]
        if combined and 4 in milestones:
            env["G1_INTEGRATION_TESTS"] = "1"
            paths.append("tests/integration/test_unitree_g1_xr_mujoco.py")
        if combined and 5 in milestones:
            paths.append("tests/integration/test_unitree_g1_xr_video_mujoco.py")
        counts = self.junit(
            name,
            paths,
            env=env,
            extra=["-k", "not real_offscreen_delivery_and_recovery"],
        )
        if 4 in milestones:
            self.run(
                [
                    self.args.python,
                    "-c",
                    "from lerobot.teleoperators.xr_controllers.xr_controllers import IsaacControllerSession; "
                    "from lerobot.teleoperators.xr_controllers import XRControllersConfig; "
                    "s=IsaacControllerSession(XRControllersConfig()); assert set(s.pipeline.output_types()) == {'left', 'right'}; print('SDK pipeline verified without OpenXR connection')",
                ],
                env=self.sdk_env,
            )
        gpu = None
        if self.args.gpu and 5 in milestones:
            gpu_env = dict(self.sdk_env, G1_VIDEO_GPU_TESTS="1")
            gpu = self.junit(
                f"{name}-gpu",
                SUITES[5],
                env=gpu_env,
                extra=["-k", "real_offscreen_delivery_and_recovery"],
            )
        self.report["stages"].append(
            {
                "name": name,
                "commit": commit,
                "milestones": milestones,
                **counts,
                "gpu": gpu,
            }
        )
        self.save()
        print(
            f"  {counts['passed']} passed, {counts['optional_skipped']} optional skips; {commit[:12]}",
            flush=True,
        )

    def examples(self):
        print("Verifying headless launchers for both embodiments", flush=True)
        self.run(
            [
                self.args.python,
                "examples/unitree_g1/verify_cartesian_control.py",
                "--assets",
                self.args.assets,
                "--headless",
                "--samples-per-phase",
                "16",
                "--report",
                self.output / "cartesian.json",
            ]
        )
        for embodiment in ("g1_29", "g1_23"):
            for no_gravity in (False, True):
                command = [
                    self.args.python,
                    "examples/unitree_g1/run_simulation.py",
                    "--assets",
                    self.args.assets,
                    "--embodiment",
                    embodiment,
                    "--headless",
                ]
                if no_gravity:
                    command.append("--no-gravity-compensation")
                self.run(command)
            self.run(
                [
                    self.args.python,
                    "examples/unitree_g1/run_xr_simulation.py",
                    "--assets",
                    self.args.assets,
                    "--embodiment",
                    embodiment,
                    "--headless",
                    "--steps",
                    "100",
                    "--channel",
                    self.output / f"{embodiment}.rgb",
                ]
            )

    def execute(self):
        if self.checkout.exists():
            raise ValueError(
                "Checkout must not exist; this tool never resets/deletes an existing checkout"
            )
        self.checkout.parent.mkdir(parents=True, exist_ok=True)
        self.run(
            ["git", "clone", self.args.remote, self.checkout], cwd=self.checkout.parent
        )
        self.report["branch_commits"] = {
            branch: self.run(["git", "rev-parse", f"origin/{branch}"])
            for branch in BRANCHES
        }
        self.report["environment"] = self.run(
            [
                self.args.python,
                "-c",
                "import sys, pinocchio, casadi, mujoco; "
                "print(sys.version); print('pinocchio',pinocchio.__version__,'casadi',casadi.__version__,'mujoco',mujoco.__version__)",
            ]
        )
        for step, branch in enumerate(BRANCHES, 1):
            self.run(["git", "switch", "--detach", f"origin/{branch}"])
            self.verify(f"branch-{step}", LOCAL[step])
        self.run(["git", "switch", "-c", self.args.integration_branch, self.args.base])
        for step, branch in enumerate(BRANCHES, 1):
            self.run(["git", "merge", "--no-ff", "--no-edit", f"origin/{branch}"])
            self.verify(f"merge-{step}", list(range(1, step + 1)), combined=True)
        self.examples()
        dirty = self.run(["git", "status", "--porcelain"])
        if dirty:
            raise RuntimeError(f"Fresh checkout became dirty: {dirty}")
        self.report["final_commit"] = self.run(["git", "rev-parse", "HEAD"])
        self.report["status"] = "passed" if self.args.gpu else "passed_without_gpu_gate"
        self.save()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkout", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--assets", type=Path, required=True)
    parser.add_argument("--python", type=Path, default=Path(sys.executable))
    parser.add_argument("--sdk-site-packages", type=Path)
    parser.add_argument(
        "--gpu", action="store_true", help="Require headless Vulkan/CUDA pixel checks"
    )
    parser.add_argument("--remote", default="git@github.com:MoissanClub/lerobot.git")
    parser.add_argument("--base", default=BASE)
    parser.add_argument("--integration-branch", default="integration/g1-acceptance")
    args = parser.parse_args()
    args.assets = args.assets.resolve()
    verification = Verification(args)
    try:
        verification.execute()
    except BaseException as exc:
        verification.report["status"] = "failed"
        verification.report["error"] = str(exc)
        verification.save()
        raise
    print(f"Report: {verification.output / 'report.json'}", flush=True)


if __name__ == "__main__":
    main()
