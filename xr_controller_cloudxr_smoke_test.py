#!/usr/bin/env python3
"""Robot-free CloudXR/XR controller smoke test for LeRobot Isaac Teleop.

Run this to verify that the headset can connect to CloudXR and stream controller
state before involving SO-101 or G1 robot code.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--lerobot-root",
        default="/home/dwei/lerobot-sim/lerobot",
        help="Path to the LeRobot source checkout containing examples/isaac_teleop_to_so101.",
    )
    parser.add_argument("--hand-side", choices=("left", "right"), default="right")
    parser.add_argument(
        "--external-cloudxr",
        action="store_true",
        help="Attach to a CloudXR runtime already started with python -m isaacteleop.cloudxr.",
    )
    parser.add_argument(
        "--cloudxr-env-file",
        default=None,
        help="Optional CloudXR input .env. Defaults to LeRobot example default.env when auto-launching.",
    )
    parser.add_argument("--duration-s", type=float, default=120.0)
    parser.add_argument("--poll-hz", type=float, default=10.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    lerobot_root = Path(args.lerobot_root).expanduser().resolve()
    if not lerobot_root.is_dir():
        print(f"LeRobot checkout not found: {lerobot_root}", file=sys.stderr)
        return 2

    sys.path.insert(0, str(lerobot_root))

    from examples.isaac_teleop_to_so101.isaac_teleop import XRController, XRControllerConfig

    if args.external_cloudxr:
        os.environ["LEROBOT_CLOUDXR_SKIP_AUTOLAUNCH"] = "1"

    cloudxr_env_file = args.cloudxr_env_file
    if cloudxr_env_file is None and not args.external_cloudxr:
        default_env = lerobot_root / "examples" / "isaac_teleop_to_so101" / "default.env"
        cloudxr_env_file = str(default_env) if default_env.is_file() else None

    teleop = XRController(
        XRControllerConfig(
            hand_side=args.hand_side,
            auto_launch_cloudxr=not args.external_cloudxr,
            cloudxr_env_file=cloudxr_env_file,
        )
    )

    print("Starting XR controller smoke test")
    print(f"  LeRobot root:       {lerobot_root}")
    print(f"  hand_side:          {args.hand_side}")
    print(f"  external_cloudxr:   {args.external_cloudxr}")
    print(f"  cloudxr_env_file:   {cloudxr_env_file}")
    print()
    print("Headset browser:")
    print("  1. Open https://nvidia.github.io/IsaacTeleop/client")
    print("  2. Enter the workstation IP, e.g. 10.9.0.149")
    print("  3. Accept https://<workstation-ip>:48322/ if prompted")
    print("  4. Enter XR and connect")
    print()

    period_s = 1.0 / args.poll_hz
    deadline = time.monotonic() + args.duration_s
    last_tracking: bool | None = None

    try:
        teleop.connect()
        print("Teleop session connected. Waiting for controller frames...")
        while time.monotonic() < deadline:
            action = teleop.get_action()
            tracking = bool(teleop.is_tracking)
            if tracking or tracking != last_tracking:
                pos = action["grip_pos"]
                quat = action["grip_quat"]
                print(
                    "tracking={tracking} "
                    "pos=({px:+.3f},{py:+.3f},{pz:+.3f}) "
                    "quat=({qx:+.3f},{qy:+.3f},{qz:+.3f},{qw:+.3f}) "
                    "squeeze={squeeze:.3f} trigger={trigger:.3f}".format(
                        tracking=tracking,
                        px=float(pos[0]),
                        py=float(pos[1]),
                        pz=float(pos[2]),
                        qx=float(quat[0]),
                        qy=float(quat[1]),
                        qz=float(quat[2]),
                        qw=float(quat[3]),
                        squeeze=float(action["squeeze"]),
                        trigger=float(action["trigger"]),
                    ),
                    flush=True,
                )
            last_tracking = tracking
            time.sleep(period_s)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        teleop.disconnect()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
