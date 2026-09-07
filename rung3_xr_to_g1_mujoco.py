#!/usr/bin/env python3
"""Rung 3 - drive the G1 MuJoCo sim from an Isaac Teleop XR controller.

This joins rung 1' (CloudXR -> XRController.get_action) with rung 2b
(G1_29_ArmIK -> UnitreeG1(is_simulation=True)).

Default behavior is intentionally conservative: one controller drives one wrist while the
other wrist holds the IK home pose. Hold the controller squeeze past the clutch threshold
to move; release squeeze to freeze the commanded wrist pose while repositioning.
"""

from __future__ import annotations

import argparse
import os
import shlex
import sys
import time
from pathlib import Path

import numpy as np
import pinocchio as pin


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--lerobot-root",
        default="/home/dwei/lerobot-sim/lerobot",
        help="Path to the LeRobot checkout.",
    )
    parser.add_argument("--hand-side", choices=("left", "right"), default="right")
    parser.add_argument(
        "--external-cloudxr",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use a CloudXR runtime already started by ./run_isaac_teleop.sh.",
    )
    parser.add_argument(
        "--cloudxr-env-file",
        default=None,
        help="CloudXR env file when auto-launching. Defaults to examples/isaac_teleop_to_so101/default.env.",
    )
    parser.add_argument("--duration-s", type=float, default=120.0)
    parser.add_argument("--control-hz", type=float, default=30.0)
    parser.add_argument("--clutch-threshold", type=float, default=0.5)
    parser.add_argument(
        "--max-delta-m",
        type=float,
        default=0.20,
        help="Clamp commanded wrist translation to this radius from the IK home pose.",
    )
    parser.add_argument(
        "--no-wait",
        action="store_true",
        help="Do not pause before creating the OpenXR session.",
    )
    parser.add_argument(
        "--dry-run-ik",
        action="store_true",
        help="Build G1 IK, solve the home pose once, then exit without XR or MuJoCo.",
    )
    return parser.parse_args()


def load_env_file(path: Path) -> None:
    for raw_line in path.read_text().splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[len("export "):].strip()
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        os.environ[key.strip()] = os.path.expandvars(os.path.expanduser(shlex.split(value, comments=False)[0] if value else ""))


def make_transform(pos: np.ndarray, quat_xyzw: np.ndarray, rotation_cls: type) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation_cls.from_quat(np.asarray(quat_xyzw, dtype=float)).as_matrix()
    transform[:3, 3] = np.asarray(pos, dtype=float)
    return transform


def clamp_translation(target: np.ndarray, home: np.ndarray, max_delta_m: float) -> np.ndarray:
    if max_delta_m <= 0.0:
        return target
    clamped = target.copy()
    delta = clamped[:3, 3] - home[:3, 3]
    norm = float(np.linalg.norm(delta))
    if norm > max_delta_m:
        clamped[:3, 3] = home[:3, 3] + delta * (max_delta_m / norm)
    return clamped


def fk(model: pin.Model, data: pin.Data, left_frame_id: int, right_frame_id: int, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    return data.oMf[left_frame_id].homogeneous.copy(), data.oMf[right_frame_id].homogeneous.copy()


def action_from_arm_q(q_g1: np.ndarray, joint_index: type, arm_index: type) -> dict[str, float]:
    action = {f"{joint.name}.q": 0.0 for joint in joint_index}
    action.update({f"{joint.name}.q": float(q_g1[i]) for i, joint in enumerate(arm_index)})
    return action


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(line_buffering=True)
    args = parse_args()

    lerobot_root = Path(args.lerobot_root).expanduser().resolve()
    if not lerobot_root.is_dir():
        print(f"LeRobot checkout not found: {lerobot_root}", file=sys.stderr)
        return 2
    sys.path.insert(0, str(lerobot_root))

    from examples.isaac_teleop_to_so101.isaac_teleop import XRController, XRControllerConfig
    from examples.isaac_teleop_to_so101.isaac_teleop.clutch import Clutch
    from lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config
    from lerobot.robots.unitree_g1.g1_kinematics import G1_29_ArmIK
    from lerobot.robots.unitree_g1.g1_utils import G1_29_JointArmIndex, G1_29_JointIndex
    from lerobot.utils.rotation import Rotation

    if args.external_cloudxr:
        os.environ["LEROBOT_CLOUDXR_SKIP_AUTOLAUNCH"] = "1"
        runtime_env = Path.home() / ".cloudxr" / "run" / "cloudxr.env"
        if runtime_env.is_file():
            load_env_file(runtime_env)
        else:
            print(f"External CloudXR requested, but env file is missing: {runtime_env}", file=sys.stderr)
            print("Start CloudXR first with ./run_isaac_teleop.sh", file=sys.stderr)
            return 2

    cloudxr_env_file = args.cloudxr_env_file
    if cloudxr_env_file is None and not args.external_cloudxr:
        default_env = lerobot_root / "examples" / "isaac_teleop_to_so101" / "default.env"
        cloudxr_env_file = str(default_env) if default_env.is_file() else None

    print("Building G1 IK")
    ik = G1_29_ArmIK()
    model = ik.reduced_robot.model
    data = model.createData()
    ik.reduced_robot.data = data

    q = np.zeros(model.nq, dtype=float)
    left_home, right_home = fk(model, data, ik.L_hand_id, ik.R_hand_id, q)
    left_target = left_home.copy()
    right_target = right_home.copy()
    active_home = right_home if args.hand_side == "right" else left_home
    clutch = Clutch(active_home)
    reorder = np.asarray(ik._arm_reorder_pin_to_g1)

    q_home, _ = ik.solve_ik(left_home, right_home, q)
    if not np.all(np.isfinite(q_home)):
        print("IK home solve produced non-finite values.", file=sys.stderr)
        return 1
    q = np.asarray(q_home, dtype=float)
    if args.dry_run_ik:
        print("dry-run IK ok")
        print(f"  q_dim: {q.size}")
        print(f"  first_arm_joint_g1: {float(q[reorder][0]):+.4f}")
        return 0

    print("Connecting UnitreeG1 MuJoCo sim")
    robot = UnitreeG1(UnitreeG1Config(is_simulation=True))
    robot.connect()
    print(f"robot connected: {robot.is_connected}")

    teleop = XRController(
        XRControllerConfig(
            hand_side=args.hand_side,
            clutch_threshold=args.clutch_threshold,
            auto_launch_cloudxr=not args.external_cloudxr,
            cloudxr_env_file=cloudxr_env_file,
        )
    )

    print()
    print("Headset browser:")
    print("  1. Open https://nvidia.github.io/IsaacTeleop/client")
    print("  2. Set/leave headset profile as Quest3")
    print("  3. Enter the workstation IP printed by ./run_isaac_teleop.sh")
    print("  4. Enter XR and connect")
    print()
    print("Controls:")
    print(f"  Move the {args.hand_side} controller while holding squeeze > {args.clutch_threshold:.2f}.")
    print("  Release squeeze to freeze the G1 wrist while repositioning your hand.")
    print()
    if not args.no_wait:
        input("After the headset client is connected, press Enter to create the OpenXR session...")

    teleop.connect()
    print("XR teleop connected. Waiting for tracked controller frames...")

    period_s = 1.0 / args.control_hz
    deadline = time.monotonic() + args.duration_s
    was_engaged = False
    last_tracking: bool | None = None
    step = 0

    try:
        while time.monotonic() < deadline:
            t0 = time.perf_counter()
            xr_action = teleop.get_action()
            tracking = bool(teleop.is_tracking)
            squeeze = float(xr_action["squeeze"])
            engaged = tracking and squeeze >= args.clutch_threshold

            if engaged and not was_engaged:
                measured_left, measured_right = fk(model, data, ik.L_hand_id, ik.R_hand_id, q)
                measured_active = measured_right if args.hand_side == "right" else measured_left
                clutch.engage(xr_action["grip_pos"], xr_action["grip_quat"], measured_active)
                print(f"clutch engaged at step {step}")

            if engaged:
                pos, quat = clutch.rebase(xr_action["grip_pos"], xr_action["grip_quat"])
                active_target = make_transform(pos, quat, Rotation)
                active_target = clamp_translation(active_target, active_home, args.max_delta_m)
                if args.hand_side == "right":
                    right_target = active_target
                else:
                    left_target = active_target

                q_next, _ = ik.solve_ik(left_target, right_target, q)
                q_next = np.asarray(q_next, dtype=float)
                if np.all(np.isfinite(q_next)):
                    q = q_next
                else:
                    print(f"non-finite IK result at step {step}; holding previous command", file=sys.stderr)

            q_g1 = q[reorder]
            robot.send_action(action_from_arm_q(q_g1, G1_29_JointIndex, G1_29_JointArmIndex))

            if tracking or tracking != last_tracking or step % max(1, int(args.control_hz)) == 0:
                active_target = right_target if args.hand_side == "right" else left_target
                print(
                    "step={step:04d} tracking={tracking} engaged={engaged} squeeze={squeeze:.3f} "
                    "target=({x:+.3f},{y:+.3f},{z:+.3f}) q0={q0:+.3f}".format(
                        step=step,
                        tracking=tracking,
                        engaged=engaged,
                        squeeze=squeeze,
                        x=float(active_target[0, 3]),
                        y=float(active_target[1, 3]),
                        z=float(active_target[2, 3]),
                        q0=float(q_g1[0]),
                    )
                )

            was_engaged = engaged
            last_tracking = tracking
            step += 1
            time.sleep(max(0.0, period_s - (time.perf_counter() - t0)))
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        teleop.disconnect()
        robot.disconnect()
        print("disconnected")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
