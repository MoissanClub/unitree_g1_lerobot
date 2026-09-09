#!/usr/bin/env python3
"""Bounded startup diagnostics for the G1 MuJoCo/XR launch scripts.

Each mode publishes a short IK-generated arm command sequence to an already-running
LeRobot G1 MuJoCo DDS simulator. The messages are intentionally user-visible so the
operator can confirm that the right process is connected to the sim before wearing the
headset.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import pinocchio as pin

from unitree_g1_lerobot.robots.control import (
    action_from_arm_q,
    make_ready_targets,
    solve_ready_q,
)

from unitree_g1_lerobot.simulation.dds import connect_unitree_g1_external_dds, patch_unitree_dds_config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("raise", "orient", "hands-up", "lower"))
    parser.add_argument("--lerobot-root", default="/home/dwei/lerobot-sim/lerobot")
    parser.add_argument("--duration-s", type=float, default=2.0)
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--g1-state-timeout-s", type=float, default=10.0)
    parser.add_argument("--ready-x-m", type=float, default=0.03)
    parser.add_argument("--ready-z-m", type=float, default=0.12)
    parser.add_argument("--ready-spread-m", type=float, default=0.03)
    parser.add_argument("--orientation-deg", type=float, default=35.0)
    parser.add_argument("--hands-up-deg", type=float, default=90.0)
    parser.add_argument(
        "--hands-up-direction",
        choices=("inward", "outward"),
        default="inward",
        help="Wrist-roll direction for the hands-up diagnostic.",
    )
    parser.add_argument("--optional", action="store_true", help="Warn and continue if the DDS sim is not available.")
    parser.add_argument("--confirm", action="store_true", help="Wait for the user to confirm the observed motion before exiting.")
    return parser.parse_args()


def rot_y(theta: float) -> np.ndarray:
    c = float(np.cos(theta))
    s = float(np.sin(theta))
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)


def send_for(robot, action: dict[str, float], duration_s: float, hz: float) -> None:
    period_s = 1.0 / hz
    deadline = time.monotonic() + max(0.0, duration_s)
    while time.monotonic() < deadline:
        t0 = time.perf_counter()
        robot.send_action(action)
        time.sleep(max(0.0, period_s - (time.perf_counter() - t0)))


def build_actions(args: argparse.Namespace, ik, joint_arm_index, joint_index) -> dict[str, dict[str, float]]:
    model = ik.reduced_robot.model
    data = model.createData()
    ik.reduced_robot.data = data
    q0 = np.zeros(model.nq, dtype=float)
    pin.forwardKinematics(model, data, q0)
    pin.updateFramePlacements(model, data)
    left_home = data.oMf[ik.L_hand_id].homogeneous.copy()
    right_home = data.oMf[ik.R_hand_id].homogeneous.copy()
    left_ready, right_ready = make_ready_targets(
        left_home,
        right_home,
        args.ready_x_m,
        args.ready_z_m,
        args.ready_spread_m,
    )
    reorder = np.asarray(ik._arm_reorder_pin_to_g1)

    q_home, _ = ik.solve_ik(left_home, right_home, q0)
    q_home = np.asarray(q_home, dtype=float)
    q_ready = solve_ready_q(ik, left_ready, right_ready, q_home)

    right_oriented = right_ready.copy()
    right_oriented[:3, :3] = right_oriented[:3, :3] @ rot_y(np.deg2rad(args.orientation_deg))
    q_orient = solve_ready_q(ik, left_ready, right_oriented, q_ready)

    q_ready_g1 = q_ready[reorder].copy()
    q_hands_up_g1 = q_ready_g1.copy()
    wrist_roll_rad = np.deg2rad(args.hands_up_deg)
    if args.hands_up_direction == "inward":
        left_wrist_roll = -wrist_roll_rad
        right_wrist_roll = wrist_roll_rad
    else:
        left_wrist_roll = wrist_roll_rad
        right_wrist_roll = -wrist_roll_rad
    q_hands_up_g1[4] = left_wrist_roll
    q_hands_up_g1[11] = right_wrist_roll

    return {
        "lower": action_from_arm_q(q_home[reorder], joint_index, joint_arm_index),
        "raise": action_from_arm_q(q_ready_g1, joint_index, joint_arm_index),
        "orient": action_from_arm_q(q_orient[reorder], joint_index, joint_arm_index),
        "hands-up": action_from_arm_q(q_hands_up_g1, joint_index, joint_arm_index),
    }


def run_diagnostic(args: argparse.Namespace) -> int:
    lerobot_root = Path(args.lerobot_root).expanduser().resolve()
    if not lerobot_root.is_dir():
        print(f"LeRobot checkout not found: {lerobot_root}", file=sys.stderr)
        return 2
    sys.path.insert(0, str(lerobot_root))

    from lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config
    from lerobot.robots.unitree_g1 import unitree_g1 as g1_module
    from lerobot.robots.unitree_g1.g1_kinematics import G1_29_ArmIK
    from lerobot.robots.unitree_g1.g1_utils import G1_29_JointArmIndex, G1_29_JointIndex

    messages = {
        "raise": "raising robot arm - verify the G1 arms move up in the MuJoCo viewer",
        "orient": "simulating CloudXR input - verify the right arm orientation changes",
        "hands-up": "simulating CloudXR input - verify both hands face up",
        "lower": "simulating XR device input - verify both robot arms raise briefly, then lower",
    }
    confirmation_prompts = {
        "raise": "Press Enter after you verify the diagnostic motion of arm raising, or Ctrl+C to abort...",
        "orient": "Press Enter after you verify the diagnostic motion of right-arm orientation change, or Ctrl+C to abort...",
        "hands-up": "Press Enter after you verify the diagnostic motion of both hands facing up, or Ctrl+C to abort...",
        "lower": "Press Enter after you verify the diagnostic motion of both arms lowering, or Ctrl+C to abort...",
    }
    print(f"== Startup diagnostic: {messages[args.mode]} ==", flush=True)

    patch_unitree_dds_config()
    robot = UnitreeG1(UnitreeG1Config(is_simulation=True))
    try:
        connect_unitree_g1_external_dds(robot, g1_module, G1_29_JointIndex, args.g1_state_timeout_s)
    except TimeoutError as exc:
        if args.optional:
            print(f"Startup diagnostic skipped: {exc}", file=sys.stderr, flush=True)
            return 0
        print(str(exc), file=sys.stderr, flush=True)
        return 2

    try:
        actions = build_actions(args, G1_29_ArmIK(), G1_29_JointArmIndex, G1_29_JointIndex)
        if args.mode in {"orient", "hands-up"}:
            send_for(robot, actions["raise"], min(0.8, args.duration_s), args.hz)
        elif args.mode == "lower":
            send_for(robot, actions["raise"], min(1.0, args.duration_s), args.hz)
        send_for(robot, actions[args.mode], args.duration_s, args.hz)
        print("== Startup diagnostic complete ==", flush=True)
        if args.confirm:
            try:
                input(confirmation_prompts[args.mode])
            except EOFError:
                print("Startup diagnostic confirmation could not read from stdin; aborting.", file=sys.stderr, flush=True)
                return 2
            print("== User verification confirmed; entering steady-state listening ==", flush=True)
    finally:
        robot.disconnect()
    return 0


def main() -> int:
    return run_diagnostic(parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
