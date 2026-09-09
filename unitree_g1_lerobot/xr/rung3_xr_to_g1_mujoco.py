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


from unitree_g1_lerobot.robots.control import (
    fk,
    action_from_arm_q,
    scale_arm_gains,
    make_ready_targets,
    solve_ready_q,
    publish_ready_for,
)

from unitree_g1_lerobot.simulation.dds import (
    patch_unitree_dds_config,
    connect_unitree_g1_external_dds,
)

from unitree_g1_lerobot.diagnostics.requests import (
    StartupDiagnosticRequests,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--lerobot-root",
        default="/home/dwei/lerobot-sim/lerobot",
        help="Path to the LeRobot checkout.",
    )
    parser.add_argument(
        "--isaacteleop-site-packages",
        default="/home/dwei/.venvs/isaacteleop/lib/python3.12/site-packages",
        help="Existing Isaac Teleop venv site-packages path to append when running in lerobot-g1.",
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
    parser.add_argument("--duration-s", type=float, default=0.0, help="Run duration in seconds; <= 0 runs until Ctrl+C.")
    parser.add_argument("--control-hz", type=float, default=30.0)
    parser.add_argument("--arm-kp-scale", type=float, default=1.0, help="Scale G1 arm position gains for MuJoCo responsiveness.")
    parser.add_argument("--arm-kd-scale", type=float, default=1.0, help="Scale G1 arm damping gains; raise with kp if the sim overshoots.")
    parser.add_argument("--clutch-threshold", type=float, default=0.5)
    parser.add_argument(
        "--clutch-axis",
        choices=("squeeze", "trigger", "max"),
        default="max",
        help="Controller analog input used for hold-to-enable clutch; max uses max(squeeze, trigger).",
    )
    parser.add_argument(
        "--max-delta-m",
        type=float,
        default=0.20,
        help="Clamp commanded wrist translation to this radius from the IK home pose.",
    )
    parser.add_argument(
        "--xr-pos-scale",
        type=float,
        default=1.0,
        help="Scale controller translation before applying it to the G1 wrist target.",
    )
    parser.add_argument(
        "--debug-xr",
        action="store_true",
        help="Print raw controller position, controller delta, target delta, and IK joint delta.",
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
    parser.add_argument(
        "--mock-xr",
        action="store_true",
        help="Use a deterministic fake XR controller; validates rung 3 without headset/CloudXR.",
    )
    parser.add_argument(
        "--external-g1-sim",
        action="store_true",
        help="Attach to an already-running G1 MuJoCo DDS sim instead of starting one in this process.",
    )
    parser.add_argument(
        "--wait-for-cloudxr",
        action="store_true",
        help="Keep retrying XR connection so this bridge can be started before CloudXR/headset.",
    )
    parser.add_argument("--cloudxr-retry-s", type=float, default=2.0)
    parser.add_argument("--g1-state-timeout-s", type=float, default=10.0)
    parser.add_argument(
        "--tracking-timeout-s",
        type=float,
        default=20.0,
        help="Reconnect OpenXR if no controller tracking arrives within this many seconds; <= 0 disables.",
    )
    parser.add_argument("--ready-x-m", type=float, default=0.03, help="Ready pose hand offset forward from IK home.")
    parser.add_argument("--ready-z-m", type=float, default=0.12, help="Ready pose hand offset upward from IK home.")
    parser.add_argument("--ready-spread-m", type=float, default=0.03, help="Ready pose outward lateral offset for each hand.")
    parser.add_argument(
        "--diagnostic-request-file",
        default="/tmp/g1_mujoco_startup_diagnostic.request",
        help="File used by launchers to request bridge-owned startup diagnostic poses.",
    )
    parser.add_argument(
        "--diagnostic-ack-file",
        default="/tmp/g1_mujoco_startup_diagnostic.ack",
        help="File written after the bridge observes a startup diagnostic request.",
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


def valid_pose_frame(pos: np.ndarray, quat: np.ndarray) -> bool:
    pos = np.asarray(pos, dtype=float)
    quat = np.asarray(quat, dtype=float)
    return (
        pos.shape == (3,)
        and quat.shape == (4,)
        and np.all(np.isfinite(pos))
        and np.all(np.isfinite(quat))
        and float(np.linalg.norm(quat)) > 1e-6
    )


def scaled_rebase(clutch, grip_pos: np.ndarray, grip_quat: np.ndarray, scale: float) -> tuple[np.ndarray, np.ndarray]:
    if scale == 1.0:
        return clutch.rebase(grip_pos, grip_quat)
    origin_pos = clutch._origin_pos.copy()
    grip_pos_scaled = origin_pos + (np.asarray(grip_pos, dtype=float) - origin_pos) * scale
    return clutch.rebase(grip_pos_scaled, grip_quat)


def clutch_value(action: dict, axis: str) -> float:
    squeeze = float(action.get("squeeze", 0.0))
    trigger = float(action.get("trigger", 0.0))
    if axis == "squeeze":
        return squeeze
    if axis == "trigger":
        return trigger
    return max(squeeze, trigger)


def clamp_translation(target: np.ndarray, home: np.ndarray, max_delta_m: float) -> np.ndarray:
    if max_delta_m <= 0.0:
        return target
    clamped = target.copy()
    delta = clamped[:3, 3] - home[:3, 3]
    norm = float(np.linalg.norm(delta))
    if norm > max_delta_m:
        clamped[:3, 3] = home[:3, 3] + delta * (max_delta_m / norm)
    return clamped


def patch_g1_hub_env_factory() -> None:
    import lerobot.envs.factory as env_factory

    original_call_make_env = env_factory._call_make_env

    def call_make_env_headless(module, n_envs, use_async_envs, cfg):
        module_file = str(getattr(module, "__file__", ""))
        if "models--lerobot--unitree-g1-mujoco" not in module_file:
            return original_call_make_env(module, n_envs, use_async_envs, cfg)

        original_safe_load = module.yaml.safe_load

        def safe_load_headless(stream):
            config = original_safe_load(stream)
            config["ENABLE_ONSCREEN"] = False
            config["ENABLE_OFFSCREEN"] = False
            return config

        module.yaml.safe_load = safe_load_headless
        try:
            return module.make_env(
                n_envs=n_envs,
                use_async_envs=use_async_envs,
                publish_images=False,
                cameras=[],
            )
        finally:
            module.yaml.safe_load = original_safe_load

    env_factory._call_make_env = call_make_env_headless


def cloudxr_runtime_error(exc: RuntimeError) -> bool:
    message = str(exc)
    return (
        "Failed to create OpenXR instance" in message
        or "XR_ERROR_RUNTIME_UNAVAILABLE" in message
        or "Failed to get OpenXR system" in message
    )


class MockXRController:
    def __init__(self, hand_side: str, clutch_threshold: float) -> None:
        self.hand_side = hand_side
        self.clutch_threshold = clutch_threshold
        self.is_tracking = True
        self.step = 0

    def connect(self) -> None:
        print("mock XR connected")

    def disconnect(self) -> None:
        print("mock XR disconnected")

    def get_action(self) -> dict[str, np.ndarray | float]:
        a = 2.0 * np.pi * self.step / 180.0
        self.step += 1
        return {
            "grip_pos": np.array(
                [0.03 * np.sin(a), 0.03 * (1.0 - np.cos(a)), 0.02 * np.sin(0.5 * a)],
                dtype=np.float32,
            ),
            "grip_quat": np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
            "squeeze": max(self.clutch_threshold, 0.8),
            "trigger": 0.0,
        }


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(line_buffering=True)
    args = parse_args()

    lerobot_root = Path(args.lerobot_root).expanduser().resolve()
    if not lerobot_root.is_dir():
        print(f"LeRobot checkout not found: {lerobot_root}", file=sys.stderr)
        return 2
    sys.path.insert(0, str(lerobot_root))
    isaacteleop_site = Path(args.isaacteleop_site_packages).expanduser()
    if isaacteleop_site.is_dir() and str(isaacteleop_site) not in sys.path:
        sys.path.append(str(isaacteleop_site))

    from examples.isaac_teleop_to_so101.isaac_teleop.clutch import Clutch
    from lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config
    from lerobot.robots.unitree_g1 import unitree_g1 as g1_module
    from lerobot.robots.unitree_g1.g1_kinematics import G1_29_ArmIK
    from lerobot.robots.unitree_g1.g1_utils import G1_29_JointArmIndex, G1_29_JointIndex
    from lerobot.utils.rotation import Rotation

    patch_unitree_dds_config()
    patch_g1_hub_env_factory()

    if args.external_cloudxr and not args.mock_xr:
        os.environ["LEROBOT_CLOUDXR_SKIP_AUTOLAUNCH"] = "1"
        runtime_env = Path.home() / ".cloudxr" / "run" / "cloudxr.env"
        if runtime_env.is_file():
            load_env_file(runtime_env)
        else:
            print(f"External CloudXR requested, but env file is missing: {runtime_env}", file=sys.stderr)
            print("Start CloudXR first with ./run_isaac_teleop.sh", file=sys.stderr)
            return 2

    cloudxr_env_file = args.cloudxr_env_file
    if cloudxr_env_file is None and not args.external_cloudxr and not args.mock_xr:
        default_env = lerobot_root / "examples" / "isaac_teleop_to_so101" / "default.env"
        cloudxr_env_file = str(default_env) if default_env.is_file() else None

    print("Building G1 IK")
    ik = G1_29_ArmIK()
    model = ik.reduced_robot.model
    data = model.createData()
    ik.reduced_robot.data = data

    q = np.zeros(model.nq, dtype=float)
    left_home, right_home = fk(model, data, ik.L_hand_id, ik.R_hand_id, q)
    left_ready, right_ready = make_ready_targets(
        left_home,
        right_home,
        args.ready_x_m,
        args.ready_z_m,
        args.ready_spread_m,
    )
    reorder = np.asarray(ik._arm_reorder_pin_to_g1)

    q_home, _ = ik.solve_ik(left_home, right_home, q)
    if not np.all(np.isfinite(q_home)):
        print("IK home solve produced non-finite values.", file=sys.stderr)
        return 1
    try:
        q_ready = solve_ready_q(ik, left_ready, right_ready, np.asarray(q_home, dtype=float))
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    q = q_ready.copy()
    left_target = left_ready.copy()
    right_target = right_ready.copy()
    active_home = right_ready if args.hand_side == "right" else left_ready
    clutch = Clutch(active_home)
    ready_action = action_from_arm_q(q_ready[reorder], G1_29_JointIndex, G1_29_JointArmIndex)
    lower_action = action_from_arm_q(np.asarray(q_home, dtype=float)[reorder], G1_29_JointIndex, G1_29_JointArmIndex)
    startup_diagnostics = StartupDiagnosticRequests(
        Path(args.diagnostic_request_file),
        Path(args.diagnostic_ack_file),
        ready_action,
        lower_action,
        args.control_hz,
    )
    if args.dry_run_ik:
        print("dry-run IK ok")
        print(f"  q_dim: {q.size}")
        print(f"  first_arm_joint_g1: {float(q[reorder][0]):+.4f}")
        print(f"  ready_arm_norm: {float(np.linalg.norm(q_ready[reorder])):.4f}")
        return 0

    robot = None
    if args.external_g1_sim:
        print("Connecting to existing G1 MuJoCo DDS sim")
        robot = UnitreeG1(UnitreeG1Config(is_simulation=True))
        try:
            connect_unitree_g1_external_dds(robot, g1_module, G1_29_JointIndex, args.g1_state_timeout_s)
        except TimeoutError as exc:
            print(str(exc), file=sys.stderr)
            return 2
        print(f"external G1 DDS sim connected: {robot.is_connected}")
        scale_arm_gains(robot, G1_29_JointArmIndex, args.arm_kp_scale, args.arm_kd_scale)
        robot.send_action(ready_action)
        print("sent G1 ready pose")

    if args.mock_xr:
        teleop = MockXRController(args.hand_side, args.clutch_threshold)
    else:
        from examples.isaac_teleop_to_so101.isaac_teleop import XRController, XRControllerConfig

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

    def connect_teleop_with_retry() -> None:
        while True:
            try:
                teleop.connect()
                return
            except RuntimeError as exc:
                if not cloudxr_runtime_error(exc):
                    raise
                print("\nOpenXR/CloudXR is not available yet.", file=sys.stderr)
                print("Start CloudXR in another terminal with: cd ~/lerobot-sim/unitree_g1_lerobot && ./run_isaac_teleop.sh", file=sys.stderr)
                print("Then connect the headset browser client.", file=sys.stderr)
                if not args.wait_for_cloudxr:
                    raise
                if robot is not None:
                    print("Holding G1 ready pose while waiting for XR attach.", file=sys.stderr)
                print(f"Waiting {args.cloudxr_retry_s:.1f}s before retrying XR attach...", file=sys.stderr)
                startup_diagnostics.publish_or_ready_for(robot, ready_action, args.cloudxr_retry_s)

    try:
        connect_teleop_with_retry()
    except RuntimeError:
        return 2
    print("XR teleop connected. Waiting for tracked controller frames...")

    if robot is None:
        print("Connecting UnitreeG1 MuJoCo sim")
        robot = UnitreeG1(UnitreeG1Config(is_simulation=True))
        robot.connect()
        print(f"robot connected: {robot.is_connected}")
        scale_arm_gains(robot, G1_29_JointArmIndex, args.arm_kp_scale, args.arm_kd_scale)
        robot.send_action(ready_action)
        print("sent G1 ready pose")

    period_s = 1.0 / args.control_hz
    deadline = float("inf") if args.duration_s <= 0.0 else time.monotonic() + args.duration_s
    was_engaged = False
    last_raw_grip_pos: np.ndarray | None = None
    last_q_g1: np.ndarray | None = None
    last_tracking: bool | None = None
    last_tracking_time = time.monotonic()
    step = 0

    try:
        while time.monotonic() < deadline:
            t0 = time.perf_counter()
            if startup_diagnostics.step(robot):
                step += 1
                time.sleep(max(0.0, period_s - (time.perf_counter() - t0)))
                continue
            xr_action = teleop.get_action()
            tracking = bool(teleop.is_tracking)
            now = time.monotonic()
            if tracking:
                last_tracking_time = now
            elif (
                args.wait_for_cloudxr
                and args.tracking_timeout_s > 0.0
                and now - last_tracking_time >= args.tracking_timeout_s
            ):
                print(
                    f"No tracked controller frames for {args.tracking_timeout_s:.1f}s; "
                    "recreating OpenXR session and holding ready pose.",
                    file=sys.stderr,
                    flush=True,
                )
                teleop.disconnect()
                startup_diagnostics.publish_or_ready_for(robot, ready_action, args.cloudxr_retry_s)
                connect_teleop_with_retry()
                print("XR teleop reconnected. Waiting for tracked controller frames...", flush=True)
                last_tracking_time = time.monotonic()
                was_engaged = False
                last_tracking = None
                continue
            pose_valid = tracking and valid_pose_frame(xr_action["grip_pos"], xr_action["grip_quat"])
            squeeze = float(xr_action["squeeze"])
            trigger = float(xr_action.get("trigger", 0.0))
            clutch_level = clutch_value(xr_action, args.clutch_axis)
            if tracking and not pose_valid:
                tracking = False
                engaged = False
                if step % max(1, int(args.control_hz)) == 0:
                    print(
                        "tracked controller has invalid grip pose; holding previous target "
                        f"pos={np.asarray(xr_action['grip_pos']).tolist()} quat={np.asarray(xr_action['grip_quat']).tolist()}",
                        file=sys.stderr,
                        flush=True,
                    )
            else:
                engaged = pose_valid and clutch_level >= args.clutch_threshold

            if engaged and not was_engaged:
                measured_left, measured_right = fk(model, data, ik.L_hand_id, ik.R_hand_id, q)
                measured_active = measured_right if args.hand_side == "right" else measured_left
                clutch.engage(xr_action["grip_pos"], xr_action["grip_quat"], measured_active)
                print(f"clutch engaged at step {step}")

            if engaged:
                pos, quat = scaled_rebase(clutch, xr_action["grip_pos"], xr_action["grip_quat"], args.xr_pos_scale)
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
            arm_norm = float(np.linalg.norm(q_g1))

            if tracking or tracking != last_tracking or step % max(1, int(args.control_hz)) == 0:
                active_target = right_target if args.hand_side == "right" else left_target
                raw_pos = np.asarray(xr_action["grip_pos"], dtype=float)
                raw_delta = 0.0 if last_raw_grip_pos is None else float(np.linalg.norm(raw_pos - last_raw_grip_pos))
                q_delta = 0.0 if last_q_g1 is None else float(np.linalg.norm(q_g1 - last_q_g1))
                target_delta = float(np.linalg.norm(active_target[:3, 3] - active_home[:3, 3]))
                line = (
                    "step={step:04d} tracking={tracking} engaged={engaged} "
                    "squeeze={squeeze:.3f} trigger={trigger:.3f} clutch={clutch:.3f} "
                    "target=({x:+.3f},{y:+.3f},{z:+.3f}) q0={q0:+.3f} |q|={qnorm:.3f}"
                ).format(
                    step=step,
                    tracking=tracking,
                    engaged=engaged,
                    squeeze=squeeze,
                    trigger=trigger,
                    clutch=clutch_level,
                    x=float(active_target[0, 3]),
                    y=float(active_target[1, 3]),
                    z=float(active_target[2, 3]),
                    q0=float(q_g1[0]),
                    qnorm=arm_norm,
                )
                if args.debug_xr:
                    line += (
                        " raw=({rx:+.3f},{ry:+.3f},{rz:+.3f}) "
                        "raw_d={raw_delta:.4f} target_d={target_delta:.4f} q_d={q_delta:.4f}"
                    ).format(
                        rx=float(raw_pos[0]),
                        ry=float(raw_pos[1]),
                        rz=float(raw_pos[2]),
                        raw_delta=raw_delta,
                        target_delta=target_delta,
                        q_delta=q_delta,
                    )
                print(line)
                last_raw_grip_pos = raw_pos.copy()
                last_q_g1 = q_g1.copy()

            was_engaged = engaged
            last_tracking = tracking
            step += 1
            time.sleep(max(0.0, period_s - (time.perf_counter() - t0)))
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        teleop.disconnect()
        if robot is not None:
            robot.disconnect()
        print("disconnected")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
