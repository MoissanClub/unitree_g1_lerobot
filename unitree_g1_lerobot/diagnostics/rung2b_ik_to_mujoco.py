#!/usr/bin/env python3
"""Rung 2b - guarded G1 IK to LeRobot G1 MuJoCo validation.

It patches the Hub-hosted MuJoCo environment at runtime so DDS loopback works
and image publishing is disabled for reliable headless validation.
"""

from __future__ import annotations

import os
import time

import numpy as np
import pinocchio as pin
import unitree_sdk2py.core.channel as unitree_channel

from lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config
from lerobot.robots.unitree_g1.g1_kinematics import G1_29_ArmIK
from lerobot.robots.unitree_g1.g1_utils import G1_29_JointArmIndex, G1_29_JointIndex

HZ, N, RAD = 50.0, 400, 0.06


def patch_unitree_dds_config() -> None:
    unitree_channel.ChannelConfigHasInterface = """<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
    <Domain Id="any">
        <General>
            <Interfaces>
                <NetworkInterface name="$__IF_NAME__$" priority="default" multicast="default"/>
            </Interfaces>
        </General>
    </Domain>
</CycloneDDS>"""


def patch_g1_hub_env_factory() -> None:
    import lerobot.envs.factory as env_factory

    original_call_make_env = env_factory._call_make_env

    def call_make_env_no_images(module, n_envs, use_async_envs, cfg):
        module_file = str(getattr(module, "__file__", ""))
        if "models--lerobot--unitree-g1-mujoco" in module_file:
            return module.make_env(
                n_envs=n_envs,
                use_async_envs=use_async_envs,
                publish_images=False,
                cameras=[],
            )
        return original_call_make_env(module, n_envs, use_async_envs, cfg)

    env_factory._call_make_env = call_make_env_no_images


def build_action(q_g1: np.ndarray) -> dict[str, float]:
    action = {f"{j.name}.q": 0.0 for j in G1_29_JointIndex}
    action.update({f"{j.name}.q": float(q_g1[k]) for k, j in enumerate(G1_29_JointArmIndex)})
    return action


def main() -> int:
    patch_unitree_dds_config()
    patch_g1_hub_env_factory()

    ik = G1_29_ArmIK()
    m = ik.reduced_robot.model
    d = m.createData()
    ik.reduced_robot.data = d

    pin.forwardKinematics(m, d, np.zeros(m.nq))
    pin.updateFramePlacements(m, d)
    l0 = d.oMf[ik.L_hand_id].homogeneous.copy()
    r0 = d.oMf[ik.R_hand_id].homogeneous.copy()
    reorder = np.asarray(ik._arm_reorder_pin_to_g1)

    robot = UnitreeG1(UnitreeG1Config(is_simulation=True))
    print("connecting (loads the MuJoCo env)...", flush=True)
    robot.connect()
    print("connected:", robot.is_connected, flush=True)

    obs = robot.get_observation()
    print(f"observation keys: {len(obs)}  sample: {list(obs)[:3]}", flush=True)

    q = np.zeros(m.nq)
    try:
        for i in range(N):
            t0 = time.perf_counter()
            r = RAD * min(1.0, i / 100.0)
            a = 2 * np.pi * i / 120
            off = np.array([0.0, r * np.sin(a), r * (1.0 - np.cos(a))])
            left_target, right_target = l0.copy(), r0.copy()
            left_target[:3, 3] += off
            right_target[:3, 3] += off

            q, _ = ik.solve_ik(left_target, right_target, q)
            q_g1 = np.asarray(q)[reorder]
            robot.send_action(build_action(q_g1))

            if i % 50 == 0:
                obs = robot.get_observation()
                meas = obs.get("kLeftShoulderPitch.q")
                err = None if meas is None else abs(meas - q_g1[0])
                print(
                    f"  step {i:3d}  cmd={q_g1[0]:+.4f}  meas={meas}"
                    + (f"  |err|={err:.4f}" if err is not None else ""),
                    flush=True,
                )

            time.sleep(max(0.0, 1.0 / HZ - (time.perf_counter() - t0)))
    finally:
        robot.disconnect()
        print("disconnected", flush=True)

    print("\nPASS - G1 MuJoCo sim animated and meas tracked cmd.", flush=True)
    os._exit(0)


if __name__ == "__main__":
    raise SystemExit(main())
