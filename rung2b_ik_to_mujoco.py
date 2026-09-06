#!/usr/bin/env python3
"""Rung 2b - drive LeRobot's G1 MuJoCo sim from the IK. No XR, no hardware.

Checks: make_env("lerobot/unitree-g1-mujoco"), the UnitreeG1 robot interface,
DDS-on-loopback init, and the action/observation round trip.
"""
import time

import numpy as np
import pinocchio as pin

from lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config
from lerobot.robots.unitree_g1.g1_kinematics import G1_29_ArmIK
from lerobot.robots.unitree_g1.g1_utils import G1_29_JointArmIndex, G1_29_JointIndex

HZ, N, RAD = 50.0, 400, 0.06

ik = G1_29_ArmIK()
# NOTE: G1_29_ArmIK adds the L_ee/R_ee frames AFTER buildReducedRobot() made
# reduced_robot.data, so the shipped data is stale (data.oMf too short).
# Recreate it -- this is what exo_ik.py does too.
m = ik.reduced_robot.model
d = m.createData()
ik.reduced_robot.data = d

pin.forwardKinematics(m, d, np.zeros(m.nq))
pin.updateFramePlacements(m, d)
L0 = d.oMf[ik.L_hand_id].homogeneous.copy()
R0 = d.oMf[ik.R_hand_id].homogeneous.copy()

# solve_ik returns q in Pinocchio order; the robot wants G1 motor order.
reorder = np.asarray(ik._arm_reorder_pin_to_g1)

robot = UnitreeG1(UnitreeG1Config(is_simulation=True))
print("connecting (loads the MuJoCo env)...")
robot.connect()
print("connected:", robot.is_connected)

obs = robot.get_observation()
print(f"observation keys: {len(obs)}  sample: {list(obs)[:3]}")

q = np.zeros(m.nq)
try:
    for i in range(N):
        t0 = time.perf_counter()
        # ramp the radius in over the first 100 steps so step 0 is not a jump
        r = RAD * min(1.0, i / 100.0)
        a = 2 * np.pi * i / 120
        off = np.array([0.0, r * np.sin(a), r * (1.0 - np.cos(a))])
        L, R = L0.copy(), R0.copy()
        L[:3, 3] += off
        R[:3, 3] += off

        q, _ = ik.solve_ik(L, R, q)
        q_g1 = np.asarray(q)[reorder]          # pinocchio order -> G1 motor order

        action = {f"{j.name}.q": 0.0 for j in G1_29_JointIndex}      # legs+waist held at 0
        action.update({f"{j.name}.q": float(q_g1[k])
                       for k, j in enumerate(G1_29_JointArmIndex)})
        robot.send_action(action)

        if i % 50 == 0:
            o = robot.get_observation()
            meas = o.get("kLeftShoulderPitch.q")
            err = None if meas is None else abs(meas - q_g1[0])
            print(f"  step {i:3d}  cmd={q_g1[0]:+.4f}  meas={meas}"
                  + (f"  |err|={err:.4f}" if err is not None else ""))

        time.sleep(max(0.0, 1.0 / HZ - (time.perf_counter() - t0)))
finally:
    robot.disconnect()
    print("disconnected")

print("\nPASS if the sim window animated and meas tracked cmd.")
