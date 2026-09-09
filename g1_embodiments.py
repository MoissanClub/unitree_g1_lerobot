#!/usr/bin/env python3
"""Local G1 arm embodiment specs used while upstream LeRobot has only G1-29 IK."""

from __future__ import annotations

import logging
import os
import sys
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path

import numpy as np

logger = logging.getLogger(__name__)

PROJECT_ROOT = Path(__file__).resolve().parent


class G1_23_JointArmIndex(IntEnum):
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26


class G1_23_JointIndex(IntEnum):
    kLeftHipPitch = 0
    kLeftHipRoll = 1
    kLeftHipYaw = 2
    kLeftKnee = 3
    kLeftAnklePitch = 4
    kLeftAnkleRoll = 5
    kRightHipPitch = 6
    kRightHipRoll = 7
    kRightHipYaw = 8
    kRightKnee = 9
    kRightAnklePitch = 10
    kRightAnkleRoll = 11
    kWaistYaw = 12
    kWaistRollNotUsed = 13
    kWaistPitchNotUsed = 14
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19
    kLeftWristPitchNotUsed = 20
    kLeftWristYawNotUsed = 21
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26
    kRightWristPitchNotUsed = 27
    kRightWristYawNotUsed = 28
    kNotUsedJoint0 = 29
    kNotUsedJoint1 = 30
    kNotUsedJoint2 = 31
    kNotUsedJoint3 = 32
    kNotUsedJoint4 = 33
    kNotUsedJoint5 = 34


@dataclass(frozen=True)
class G1ArmEmbodiment:
    name: str
    urdf_path: Path | None
    arm_joint_names_g1: tuple[str, ...]
    locked_joint_names: tuple[str, ...]
    left_ee_parent: str
    right_ee_parent: str
    ee_offset_m: tuple[float, float, float]
    rotation_weight: float
    filter_weights: tuple[float, ...]


G1_23_SPEC = G1ArmEmbodiment(
    name="g1_23",
    urdf_path=PROJECT_ROOT / "assets" / "g1" / "g1_body23.urdf",
    arm_joint_names_g1=(
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
    ),
    locked_joint_names=(
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
        "waist_yaw_joint",
    ),
    left_ee_parent="left_wrist_roll_joint",
    right_ee_parent="right_wrist_roll_joint",
    ee_offset_m=(0.20, 0.0, 0.0),
    rotation_weight=0.5,
    filter_weights=(0.4, 0.3, 0.2, 0.1),
)


def ensure_lerobot_import(lobot_root: str | Path = "/home/dwei/lerobot-sim/lerobot") -> None:
    root = Path(lobot_root).expanduser().resolve()
    if root.is_dir() and str(root) not in sys.path:
        sys.path.insert(0, str(root))


class ParametricG1ArmIK:
    def __init__(self, spec: G1ArmEmbodiment):
        import casadi
        import pinocchio as pin
        from huggingface_hub import snapshot_download
        from pinocchio import casadi as cpin
        from lerobot.robots.unitree_g1.g1_kinematics import WeightedMovingFilter

        if spec.urdf_path is None or not spec.urdf_path.is_file():
            raise FileNotFoundError(f"{spec.name} URDF not found: {spec.urdf_path}")

        self._pin = pin
        self.spec = spec
        self.unit_test = False
        self.repo_path = snapshot_download("lerobot/unitree-g1-mujoco")
        mesh_dir = os.path.join(self.repo_path, "assets")
        self.robot = pin.RobotWrapper.BuildFromURDF(str(spec.urdf_path), mesh_dir)
        self.mixed_jointsToLockIDs = list(spec.locked_joint_names)
        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=np.zeros(self.robot.model.nq),
        )

        self._arm_joint_names_g1 = list(spec.arm_joint_names_g1)
        self._arm_joint_names_pin = sorted(
            self._arm_joint_names_g1,
            key=lambda name: self.reduced_robot.model.idx_qs[self.reduced_robot.model.getJointId(name)],
        )
        self._arm_reorder_g1_to_pin = [
            self._arm_joint_names_g1.index(name) for name in self._arm_joint_names_pin
        ]
        self._arm_reorder_pin_to_g1 = np.argsort(self._arm_reorder_g1_to_pin)
        logger.info("Pinocchio %s arm joint order: %s", spec.name, self._arm_joint_names_pin)

        self.reduced_robot.model.addFrame(
            pin.Frame(
                "L_ee",
                self.reduced_robot.model.getJointId(spec.left_ee_parent),
                pin.SE3(np.eye(3), np.array(spec.ee_offset_m, dtype=float)),
                pin.FrameType.OP_FRAME,
            )
        )
        self.reduced_robot.model.addFrame(
            pin.Frame(
                "R_ee",
                self.reduced_robot.model.getJointId(spec.right_ee_parent),
                pin.SE3(np.eye(3), np.array(spec.ee_offset_m, dtype=float)),
                pin.FrameType.OP_FRAME,
            )
        )
        self.reduced_robot.data = self.reduced_robot.model.createData()

        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1)
        self.cTf_l = casadi.SX.sym("tf_l", 4, 4)
        self.cTf_r = casadi.SX.sym("tf_r", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        self.L_hand_id = self.reduced_robot.model.getFrameId("L_ee")
        self.R_hand_id = self.reduced_robot.model.getFrameId("R_ee")
        self.translational_error = casadi.Function(
            "translational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    self.cdata.oMf[self.L_hand_id].translation - self.cTf_l[:3, 3],
                    self.cdata.oMf[self.R_hand_id].translation - self.cTf_r[:3, 3],
                )
            ],
        )
        self.rotational_error = casadi.Function(
            "rotational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    cpin.log3(self.cdata.oMf[self.L_hand_id].rotation @ self.cTf_l[:3, :3].T),
                    cpin.log3(self.cdata.oMf[self.R_hand_id].rotation @ self.cTf_r[:3, :3].T),
                )
            ],
        )

        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)
        self.param_tf_l = self.opti.parameter(4, 4)
        self.param_tf_r = self.opti.parameter(4, 4)
        self.translational_cost = casadi.sumsqr(
            self.translational_error(self.var_q, self.param_tf_l, self.param_tf_r)
        )
        self.rotation_cost = casadi.sumsqr(
            self.rotational_error(self.var_q, self.param_tf_l, self.param_tf_r)
        )
        self.regularization_cost = casadi.sumsqr(self.var_q)
        self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)
        self.opti.subject_to(
            self.opti.bounded(
                self.reduced_robot.model.lowerPositionLimit,
                self.var_q,
                self.reduced_robot.model.upperPositionLimit,
            )
        )
        self.opti.minimize(
            50 * self.translational_cost
            + spec.rotation_weight * self.rotation_cost
            + 0.02 * self.regularization_cost
            + 0.1 * self.smooth_cost
        )
        self.opti.solver(
            "ipopt",
            {
                "ipopt": {"print_level": 0, "max_iter": 50, "tol": 1e-6},
                "print_time": False,
                "calc_lam_p": False,
            },
        )

        self.init_data = np.zeros(self.reduced_robot.model.nq)
        self.smooth_filter = WeightedMovingFilter(
            np.array(spec.filter_weights, dtype=float),
            self.reduced_robot.model.nq,
        )

    def solve_ik(self, left_wrist, right_wrist, current_lr_arm_motor_q=None, current_lr_arm_motor_dq=None):
        if current_lr_arm_motor_q is not None:
            self.init_data = np.asarray(current_lr_arm_motor_q, dtype=float)
        self.opti.set_initial(self.var_q, self.init_data)
        self.opti.set_value(self.param_tf_l, left_wrist)
        self.opti.set_value(self.param_tf_r, right_wrist)
        self.opti.set_value(self.var_q_last, self.init_data)

        converged = True
        try:
            self.opti.solve()
            sol_q = self.opti.value(self.var_q)
        except Exception as exc:
            converged = False
            logger.error("IK convergence error for %s: %s", self.spec.name, exc)
            sol_q = self.opti.debug.value(self.var_q)

        self.smooth_filter.add_data(sol_q)
        sol_q = self.smooth_filter.filtered_data
        self.init_data = sol_q
        if not converged and current_lr_arm_motor_q is not None:
            return current_lr_arm_motor_q, np.zeros(self.reduced_robot.model.nv)

        sol_tauff = self._pin.rnea(
            self.reduced_robot.model,
            self.reduced_robot.data,
            sol_q,
            np.zeros(self.reduced_robot.model.nv),
            np.zeros(self.reduced_robot.model.nv),
        )
        return sol_q, sol_tauff


def make_arm_ik(embodiment: str, lerobot_root: str | Path = "/home/dwei/lerobot-sim/lerobot"):
    ensure_lerobot_import(lerobot_root)
    if embodiment == "g1_23":
        return ParametricG1ArmIK(G1_23_SPEC)
    if embodiment == "g1_29":
        from lerobot.robots.unitree_g1.g1_kinematics import G1_29_ArmIK

        ik = G1_29_ArmIK()
        ik.reduced_robot.data = ik.reduced_robot.model.createData()
        return ik
    raise ValueError(f"Unknown G1 embodiment: {embodiment}")


def active_arm_q_to_visual_29(q_g1: np.ndarray, embodiment: str) -> np.ndarray:
    q_g1 = np.asarray(q_g1, dtype=float)
    if embodiment == "g1_29":
        if q_g1.shape != (14,):
            raise ValueError(f"Expected 14 G1-29 arm joints, got {q_g1.shape}")
        return q_g1
    if embodiment == "g1_23":
        if q_g1.shape != (10,):
            raise ValueError(f"Expected 10 G1-23 arm joints, got {q_g1.shape}")
        q29 = np.zeros(14, dtype=float)
        q29[0:5] = q_g1[0:5]
        q29[7:12] = q_g1[5:10]
        return q29
    raise ValueError(f"Unknown G1 embodiment: {embodiment}")


def visual_29_arm_joint_names() -> tuple[str, ...]:
    return (
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    )
