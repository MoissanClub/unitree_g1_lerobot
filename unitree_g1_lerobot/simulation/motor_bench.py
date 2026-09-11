"""Supported-arm physics benchmark, independent of XR, DDS, and IK."""
from __future__ import annotations

import time
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from unitree_g1_lerobot.diagnostics.shared.motor_suite import MotorTest, ready, target
from unitree_g1_lerobot.robots.motor_configs import SOURCES

DT = 0.002
CONTROL_DT = 0.004
MESH_REVISION = "a38dc8617f0fca51b38e9354dc58ee35ad850fb5"


def mesh_directory() -> Path:
    from huggingface_hub import snapshot_download
    return Path(snapshot_download("lerobot/unitree-g1-mujoco", revision=MESH_REVISION)) / "assets/meshes"


class MotorPlant:
    def __init__(self, profile: dict, meshes: Path, payload_kg: float = 0, gravity_compensation: bool = False):
        self.profile = profile
        self.gravity_compensation = gravity_compensation
        self.motors = [m for m in profile["motors"] if m["arm"]]
        self.joints = [m["joint"] for m in self.motors]
        root = ET.parse(SOURCES / f"{profile['variant']}dof.xml").getroot()
        root.find("compiler").set("meshdir", str(meshes.resolve()))
        world = root.find("worldbody")
        # Support the pelvis and all non-arm joints, retaining the native link geometry.
        for body in list(world.findall("body")):
            if body.get("name") != "pelvis":
                world.remove(body)
        for parent in root.iter():
            for joint in list(parent.findall("joint")):
                if joint.get("name") and joint.get("name") not in self.joints:
                    parent.remove(joint)
        for tag in ("sensor", "keyframe", "equality"):
            for node in list(root.findall(tag)):
                root.remove(node)
        for actuator in list(root.find("actuator")):
            if actuator.get("joint") not in self.joints:
                root.find("actuator").remove(actuator)
        for geom in world.iter("geom"):
            if geom.get("contype") != "0":
                geom.set("group", "3")
        ET.SubElement(root, "option", timestep=str(DT), integrator="implicitfast", gravity="0 0 -9.81")
        ET.SubElement(root, "visual")
        ET.SubElement(root.find("visual"), "global", offwidth="1600", offheight="1200")
        ET.SubElement(world, "light", pos="2 1 3", dir="-1 -0.3 -1", diffuse="0.7 0.7 0.7", ambient="0.4 0.4 0.4")
        ET.SubElement(world, "light", pos="-1 -2 2", dir="0.5 1 -1", diffuse="0.4 0.4 0.4")
        ET.SubElement(world, "geom", name="floor", type="plane", size="3 3 0.1", rgba="0.30 0.34 0.36 1", group="2")
        self.ee_offsets = {}
        for side in ("left", "right"):
            last = f"{side}_wrist_yaw_joint" if profile["variant"] == "g1_29" else f"{side}_wrist_roll_joint"
            body = next(b for b in world.iter("body") if any(j.get("name") == last for j in b.findall("joint")))
            offset = "0.05 0 0" if profile["variant"] == "g1_29" else "0.20 0 0"
            ET.SubElement(body, "site", name=f"{side}_ee", pos=offset, size="0.008", rgba="0.2 0.8 0.5 1", group="2")
            if payload_kg:
                payload = ET.SubElement(body, "body", name=f"{side}_payload", pos=offset)
                ET.SubElement(payload, "geom", type="sphere", size="0.035", mass=str(payload_kg), rgba="0.8 0.5 0.15 1", contype="0", conaffinity="0", group="2")
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
        self.data = mujoco.MjData(self.model)
        self.reference = mujoco.MjData(self.model)
        self.gravity_data = mujoco.MjData(self.model)
        self.last_feedforward = np.zeros(self.model.nu)
        joint_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in self.joints]
        self.qadr = self.model.jnt_qposadr[joint_ids]
        self.vadr = self.model.jnt_dofadr[joint_ids]
        self.aids = np.array([next(i for i in range(self.model.nu) if self.model.actuator_trnid[i, 0] == j) for j in joint_ids])
        self.kp = np.array([m["kp"] for m in self.motors])
        self.kd = np.array([m["kd"] for m in self.motors])
        self.limits = np.array([m["torque_limit_nm"] for m in self.motors])
        self.ranges = np.array([m["range_rad"] for m in self.motors])
        self.sites = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_ee") for side in ("left", "right")]
        assert self.model.nq == len(self.joints) == self.model.nu
        for motor, jid in zip(self.motors, joint_ids, strict=True):
            did = self.model.jnt_dofadr[jid]
            assert np.isclose(self.model.dof_damping[did], motor["damping"])
            assert np.isclose(self.model.dof_armature[did], motor["armature"])
            assert np.isclose(self.model.dof_frictionloss[did], motor["frictionloss"])
        self.reset()

    def reset(self, command=None):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qadr] = ready(self.joints) if command is None else command
        mujoco.mj_forward(self.model, self.data)

    def gravity_torque(self):
        # Evaluate bias at the measured pose with zero velocity: gravity only.
        self.gravity_data.qpos[:] = self.data.qpos
        self.gravity_data.qvel[:] = 0
        mujoco.mj_forward(self.model, self.gravity_data)
        return self.gravity_data.qfrc_bias[self.vadr].copy()

    def step(self, command):
        raw = self.kp * (command - self.data.qpos[self.qadr]) - self.kd * self.data.qvel[self.vadr]
        self.last_feedforward = self.gravity_torque() if self.gravity_compensation else np.zeros_like(raw)
        raw += self.last_feedforward
        torque = np.clip(raw, -self.limits, self.limits)
        self.data.ctrl[self.aids] = torque
        mujoco.mj_step(self.model, self.data)
        if not np.all(np.isfinite(self.data.qpos)) or np.any(self.data.warning.number):
            raise RuntimeError(f"{self.profile['name']}: unstable simulation: {self.data.warning.number}")
        return raw, torque

    def hand_poses(self, q):
        self.reference.qpos[self.qadr] = q
        mujoco.mj_forward(self.model, self.reference)
        return self.reference.site_xpos[self.sites].copy(), self.reference.site_xmat[self.sites].reshape(2, 3, 3).copy()


def run_trial(plant: MotorPlant, test: MotorTest) -> dict:
    initial = target(test, 0, plant.joints)
    plant.reset(initial)
    for _ in range(round(0.75 / DT)):
        plant.step(initial)
    count = round(test.seconds / DT)
    n = len(plant.joints)
    log = {key: np.zeros((count, n)) for key in ("target", "delivered", "q", "dq", "raw_torque", "torque", "gravity_feedforward", "gravity_required")}
    log.update(time=np.arange(count) * DT, hand_error=np.zeros((count, 2)), orientation_error=np.zeros((count, 2)))
    log["target_hand"] = np.zeros((count, 2, 3))
    log["actual_hand"] = np.zeros((count, 2, 3))
    started = time.perf_counter()
    for i in range(count):
        t = i * DT
        if i % 2 == 0:
            desired = target(test, t, plant.joints)
            delivered = target(test, t - test.delay_s, plant.joints)
            if np.any(desired < plant.ranges[:, 0]) or np.any(desired > plant.ranges[:, 1]):
                raise ValueError(f"{test.name}: target outside joint limits")
        q = plant.data.qpos[plant.qadr].copy()
        dq = plant.data.qvel[plant.vadr].copy()
        actual_hand, actual_rot = plant.hand_poses(q)
        target_hand, target_rot = plant.hand_poses(desired)
        log["target"][i], log["delivered"][i], log["q"][i], log["dq"][i] = desired, delivered, q, dq
        log["actual_hand"][i], log["target_hand"][i] = actual_hand, target_hand
        log["hand_error"][i] = np.linalg.norm(actual_hand - target_hand, axis=1)
        relative = actual_rot @ np.swapaxes(target_rot, -1, -2)
        log["orientation_error"][i] = np.arccos(np.clip((np.trace(relative, axis1=-2, axis2=-1) - 1) / 2, -1, 1))
        log["gravity_required"][i] = plant.gravity_torque()
        log["raw_torque"][i], log["torque"][i] = plant.step(delivered)
        log["gravity_feedforward"][i] = plant.last_feedforward
    elapsed = time.perf_counter() - started
    error = log["q"] - log["target"]
    common = [i for i, j in enumerate(plant.joints) if "wrist_pitch" not in j and "wrist_yaw" not in j]
    metrics = {
        "joint_rmse_rad": float(np.sqrt(np.mean(error ** 2))),
        "common_joint_rmse_rad": float(np.sqrt(np.mean(error[:, common] ** 2))),
        "last_second_common_rmse_rad": float(np.sqrt(np.mean(error[-round(1 / DT):, common] ** 2))),
        "hand_rmse_m": float(np.sqrt(np.mean(log["hand_error"] ** 2))),
        "orientation_rmse_deg": float(np.rad2deg(np.sqrt(np.mean(log["orientation_error"] ** 2)))),
        "peak_torque_nm": float(np.max(np.abs(log["torque"]))),
        "saturation_fraction": float(np.mean(np.abs(log["raw_torque"]) >= plant.limits)),
        "peak_joint_speed_rad_s": float(np.max(np.abs(log["dq"]))),
        "max_limit_violation_rad": float(max(0, np.max(plant.ranges[:, 0] - log["q"]), np.max(log["q"] - plant.ranges[:, 1]))),
        "physics_compute_s": elapsed,
        "max_step_overshoot_rad": None, "settling_to_target_s": None,
        "step_equilibrium_overshoot_rad": None, "settling_to_equilibrium_s": None,
        "torque_slew_rms_nm_s": float(np.sqrt(np.mean((np.diff(log["torque"], axis=0) / DT) ** 2))),
        "estimated_lag_s": None, "stop_peak_excursion_rad": None,
        "stop_last_second_peak_speed_rad_s": None,
        "rise_to_90_s": None, "rise_10_to_90_s": None,
        "max_step_overshoot_pct": None, "active_joint_rmse_rad": None,
        "post_motion_settling_s": None, "post_motion_peak_error_rad": None,
        "last_second_hand_sag_m": float(np.max(np.mean((log["target_hand"] - log["actual_hand"])[-round(1 / DT):, :, 2], axis=0))),
        "peak_gravity_torque_fraction": float(np.max(np.abs(log["gravity_required"]) / plant.limits)),
    }
    active = np.flatnonzero(np.ptp(log["target"], axis=0) > 1e-9)
    if len(active):
        metrics["active_joint_rmse_rad"] = float(np.sqrt(np.mean(error[:, active] ** 2)))
    if test.name.endswith("_step"):
        delta = log["target"][-1] - log["target"][0]
        active = np.flatnonzero(np.abs(delta) > 0)
        start = round(1 / DT)
        signed = error[start:, active] * np.sign(delta[active])
        metrics["max_step_overshoot_rad"] = float(max(0, signed.max()))
        metrics["max_step_overshoot_pct"] = float(max(0, np.max(signed / np.abs(delta[active]))) * 100)
        progress = (log["q"][start:, active] - log["target"][0, active]) / delta[active]
        crossings = []
        for column in progress.T:
            lo, hi = np.flatnonzero(column >= 0.1), np.flatnonzero(column >= 0.9)
            if len(lo) and len(hi):
                crossings.append((hi[0] * DT, (hi[0] - lo[0]) * DT))
        if len(crossings) == len(active):
            metrics["rise_to_90_s"] = float(max(c[0] for c in crossings))
            metrics["rise_10_to_90_s"] = float(max(c[1] for c in crossings))
        inside = np.all(np.abs(error[start:, active]) <= 0.02, axis=1)
        last_bad = np.flatnonzero(~inside)
        settle_index = int(last_bad[-1] + 1) if len(last_bad) else 0
        if len(inside) - settle_index >= round(0.5 / DT):
            metrics["settling_to_target_s"] = settle_index * DT
        # Gravity can bias equilibrium away from the target; also measure ringing
        # around the achieved final pose so bias does not hide transient overshoot.
        equilibrium = np.mean(log["q"][-round(0.5 / DT):, active], axis=0)
        equilibrium_error = log["q"][start:, active] - equilibrium
        metrics["step_equilibrium_overshoot_rad"] = float(max(0, np.max(equilibrium_error * np.sign(delta[active]))))
        outside = np.flatnonzero(np.any(np.abs(equilibrium_error) > 0.005, axis=1))
        settle = int(outside[-1] + 1) if len(outside) else 0
        if len(equilibrium_error) - settle >= round(0.5 / DT):
            metrics["settling_to_equilibrium_s"] = settle * DT
    elif test.name not in {"pose_hold", "payload_hold"}:
        joint = int(np.argmax(np.ptp(log["target"], axis=0)))
        command = log["target"][:, joint]
        actual = log["q"][:, joint]
        margin = round(0.2 / DT)
        reference = command[margin:-margin]
        reference = reference - reference.mean()
        scores = []
        shifts = range(-margin, margin + 1)
        for shift in shifts:
            measured = actual[margin + shift:len(actual) - margin + shift]
            measured = measured - measured.mean()
            scores.append(float(reference @ measured / max(1e-12, np.linalg.norm(reference) * np.linalg.norm(measured))))
        metrics["estimated_lag_s"] = list(shifts)[int(np.argmax(scores))] * DT
    if test.name == "stop_and_hold":
        stop = round(2.0 / DT)
        metrics["stop_peak_excursion_rad"] = float(np.max(np.abs(log["q"][stop:] - log["q"][stop])))
        metrics["stop_last_second_peak_speed_rad_s"] = float(np.max(np.abs(log["dq"][-round(1 / DT):])))
    if test.kind in {"fast_move", "reversal"}:
        stop = round((1 + test.move_seconds * (2 if test.kind == "reversal" else 1)) / DT)
        post_error = error[stop:, active]
        metrics["post_motion_peak_error_rad"] = float(np.max(np.abs(post_error)))
        outside = np.flatnonzero(np.any(np.abs(post_error) > 0.02, axis=1))
        settle = int(outside[-1] + 1) if len(outside) else 0
        if len(post_error) - settle >= round(0.5 / DT):
            metrics["post_motion_settling_s"] = settle * DT
    return {"test": test, "profile": plant.profile, "joints": plant.joints, "log": log, "metrics": metrics,
            "gravity_compensation": plant.gravity_compensation}
