"""Exact DDS-state pairing and MuJoCo-versus-Pinocchio arm geometry checks."""
import json
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin

from ..simulation.geometry_trace import state_key
from .verify_live_control import pose_errors
from ..robots.motor_configs import load_profile

GEOMETRY_LIMITS = dict(position_m=.001, rotation_rad=.002, joint_rad=1e-6, minimum_samples_per_case=10)


def compare_trace(path, received, ik, joints, embodiment, expected_cases):
    # Same URDF as IK, but include the measured waist state rather than assuming
    # the reduced model's locked waist. Torso origins differ across model families.
    model, data = ik.robot.model, ik.robot.model.createData()
    torso = model.getFrameId("torso_link")
    pelvis = model.getFrameId("pelvis")
    if max(torso, pelvis) >= model.nframes:
        raise ValueError("Pinocchio reference frame missing")
    motors = load_profile(embodiment)["motors"]
    frames = [ik.reduced_robot.model.frames[i] for i in (ik.L_hand_id, ik.R_hand_id)]
    parents = [model.getJointId(ik.reduced_robot.model.names[f.parentJoint]) for f in frames]
    by_case = {name: [] for name in expected_cases}
    matched = []
    source_audit = None
    with path.open() as source:
        for line in source:
            if not line.endswith("\n"):
                break
            record = json.loads(line)
            receipt = received.get(record["state_key"])
            if receipt is None or receipt["case"] not in by_case:
                continue
            if record["embodiment"] != embodiment or record["reference_frame"] != "pelvis" or record["version"] != 2:
                raise ValueError("Geometry identity/frame mismatch")
            if source_audit is None:
                urdf = Path(ik.repo_path)/"assets/g1_body29_hand14.urdf" if embodiment == "g1_29" else ik.spec.urdf_path
                root = ET.parse(urdf).getroot()
                origins = {}
                for name, xyz in record["shoulder_origins"].items():
                    joint = root.find(f"joint[@name='{name}']")
                    if joint.find("parent").get("link") != "torso_link":
                        raise ValueError("URDF shoulder parent differs from torso_link")
                    urdf_xyz = np.fromstring(joint.find("origin").get("xyz"), sep=" ")
                    origins[name] = dict(urdf_xyz=urdf_xyz.tolist(), mujoco_xyz=xyz,
                                         urdf_minus_mujoco_m=(urdf_xyz-np.asarray(xyz)).tolist())
                source_audit = dict(urdf=str(urdf), sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),
                                    shoulder_origins=origins)
            raw = receipt["q"]
            if record["state_key"] != state_key(record["tick"], raw):
                raise ValueError("DDS tick or payload mismatch")
            q = np.zeros(model.nq)
            for motor in motors:
                if not model.existJointName(motor["joint"]):
                    raise ValueError(f"Missing URDF joint {motor['joint']}")
                q[model.idx_qs[model.getJointId(motor["joint"])]] = raw[motor["dds_index"]]
            mapping_error = 0.
            for i, motor in enumerate(joints):
                name = ik._arm_joint_names_g1[i]
                value = raw[motor.value]
                q[model.idx_qs[model.getJointId(name)]] = value
                mapping_error = max(mapping_error, abs(value-record["joint_q"][name]))
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            reference = data.oMf[pelvis].inverse()
            hands = [data.oMi[parent] * frame.placement for parent, frame in zip(parents, frames)]
            poses = [(reference * hand).homogeneous.copy() for hand in hands]
            torso_poses = [(data.oMf[torso].inverse() * hand).homogeneous.copy() for hand in hands]
            mujoco_poses = [np.asarray(p) for p in record["hand_poses"]]
            error = pose_errors(poses, mujoco_poses)
            row = dict(case=receipt["case"], sequence=record["sequence"], tick=record["tick"],
                       state_key=record["state_key"], simulation_time_s=record["simulation_time_s"],
                       capture_to_receive_s=receipt["time"]-record["captured_monotonic_s"],
                       joint_mapping_error_rad=mapping_error, error=error,
                       torso_reference_error=pose_errors(torso_poses, [np.asarray(p) for p in record["torso_hand_poses"]]),
                       pinocchio_poses=[p.tolist() for p in poses], mujoco_poses=record["hand_poses"])
            matched.append(row)
            by_case[receipt["case"]].append(row)
    cases = []
    for name, rows in by_case.items():
        metrics = dict(name=name, samples=len(rows), position_m=max((e["position_m"] for r in rows for e in r["error"]), default=None),
                       rotation_rad=max((e["rotation_rad"] for r in rows for e in r["error"]), default=None),
                       joint_rad=max((r["joint_mapping_error_rad"] for r in rows), default=None))
        passed = len(rows) >= GEOMETRY_LIMITS["minimum_samples_per_case"]
        if passed:
            passed = all(metrics[key] <= GEOMETRY_LIMITS[key] for key in ("position_m", "rotation_rad", "joint_rad"))
        metrics["passed"] = bool(passed)
        cases.append(metrics)
    return dict(passed=bool(cases) and all(c["passed"] for c in cases), thresholds=GEOMETRY_LIMITS,
                pairing="exact DDS tick and float32 joint payload", frame="pelvis",
                source_audit=source_audit, cases=cases, samples=matched)
