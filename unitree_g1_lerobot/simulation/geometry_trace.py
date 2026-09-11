"""Opt-in, publish-time MuJoCo geometry journal for independent diagnostics."""
import hashlib
import json
import time

import mujoco
import numpy as np


def state_key(tick, q):
    # DDS serializes q as float32; compare the wire representation, not Python doubles.
    return f"{int(tick)}:" + hashlib.sha256(np.asarray(q, dtype="<f4").tobytes()).hexdigest()


class GeometryTrace:
    """Sample without mutating the live model/data or the outgoing DDS message.

    References are explicitly audited against the two IK frame definitions, not
    obtained from the IK solver. This makes offset changes detectable in verification.
    """
    def __init__(self, model, data, publisher, embodiment, path, every=5):
        self.model, self.live = model, data
        self.reference = mujoco.MjData(model)
        self.publisher, self.original = publisher, publisher.Write
        self.every, self.count = every, 0
        self.embodiment = embodiment
        self.torso = self._id(mujoco.mjtObj.mjOBJ_BODY, "torso_link")
        self.pelvis = self._id(mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        self.shoulder_origins = {}
        for side in ("left", "right"):
            body = self._id(mujoco.mjtObj.mjOBJ_BODY, f"{side}_shoulder_pitch_link")
            if model.body_parentid[body] != self.torso:
                raise ValueError("Shoulder is not directly parented to torso_link; audit reference frames")
            self.shoulder_origins[f"{side}_shoulder_pitch_joint"] = model.body_pos[body].tolist()
        wrist = "yaw" if embodiment == "g1_29" else "roll"
        self.offset = .05 if embodiment == "g1_29" else .20
        self.hands = []
        for side in ("left", "right"):
            joint = self._id(mujoco.mjtObj.mjOBJ_JOINT, f"{side}_wrist_{wrist}_joint")
            if not np.allclose(model.jnt_pos[joint], 0, atol=1e-12):
                raise ValueError("Wrist joint origin is not the body origin; audit the hand frame")
            self.hands.append(int(model.jnt_bodyid[joint]))
        self.joint_ids = [i for i in range(model.njnt) if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE]
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = path.open("x")
        publisher.Write = self.write

    def _id(self, kind, name):
        index = mujoco.mj_name2id(self.model, kind, name)
        if index < 0:
            raise ValueError(f"Missing MuJoCo geometry reference: {name}")
        return index

    def snapshot(self, msg):
        d, m = self.reference, self.model
        d.qpos[:] = self.live.qpos
        d.mocap_pos[:] = self.live.mocap_pos
        d.mocap_quat[:] = self.live.mocap_quat
        # mj_step may leave xpos/xmat at the pre-integration pose. Recompute only
        # kinematics on private data at the exact qpos used for this publication.
        mujoco.mj_kinematics(m, d)
        def poses_in(reference):
            rotation = d.xmat[reference].reshape(3, 3)
            origin = d.xpos[reference]
            poses = []
            for body in self.hands:
                hand_rotation = d.xmat[body].reshape(3, 3)
                pose = np.eye(4)
                pose[:3, :3] = rotation.T @ hand_rotation
                pose[:3, 3] = rotation.T @ (d.xpos[body] + hand_rotation @ [self.offset, 0, 0] - origin)
                poses.append(pose.tolist())
            return poses
        q = [float(motor.q) for motor in msg.motor_state]
        return dict(version=2, embodiment=self.embodiment, sequence=self.count,
                    captured_monotonic_s=time.monotonic(), simulation_time_s=float(self.live.time),
                    tick=int(msg.tick), state_key=state_key(msg.tick, q), dds_q=q,
                    reference_frame="pelvis", hand_offset_m=[self.offset, 0, 0],
                    shoulder_origins=self.shoulder_origins,
                    hand_bodies=[mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) for i in self.hands],
                    joint_q={mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i): float(d.qpos[m.jnt_qposadr[i]])
                             for i in self.joint_ids}, hand_poses=poses_in(self.pelvis), torso_hand_poses=poses_in(self.torso))

    def write(self, msg, *args, **kwargs):
        self.count += 1
        if self.count % self.every == 0:
            self.file.write(json.dumps(self.snapshot(msg), allow_nan=False)+"\n")
            self.file.flush()
        return self.original(msg, *args, **kwargs)

    def close(self):
        self.publisher.Write = self.original
        self.file.close()


def start_geometry_trace(model, data, publisher, args):
    if args.geometry_log is None:
        return None
    return GeometryTrace(model, data, publisher, args.embodiment, args.geometry_log)
