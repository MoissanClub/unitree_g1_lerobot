"""Independent geometry frames, exact pairing, and deliberate mismatch detection."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import mujoco
import numpy as np

from unitree_g1_lerobot.simulation.geometry_trace import GeometryTrace, state_key
from unitree_g1_lerobot.diagnostics.simulation.geometry_acceptance import compare_trace


class GeometryTraceTests(unittest.TestCase):
    def test_full_chain_model_family_audit(self):
        from huggingface_hub import snapshot_download
        from unitree_g1_lerobot.diagnostics.simulation.audit_g1_sources import compare_models, HUB_REVISION
        assets = Path(snapshot_download("lerobot/unitree-g1-mujoco", revision=HUB_REVISION, local_files_only=True))/"assets"
        cases = compare_models(assets/"g1_body29_hand14.urdf", assets/"g1_29dof_with_hand.xml")
        for case in cases:
            pelvis_error = max(e["position_m"] for e in case["pelvis"])
            if "roll" in case["name"] or "pitch" in case["name"]:
                self.assertGreater(pelvis_error, .002)
                self.assertLess(pelvis_error, .004)
            else:
                self.assertLess(pelvis_error, 1e-5)
            self.assertAlmostEqual(max(e["position_m"] for e in case["torso_link"]), .01, places=5)

    def test_wire_key_and_tick(self):
        self.assertEqual(state_key(1, [.1]), state_key(1, np.array([.1], dtype=np.float32)))
        self.assertNotEqual(state_key(1, [.1]), state_key(2, [.1]))
        self.assertNotEqual(state_key(1, [.1]), state_key(1, [-.1]))

    def test_torso_frame_invariance_and_private_kinematics(self):
        model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
          <body name="pelvis" pos="0 0 1"><freejoint/><geom size=".1"/><body name="torso_link"><geom size=".1"/>
          <body name="left_shoulder_pitch_link"/><body name="right_shoulder_pitch_link"/>
          <body name="left_wrist_yaw_link" pos=".3 .2 0"><joint name="left_wrist_yaw_joint" axis="0 0 1"/><geom size=".02"/></body>
          <body name="right_wrist_yaw_link" pos=".3 -.2 0"><joint name="right_wrist_yaw_joint" axis="0 0 1"/><geom size=".02"/></body>
          </body></body></worldbody></mujoco>''')
        data = mujoco.MjData(model)
        publisher = SimpleNamespace(Write=lambda msg: True)
        original = publisher.Write
        msg = SimpleNamespace(tick=1, motor_state=[SimpleNamespace(q=0.)]*35)
        with tempfile.TemporaryDirectory() as directory:
            trace = GeometryTrace(model, data, publisher, "g1_29", Path(directory)/"geometry.jsonl", every=1)
            try:
                before = data.xpos.copy()
                first = trace.snapshot(msg)
                np.testing.assert_allclose(np.asarray(first["hand_poses"])[0, :3, 3], [.35, .2, 0])
                data.qpos[:3] = [2, 3, 4]
                data.qpos[3:7] = [np.sqrt(.5), 0, 0, np.sqrt(.5)]
                second = trace.snapshot(msg)
                np.testing.assert_allclose(first["hand_poses"], second["hand_poses"], atol=1e-12)
                np.testing.assert_array_equal(data.xpos, before)
                data.qpos[7] = np.pi/2
                rotated = np.asarray(trace.snapshot(msg)["hand_poses"])[0]
                np.testing.assert_allclose(rotated[:3, 3], [.3, .25, 0], atol=1e-12)
                self.assertTrue(publisher.Write(msg))
            finally:
                trace.close()
            self.assertIs(publisher.Write, original)

    def test_pinned_model_discrepancy_and_deliberate_faults(self):
        from unitree_g1_lerobot.robots.unitree_g1 import get_g1_embodiment
        from unitree_g1_lerobot.robots.motor_configs import load_profile
        from unitree_g1_lerobot.simulation.motor_bench import MotorPlant, mesh_directory
        for variant in ("g1_29", "g1_23"):
            with self.subTest(variant=variant), tempfile.TemporaryDirectory() as directory:
                spec = get_g1_embodiment(variant)
                ik = spec.make_ik()
                plant = MotorPlant(load_profile(variant), mesh_directory())
                path = Path(directory)/"trace.jsonl"
                trace = GeometryTrace(plant.model, plant.data, SimpleNamespace(Write=lambda msg: True), variant, path, every=1)
                received = {}
                try:
                    for tick in range(12):
                        q = [0.]*35
                        for name, motor in zip(ik._arm_joint_names_g1, spec.arm_index):
                            q[motor.value] = float(plant.data.qpos[plant.model.joint(name).qposadr[0]])
                        msg = SimpleNamespace(tick=tick, motor_state=[SimpleNamespace(q=x) for x in q])
                        trace.write(msg)
                        received[state_key(tick, q)] = dict(q=q, case="test", time=0.)
                finally:
                    trace.close()
                def compare():
                    return compare_trace(path, received, ik, list(spec.arm_index), variant, ["test"])
                baseline = compare()
                self.assertTrue(baseline["passed"])
                if variant == "g1_29":
                    self.assertAlmostEqual(baseline["samples"][0]["torso_reference_error"][0]["position_m"], .01, places=5)
                    self.assertLess(baseline["cases"][0]["rotation_rad"], .002)
                    for audit in baseline["source_audit"]["shoulder_origins"].values():
                        np.testing.assert_allclose(audit["urdf_minus_mujoco_m"], [0, 0, .01], atol=1e-12)
                else:
                    self.assertTrue(baseline["passed"])
                original = path.read_text()
                rows = [json.loads(line) for line in original.splitlines()]
                for row in rows:
                    row["hand_poses"][0][0][3] += .02
                path.write_text("".join(json.dumps(row)+"\n" for row in rows))
                self.assertFalse(compare()["passed"])
                self.assertGreater(compare()["cases"][0]["position_m"], baseline["cases"][0]["position_m"]+.005)
                rows = [json.loads(line) for line in original.splitlines()]
                for row in rows:
                    row["joint_q"][ik._arm_joint_names_g1[0]] += .1
                path.write_text("".join(json.dumps(row)+"\n" for row in rows))
                self.assertFalse(compare()["passed"])
                self.assertGreater(compare()["cases"][0]["joint_rad"], .09)
                path.write_text(original)
                rows = [json.loads(line) for line in original.splitlines()]
                rotation = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
                for row in rows:
                    pose = np.asarray(row["hand_poses"][0])
                    pose[:3, :3] = pose[:3, :3] @ rotation
                    row["hand_poses"][0] = pose.tolist()
                path.write_text("".join(json.dumps(row)+"\n" for row in rows))
                self.assertGreater(compare()["cases"][0]["rotation_rad"], 1.)
                self.assertFalse(compare()["passed"])
                path.write_text(original)
                received.clear()
                self.assertFalse(compare()["passed"])


if __name__ == "__main__":
    unittest.main()
