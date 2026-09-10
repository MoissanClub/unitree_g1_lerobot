"""Opt-in real EGL camera and renderer-process checks, without DDS or XR."""
import os
from pathlib import Path
import tempfile
import time
import unittest

import numpy as np

from unitree_g1_lerobot.simulation.camera_frames import read_frame
from unitree_g1_lerobot.simulation.robot_camera import CameraConfig, CameraPublisher, RobotCamera

SCENE = '''<mujoco><worldbody><light pos="0 0 4" ambient=".5 .5 .5"/>
<body name="torso_link" pos="0 0 1"><freejoint/>
<geom type="box" size=".04 .04 .04" mass="1"/>
<geom type="sphere" pos=".6 .2 .05" size=".10" rgba="1 0 0 1"/>
<geom type="sphere" pos=".6 -.2 .05" size=".10" rgba="0 0 1 1"/>
</body></worldbody></mujoco>'''


@unittest.skipUnless(os.environ.get("G1_TEST_CAMERA") == "1", "requires EGL camera rendering")
class RobotCameraTests(unittest.TestCase):
    def test_mounted_pose_and_left_right_image_orientation(self):
        import mujoco
        model = mujoco.MjModel.from_xml_string(SCENE)
        data = mujoco.MjData(model)
        camera = RobotCamera(model, CameraConfig(width=320, height=240))
        state = dict(qpos=data.qpos.copy(), mocap_pos=data.mocap_pos.copy(),
                     mocap_quat=data.mocap_quat.copy(), simulation_time_s=0.)
        try:
            pixels, pose = camera.render(state)
            red = (pixels[:,:,0].astype(float) > pixels[:,:,2] * 1.5) & (pixels[:,:,0] > 100)
            blue = (pixels[:,:,2].astype(float) > pixels[:,:,0] * 1.5) & (pixels[:,:,2] > 100)
            self.assertGreater(red.sum(), 100)
            self.assertGreater(blue.sum(), 100)
            self.assertLess(np.where(red)[1].mean(), np.where(blue)[1].mean())
            self.assertAlmostEqual(pose["position_world"][2], 1.38)
            state["qpos"][3:7] = [np.cos(np.pi/4), 0., 0., np.sin(np.pi/4)]
            _, rotated = camera.render(state)
            np.testing.assert_allclose(rotated["position_world"], [0., .1, 1.38], atol=1e-8)
            self.assertAlmostEqual(rotated["forward_world"][0], 0.)
            self.assertGreater(rotated["forward_world"][1], .5)
        finally:
            camera.close()

    def test_renderer_failure_does_not_block_producer_and_cleans_channel(self):
        import mujoco
        model = mujoco.MjModel.from_xml_string(SCENE)
        data = mujoco.MjData(model)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "camera.rgb"
            producer = CameraPublisher(model, data, CameraConfig(width=160, height=120), "test", path)
            try:
                self.assertIsNotNone(read_frame(path, max_age_s=30))
                producer.process.terminate()
                producer.process.join(timeout=5)
                producer.last_offer = -float("inf")
                started = time.monotonic()
                producer.offer(data)
                self.assertLess(time.monotonic()-started, .1)
                self.assertTrue(producer.failed)
            finally:
                producer.close()
                producer.close()
            self.assertFalse(path.exists())


if __name__ == "__main__":
    unittest.main()
