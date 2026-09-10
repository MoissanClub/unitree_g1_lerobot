"""Real GPU presentation checks plus control-mailbox freshness checks."""
import os
from pathlib import Path
import queue
import tempfile
import time
import unittest
from unittest.mock import Mock

import numpy as np

from unitree_g1_lerobot.simulation.camera_frames import FrameWriter
from unitree_g1_lerobot.xr.camera_display import CameraDisplay, VideoConfig
from unitree_g1_lerobot.xr.video_controller import VideoXRController, XRVideoSessionEnded


class ControllerMailboxTests(unittest.TestCase):
    def test_stalled_display_releases_both_clutches(self):
        controller = VideoXRController(None, VideoConfig())
        controller.process = Mock()
        controller.process.is_alive.return_value = True
        controller.errors = queue.Queue()
        controller.frames = queue.Queue()
        action = {f"{side}.{key}": value for side in ("left", "right")
                  for key, value in dict(grip_pos=[1, 2, 3], grip_quat=[0, 0, 0, 1],
                                        squeeze=1., trigger=1.).items()}
        controller.frames.put(dict(action=action, tracking=dict(left=True, right=True),
                                   captured=time.monotonic(), video={}))
        self.assertEqual(controller.get_action()["right.squeeze"], 1.)
        self.assertTrue(controller.is_tracking)
        controller.snapshot["captured"] -= 1
        self.assertEqual(controller.get_action()["right.squeeze"], 0.)
        self.assertFalse(controller.is_tracking)
        controller.process.is_alive.return_value = False
        with self.assertRaises(XRVideoSessionEnded):
            controller.get_action()


@unittest.skipUnless(os.environ.get("G1_TEST_VIDEO") == "1", "requires Isaac Teleop Viz and CUDA")
class CameraDisplayTests(unittest.TestCase):
    def test_pixels_staleness_source_and_restart(self):
        with tempfile.TemporaryDirectory() as folder:
            channel = Path(folder) / "camera.rgb"
            display = CameraDisplay(VideoConfig(channel=str(channel), expected_source="test"), offscreen=True)
            writer = FrameWriter(channel, 640, 480)
            try:
                display.update_camera()
                self.assertNotEqual(display.status, "live")
                pixels = np.zeros((480, 640, 3), dtype=np.uint8)
                pixels[:240, :320] = [255, 0, 0]
                pixels[240:, 320:] = [0, 0, 255]
                writer.publish(pixels, dict(captured_monotonic_ns=time.monotonic_ns(), embodiment="test"))
                display.update_camera()
                display.render()
                actual = display.readback()
                np.testing.assert_allclose(actual[100, 100, :3], [255, 0, 0], atol=3)
                np.testing.assert_allclose(actual[380, 540, :3], [0, 0, 255], atol=3)
                writer.publish(pixels, dict(captured_monotonic_ns=time.monotonic_ns()-2_000_000_000, embodiment="test"))
                display.update_camera()
                self.assertIn("stale", display.status)
                writer.publish(pixels, dict(captured_monotonic_ns=time.monotonic_ns(), embodiment="wrong"))
                display.update_camera()
                self.assertIn("mismatch", display.status)
                old_session = display.stats["camera_session"]
                writer.close()
                writer = FrameWriter(channel, 320, 240)
                writer.publish(np.full((240, 320, 3), 180, dtype=np.uint8),
                               dict(captured_monotonic_ns=time.monotonic_ns(), embodiment="test"))
                display.update_camera()
                display.render()
                self.assertEqual(display.status, "live")
                self.assertNotEqual(display.stats["camera_session"], old_session)
                np.testing.assert_allclose(display.readback()[240, 320, :3], [180]*3, atol=3)
            finally:
                writer.close()
                display.close()


if __name__ == "__main__":
    unittest.main()
