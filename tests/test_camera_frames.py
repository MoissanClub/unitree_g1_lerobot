"""Frame protocol checks without a simulator, DDS, display, or XR runtime."""
import fcntl
import os
import mmap
from pathlib import Path
import tempfile
import time
import unittest

import numpy as np

from unitree_g1_lerobot.simulation.camera_frames import FrameWriter, read_frame
from unitree_g1_lerobot.simulation.robot_camera import CameraConfig


class CameraFramesTests(unittest.TestCase):
    def test_roundtrip_staleness_restart_and_exclusive_producer(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "camera.rgb"
            self.assertIsNone(read_frame(path))
            writer = FrameWriter(path, 32, 24)
            try:
                self.assertIsNone(read_frame(path))
                pixels = np.arange(32*24*3, dtype=np.uint8).reshape(24, 32, 3)
                writer.publish(pixels, dict(captured_monotonic_ns=time.monotonic_ns(), camera_id="head"))
                metadata, actual = read_frame(path)
                np.testing.assert_array_equal(actual, pixels)
                self.assertEqual(metadata["sequence"], 1)
                with self.assertRaises(BlockingIOError):
                    FrameWriter(path, 16, 16)
                np.testing.assert_array_equal(read_frame(path)[1], pixels)
                writer.publish(pixels, dict(captured_monotonic_ns=time.monotonic_ns()-2_000_000_000))
                self.assertIsNone(read_frame(path))
            finally:
                writer.close()
            self.assertIsNone(read_frame(path))
            second = FrameWriter(path, 16, 16)
            try:
                second.publish(np.zeros((16, 16, 3), dtype=np.uint8), dict(captured_monotonic_ns=time.monotonic_ns()))
                self.assertNotEqual(read_frame(path)[0]["session_id"], metadata["session_id"])
            finally:
                second.close()

    def test_restart_does_not_truncate_existing_reader_mapping(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "camera.rgb"
            writer = FrameWriter(path, 64, 48)
            writer.publish(np.full((48, 64, 3), 73, dtype=np.uint8),
                           dict(captured_monotonic_ns=time.monotonic_ns()))
            fd = os.open(path, os.O_RDONLY)
            old = mmap.mmap(fd, 0, access=mmap.ACCESS_READ)
            writer.close(unlink=False)
            replacement = FrameWriter(path, 16, 16)
            try:
                self.assertEqual(old[-1], 73)
                replacement.publish(np.zeros((16, 16, 3), dtype=np.uint8),
                                    dict(captured_monotonic_ns=time.monotonic_ns()))
                self.assertEqual(read_frame(path)[1].shape, (16, 16, 3))
            finally:
                replacement.close()
                old.close()
                os.close(fd)

    def test_busy_frame_skipped_and_bad_pixels_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "camera.rgb"
            writer = FrameWriter(path, 16, 16)
            fd = os.open(path, os.O_RDONLY)
            try:
                fcntl.flock(fd, fcntl.LOCK_SH)
                self.assertFalse(writer.publish(np.zeros((16, 16, 3), dtype=np.uint8), {}))
                fcntl.flock(fd, fcntl.LOCK_UN)
                with self.assertRaises(ValueError):
                    writer.publish(np.zeros((16, 16, 3)), {})
                fcntl.flock(writer.fd, fcntl.LOCK_EX)
                self.assertIsNone(read_frame(path))
                fcntl.flock(writer.fd, fcntl.LOCK_UN)
            finally:
                os.close(fd)
                writer.close()

    def test_config_limits(self):
        for kwargs in (dict(width=0), dict(fps=float("nan")), dict(fps=100),
                       dict(pitch_deg=90), dict(fovy_deg=180)):
            with self.assertRaises(ValueError):
                CameraConfig(**kwargs)


if __name__ == "__main__":
    unittest.main()
