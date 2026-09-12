"""Require actual Isaac Teleop offscreen Vulkan/CUDA image delivery, without X or XR."""

from pathlib import Path
import tempfile
import time

import numpy as np

from lerobot.cameras.frame_channel import FrameWriter
from lerobot.teleoperators.xr_controllers.camera_display import (
    CameraDisplay,
    VideoConfig,
)


def main():
    with tempfile.TemporaryDirectory(prefix="g1-vr-graphics-") as folder:
        channel = Path(folder) / "camera.rgb"
        writer = FrameWriter(channel, 64, 48)
        display = None
        try:
            display = CameraDisplay(
                VideoConfig(
                    channel=str(channel),
                    width=320,
                    height=240,
                    expected_source="installation",
                    max_age_s=10,
                ),
                offscreen=True,
            )
            for brightness in (80, 180):
                writer.publish(
                    np.full((48, 64, 3), brightness, dtype=np.uint8),
                    {
                        "captured_monotonic_ns": time.monotonic_ns(),
                        "embodiment": "installation",
                    },
                )
                display.update_camera()
                display.render()
                np.testing.assert_allclose(
                    display.readback()[120, 160, :3], [brightness] * 3, atol=3
                )
            print(
                "PASS: actual offscreen Vulkan/CUDA pixel delivery (not headset acceptance)"
            )
        finally:
            if display is not None:
                display.close()
            writer.close()


if __name__ == "__main__":
    main()
