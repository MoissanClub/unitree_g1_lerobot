"""Robot-mounted camera rendering on private MuJoCo state in a spawned process."""
from dataclasses import asdict, dataclass
import fcntl
import multiprocessing as mp
from pathlib import Path
import queue
import tempfile
import time
import traceback

import numpy as np

from .camera_frames import FrameWriter, default_channel


@dataclass(frozen=True)
class CameraConfig:
    width: int = 640
    height: int = 480
    fps: float = 20.
    body: str = "torso_link"
    position: tuple = (0.10, 0., 0.38)
    pitch_deg: float = 35.
    fovy_deg: float = 90.
    camera_id: str = "robot_head"

    def __post_init__(self):
        if not (16 <= self.width <= 1920 and 16 <= self.height <= 1200):
            raise ValueError("Camera dimensions must be 16..1920 x 16..1200")
        if not (np.isfinite(self.fps) and 0 < self.fps <= 60):
            raise ValueError("Camera fps must be finite and in (0, 60]")
        if not (0 < self.fovy_deg < 170 and -89 < self.pitch_deg < 89):
            raise ValueError("Invalid camera field of view or downward pitch")


class RobotCamera:
    def __init__(self, model, config):
        import mujoco
        self.model, self.config = model, config
        self.body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, config.body)
        if self.body_id < 0:
            raise ValueError(f"Camera mount body missing: {config.body}")
        self.data = mujoco.MjData(model)
        model.vis.global_.offwidth = max(model.vis.global_.offwidth, config.width)
        model.vis.global_.offheight = max(model.vis.global_.offheight, config.height)
        model.vis.global_.fovy = config.fovy_deg
        self.renderer = mujoco.Renderer(model, height=config.height, width=config.width)
        self.option = mujoco.MjvOption()
        self.option.geomgroup[3] = 0
        self.option.sitegroup[:] = 0

    def render(self, state):
        import mujoco
        self.data.qpos[:] = state["qpos"]
        self.data.mocap_pos[:] = state["mocap_pos"]
        self.data.mocap_quat[:] = state["mocap_quat"]
        self.data.time = state["simulation_time_s"]
        mujoco.mj_forward(self.model, self.data)
        rotation = self.data.xmat[self.body_id].reshape(3, 3)
        eye = self.data.xpos[self.body_id] + rotation @ np.array(self.config.position)
        angle = np.deg2rad(self.config.pitch_deg)
        forward = rotation @ np.array([np.cos(angle), 0., -np.sin(angle)])
        up = rotation @ np.array([np.sin(angle), 0., np.cos(angle)])
        self.renderer.update_scene(self.data, scene_option=self.option)
        # Specify both GL eyes identically: a real mono body-mounted optical pose,
        # independent of the spectator viewer and including body roll/pitch/yaw.
        for camera in self.renderer.scene.camera:
            camera.pos[:] = eye
            camera.forward[:] = forward
            camera.up[:] = up
        return self.renderer.render().copy(), dict(position_world=eye.tolist(),
                                                   forward_world=forward.tolist(), up_world=up.tolist())

    def close(self):
        self.renderer.close()


def render_worker(model_path, config, channel, embodiment, states, stop, ready, errors):
    import os
    os.environ["MUJOCO_GL"] = os.environ.get("LEROBOT_MUJOCO_GL", "egl")
    import mujoco
    camera = writer = None
    try:
        model = mujoco.MjModel.from_binary_path(model_path)
        camera = RobotCamera(model, config)
        writer = FrameWriter(channel, config.width, config.height)
        while not stop.is_set():
            try:
                state = states.get(timeout=0.1)
            except queue.Empty:
                continue
            started = time.monotonic_ns()
            pixels, pose = camera.render(state)
            writer.publish(pixels, dict(camera_id=config.camera_id, embodiment=embodiment,
                captured_monotonic_ns=state["captured_monotonic_ns"],
                simulation_time_s=state["simulation_time_s"], render_ms=(time.monotonic_ns()-started)/1e6,
                optical_pose=pose, mount=asdict(config)))
            ready.set()
    except BaseException:
        errors.put(traceback.format_exc())
        ready.set()
    finally:
        try:
            if camera is not None:
                camera.close()
        finally:
            if writer is not None:
                writer.close()


class CameraPublisher:
    """Physics calls offer() after a step; only copies small state and never waits."""
    def __init__(self, model, data, config, embodiment, channel=None):
        import mujoco
        self.config = config
        self.channel = Path(channel or default_channel())
        self.context = mp.get_context("spawn")
        self.states = self.context.Queue(maxsize=1)
        self.errors = self.context.Queue()
        self.stop = self.context.Event()
        ready = self.context.Event()
        self.folder = tempfile.TemporaryDirectory(prefix="g1-camera-model-")
        model_path = str(Path(self.folder.name) / "model.mjb")
        mujoco.mj_saveModel(model, model_path, None)
        self.last_offer = -float("inf")
        self.failed = False
        self.closed = False
        self.channel_inode = None
        self.process = self.context.Process(target=render_worker, name="robot-camera-render",
            args=(model_path, config, str(self.channel), embodiment, self.states, self.stop, ready, self.errors))
        self.process.start()
        try:
            self.offer(data)
            if not ready.wait(30):
                raise TimeoutError("Robot camera renderer did not start within 30 seconds")
            try:
                error = self.errors.get(timeout=0.1)
            except queue.Empty:
                error = None
            if error or not self.process.is_alive():
                raise RuntimeError(error or "Robot camera renderer exited at startup")
            self.channel_inode = self.channel.stat().st_ino
        except BaseException:
            self.close()
            raise
        print(f"Robot camera ready: {embodiment}/{config.camera_id} RGB8 {config.width}x{config.height} "
              f"at up to {config.fps:g} fps; channel={self.channel}", flush=True)

    def offer(self, data):
        now = time.monotonic()
        if self.failed or now - self.last_offer < 1 / self.config.fps:
            return
        if not self.process.is_alive():
            self.failed = True
            print("Robot camera renderer stopped; physics continues without video.", flush=True)
            return
        state = dict(qpos=data.qpos.copy(), mocap_pos=data.mocap_pos.copy(),
                     mocap_quat=data.mocap_quat.copy(), simulation_time_s=float(data.time),
                     captured_monotonic_ns=time.monotonic_ns())
        try:
            self.states.put_nowait(state)
        except queue.Full:
            pass
        self.last_offer = now

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.stop.set()
        self.process.join(timeout=5)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=3)
        if self.process.is_alive():
            self.process.kill()
            self.process.join()
        self.states.cancel_join_thread()
        self.states.close()
        self.errors.close()
        self.folder.cleanup()
        # A killed renderer cannot unlink its channel. Do not remove a replacement
        # producer's file if another simulator has already acquired the channel.
        if self.channel_inode is not None:
            with open(str(self.channel) + ".owner", "rb") as owner:
                try:
                    fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    if self.channel.exists() and self.channel.stat().st_ino == self.channel_inode:
                        self.channel.unlink()
                except BlockingIOError:
                    pass
