"""One isolated graphics/input session with a nonblocking controller mailbox."""
from contextlib import ExitStack
import multiprocessing as mp
import queue
import time
import traceback

import numpy as np

from .camera_display import CameraDisplay


class XRVideoSessionEnded(RuntimeError):
    reconnectable = True


def video_worker(controller_config, video_config, frames, stop, ready, errors):
    display = None
    try:
        from isaacteleop.oxr import OpenXRSessionHandles
        from isaacteleop.teleop_session_manager import TeleopSession, TeleopSessionConfig, get_required_oxr_extensions_from_pipeline
        from .both_controllers import BothXRControllers
        with ExitStack() as cleanup:
            reader = BothXRControllers(controller_config)
            reader._ensure_cloudxr_runtime()
            cleanup.callback(reader._stop_cloudxr_runtime)
            pipeline = reader._build_pipeline()
            display = CameraDisplay(video_config, get_required_oxr_extensions_from_pipeline(pipeline))
            cleanup.callback(display.close)
            handles = OpenXRSessionHandles(*display.session.get_oxr_handles())
            session = cleanup.enter_context(TeleopSession(TeleopSessionConfig(
                app_name=controller_config.app_name, pipeline=pipeline, oxr_handles=handles)))
            reader._session = session
            reader._external_inputs = reader._build_external_inputs()
            print("XR video connected: one graphics session shared with both controllers", flush=True)
            ready.set()
            while not stop.is_set():
                started = time.monotonic()
                if display.session.should_close():
                    raise XRVideoSessionEnded("XR video session ended; reconnect required")
                display.update_camera()
                display.render()
                action = reader.get_action()
                snapshot = dict(action=action, tracking=dict(reader.tracking_by_hand),
                                captured=time.monotonic(), video=dict(display.stats, status=display.status))
                try:
                    frames.put_nowait(snapshot)
                except queue.Full:
                    pass
                stop.wait(max(0., 1/90 - (time.monotonic()-started)))
    except BaseException as exc:
        if not stop.is_set():
            errors.put((isinstance(exc, XRVideoSessionEnded), traceback.format_exc()))
        ready.set()
    finally:
        if display is not None:
            print(f"XR video stopped: {display.stats}", flush=True)


class VideoXRController:
    """Bridge adapter: rendering/polling never executes in the IK/DDS control loop."""
    def __init__(self, config, video_config, hand_side="both", mock=None):
        self.config, self.video_config = config, video_config
        self.hand_side, self.mock = hand_side, mock
        self.process = None
        self.snapshot = None
        self.tracking_by_hand = dict(left=False, right=False)
        self.is_tracking = False
        self.video_stats = {}

    def connect(self):
        if self.process is not None:
            raise RuntimeError("Already connected")
        context = mp.get_context("spawn")
        self.frames = context.Queue(maxsize=1)
        self.errors = context.Queue()
        self.stop = context.Event()
        ready = context.Event()
        self.snapshot = None
        self.process = context.Process(target=video_worker, name="xr-camera-and-input",
            args=(self.config, self.video_config, self.frames, self.stop, ready, self.errors))
        self.process.start()
        try:
            if not ready.wait(30):
                raise TimeoutError("XR video session did not initialize in 30 seconds")
            self._check_error(wait=.1)
        except BaseException:
            self.disconnect()
            raise

    def _check_error(self, wait=0):
        try:
            reconnectable, error = self.errors.get(timeout=wait) if wait else self.errors.get_nowait()
        except queue.Empty:
            if self.process is None or not self.process.is_alive():
                raise XRVideoSessionEnded("XR video worker exited")
        else:
            raise (XRVideoSessionEnded if reconnectable else RuntimeError)(error)

    def get_action(self):
        self._check_error()
        try:
            self.snapshot = self.frames.get_nowait()
        except queue.Empty:
            pass
        fresh = self.snapshot is not None and time.monotonic()-self.snapshot["captured"] < .25
        self.tracking_by_hand = dict(self.snapshot["tracking"]) if fresh else dict(left=False, right=False)
        self.is_tracking = any(self.tracking_by_hand.values())
        if self.snapshot:
            self.video_stats = self.snapshot["video"]
        if self.mock is not None:
            self.tracking_by_hand = dict(left=True, right=True)
            self.is_tracking = True
            return self.mock.get_action()
        if fresh:
            action = self.snapshot["action"]
        else:
            action = {f"{side}.{key}": value for side in ("left", "right")
                      for key, value in dict(grip_pos=np.zeros(3), grip_quat=np.array([0.,0.,0.,1.]),
                                             squeeze=0., trigger=0.).items()}
        if self.hand_side == "both":
            return action
        self.is_tracking = self.tracking_by_hand[self.hand_side]
        return {key: action[f"{self.hand_side}.{key}"] for key in ("grip_pos", "grip_quat", "squeeze", "trigger")}

    def disconnect(self):
        if self.process is None:
            return
        self.stop.set()
        self.process.join(timeout=5)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=3)
        if self.process.is_alive():
            self.process.kill()
            self.process.join()
        self.frames.cancel_join_thread()
        self.frames.close()
        self.errors.close()
        self.process = None
        self.is_tracking = False
        self.tracking_by_hand = dict(left=False, right=False)
