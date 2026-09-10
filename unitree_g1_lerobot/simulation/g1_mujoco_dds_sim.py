#!/usr/bin/env python3
"""Run LeRobot Unitree G1 MuJoCo as a standalone DDS simulator with a Tk viewer."""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from PIL import Image, ImageTk

os.environ["MUJOCO_GL"] = os.environ.get("LEROBOT_MUJOCO_GL", "egl")

import unitree_sdk2py.core.channel as unitree_channel


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lerobot-root", default="/home/dwei/lerobot-sim/lerobot")
    parser.add_argument("--embodiment", choices=("g1_29", "g1_23"), default="g1_29")
    parser.add_argument("--duration-s", type=float, default=0, help="Stop after N seconds; 0 runs until interrupted.")
    parser.add_argument("--save-frame", type=Path, help="Save the native G1-23 viewer's latest frame.")
    parser.add_argument("--hz", type=float, default=250.0)
    parser.add_argument("--view-fps", type=float, default=20.0)
    parser.add_argument("--no-view", action="store_true", help="Run without the Tk/X viewer.")
    parser.add_argument("--headless", action="store_true", help="No viewer or confirmation prompts; still run diagnostics.")
    parser.add_argument("--skip-startup-diagnostic", action="store_true", help="Skip the raise-arm startup diagnostic.")
    parser.add_argument("--diagnostic-duration-s", type=float, default=2.0)
    parser.add_argument("--no-diagnostic-confirm", action="store_true", help="Do not pause for visual confirmation after the startup diagnostic.")
    args = parser.parse_args()
    if args.headless:
        args.no_view = True
        args.no_diagnostic_confirm = True
    if not all(np.isfinite(x) for x in (args.hz, args.view_fps, args.duration_s, args.diagnostic_duration_s)) or min(args.hz, args.view_fps, args.diagnostic_duration_s) <= 0 or args.duration_s < 0:
        parser.error("Rates and diagnostic duration must be positive and finite; duration must be nonnegative")
    if args.embodiment == "g1_23" and args.hz != 250:
        parser.error("Native G1-23 uses 250 Hz DDS/control with 500 Hz physics")
    return args


def patch_unitree_dds_config() -> None:
    unitree_channel.ChannelConfigHasInterface = """<?xml version=\"1.0\" encoding=\"UTF-8\" ?>
<CycloneDDS>
    <Domain Id=\"any\">
        <General>
            <Interfaces>
                <NetworkInterface name=\"$__IF_NAME__$\" priority=\"default\" multicast=\"default\"/>
            </Interfaces>
        </General>
    </Domain>
</CycloneDDS>"""


def patch_g1_hub_env_factory(enable_view: bool) -> None:
    import lerobot.envs.factory as env_factory

    original_call_make_env = env_factory._call_make_env

    def call_make_env(module, n_envs, use_async_envs, cfg):
        module_file = str(getattr(module, "__file__", ""))
        if "models--lerobot--unitree-g1-mujoco" not in module_file:
            return original_call_make_env(module, n_envs, use_async_envs, cfg)

        original_safe_load = module.yaml.safe_load

        def safe_load_for_mode(stream):
            config = original_safe_load(stream)
            config["ENABLE_ONSCREEN"] = False
            config["ENABLE_OFFSCREEN"] = enable_view
            return config

        module.yaml.safe_load = safe_load_for_mode
        cameras = ["head_camera"] if enable_view else []
        try:
            env = module.make_env(
                n_envs=n_envs,
                use_async_envs=use_async_envs,
                publish_images=False,
                cameras=cameras,
            )
            if enable_view:

                def step_without_render(action=None):
                    env.sim_env.sim_step()
                    env.step_count += 1
                    obs = env._get_obs()
                    return obs, 0.0, False, False, {}

                env.step = step_without_render
            return env
        finally:
            module.yaml.safe_load = original_safe_load

    env_factory._call_make_env = call_make_env


def run_raise_arm_diagnostic(env, args: argparse.Namespace, *, drive_steps: bool) -> int:
    if args.skip_startup_diagnostic or os.environ.get("SKIP_G1_STARTUP_DIAGNOSTIC", "").strip() == "1":
        return 0

    from unitree_g1_lerobot.diagnostics.g1_startup_diagnostic import run_diagnostic

    stop_event = threading.Event()
    thread = None

    def step_loop() -> None:
        period_s = 1.0 / args.hz
        while not stop_event.is_set():
            t0 = time.perf_counter()
            env.step(None)
            time.sleep(max(0.0, period_s - (time.perf_counter() - t0)))

    if drive_steps:
        thread = threading.Thread(target=step_loop, daemon=True)
        thread.start()
    try:
        diag_args = SimpleNamespace(
            mode="raise",
            lerobot_root=args.lerobot_root,
            duration_s=args.diagnostic_duration_s,
            hz=min(args.hz, 60.0),
            g1_state_timeout_s=10.0,
            ready_x_m=0.03,
            ready_z_m=0.12,
            ready_spread_m=0.03,
            orientation_deg=35.0,
            hands_up_deg=90.0,
            hands_up_direction="inward",
            optional=False,
            confirm=not args.no_diagnostic_confirm,
        )
        return run_diagnostic(diag_args)
    finally:
        stop_event.set()
        if thread is not None:
            thread.join(timeout=2.0)


class TkViewer:
    def __init__(self, env, fps: float, args: argparse.Namespace) -> None:
        self.env = env
        self.args = args
        self.inner = env.simulator.sim_env
        self.diagnostic_thread = None
        self.dt_ms = max(1, int(1000.0 / fps))
        self.steps = 0
        self.photo = None
        self.closed = False

        self.root = tk.Tk()
        self.root.title("G1 MuJoCo DDS Sim")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.image_label = tk.Label(self.root)
        self.image_label.pack()
        self.status = tk.StringVar()
        tk.Label(self.root, textvariable=self.status, font=("TkDefaultFont", 12), justify="left").pack(
            fill="x", padx=8, pady=6
        )
        self.root.after(self.dt_ms, self.tick)
        self.root.after(800, self.start_startup_diagnostic)
        if args.duration_s:
            self.root.after(round(args.duration_s * 1000), self.close)

    def start_startup_diagnostic(self) -> None:
        if self.closed or self.diagnostic_thread is not None:
            return

        def run() -> None:
            rc = run_raise_arm_diagnostic(self.env, self.args, drive_steps=False)
            if rc != 0:
                print(f"Startup diagnostic failed with exit code {rc}", file=sys.stderr, flush=True)
                self.root.after(0, self.close)

        self.diagnostic_thread = threading.Thread(target=run, daemon=True)
        self.diagnostic_thread.start()

    def tick(self) -> None:
        if self.closed:
            return
        self.env.step(None)
        self.steps += 1
        self.update_image()
        self.root.after(self.dt_ms, self.tick)

    def update_image(self) -> None:
        frames = self.inner.update_render_caches()
        image_array = frames.get("global_view_image")
        if image_array is None:
            image_array = frames.get("head_camera_image")
        if image_array is None:
            self.status.set("G1 MuJoCo DDS sim running; no camera frame yet")
            return
        image = Image.fromarray(np.asarray(image_array)).resize((720, 720), Image.Resampling.NEAREST)
        self.photo = ImageTk.PhotoImage(image)
        self.image_label.configure(image=self.photo)
        self.status.set(f"G1 MuJoCo DDS sim running   steps={self.steps}   close window or Ctrl+C to stop")

    def close(self) -> None:
        self.closed = True
        self.root.destroy()

    def run(self) -> None:
        try:
            self.root.mainloop()
        finally:
            # Hub renderers must be freed in the Tk thread before EGL teardown.
            for renderer in self.inner.renderers.values():
                renderer.close()
            self.inner.renderers.clear()


def main() -> int:
    args = parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(line_buffering=True)

    sys.path.insert(0, str(Path(args.lerobot_root).expanduser().resolve()))
    patch_unitree_dds_config()
    if args.embodiment == "g1_23":
        from .native_g1 import NativeG1Simulation
        from .native_g1_viewer import run_native
        args.skip_startup_diagnostic |= os.environ.get("SKIP_G1_STARTUP_DIAGNOSTIC", "").strip() == "1"
        root = None
        if not args.no_view:
            try:
                root = tk.Tk()
                root.withdraw()
            except tk.TclError as exc:
                raise SystemExit(f"Cannot open Tk/X on DISPLAY={os.environ.get('DISPLAY')!r}. Use ssh -Y or --no-view. {exc}") from exc
        print("Starting native g1_23 on loopback DDS: 10 dynamic arm joints; pelvis, legs and waist supported.", flush=True)
        env = None
        try:
            env = NativeG1Simulation()
            run_native(env, args, root)
        except KeyboardInterrupt:
            print("Stopping G1-23 simulator", flush=True)
        finally:
            if env is not None:
                env.close()
            if root is not None:
                try:
                    root.destroy()
                except tk.TclError:
                    pass
        return 0
    patch_g1_hub_env_factory(enable_view=not args.no_view)

    from lerobot.envs import make_env
    from .native_g1 import acquire_simulator_lock
    from .dds import start_simulator_identity

    print("Starting standalone G1 MuJoCo DDS sim on loopback")
    process_lock = acquire_simulator_lock()
    try:
        wrapper = make_env("lerobot/unitree-g1-mujoco", trust_remote_code=True)
        env = wrapper["hub_env"][0].envs[0]
        stop_identity = start_simulator_identity("g1_29")
    except BaseException:
        process_lock.close()
        raise
    print("G1 MuJoCo DDS sim is running. Ctrl+C to stop.")

    try:
        if not args.no_view:
            print("Opening Tk/X viewer window before startup diagnostic")
            TkViewer(env, args.view_fps, args).run()
        else:
            diagnostic_rc = run_raise_arm_diagnostic(env, args, drive_steps=True)
            if diagnostic_rc != 0:
                return diagnostic_rc
            period_s = 1.0 / args.hz
            steps = 0
            started = time.monotonic()
            while True:
                if args.duration_s and time.monotonic() - started >= args.duration_s:
                    break
                t0 = time.perf_counter()
                env.step(None)
                steps += 1
                if steps % max(1, int(args.hz * 5)) == 0:
                    print(f"sim steps={steps}")
                time.sleep(max(0.0, period_s - (time.perf_counter() - t0)))
    except KeyboardInterrupt:
        print("\nStopping G1 MuJoCo DDS sim")
    finally:
        stop_identity()
        env.close()
        process_lock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
