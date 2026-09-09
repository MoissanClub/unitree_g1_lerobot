#!/usr/bin/env python3
"""Run LeRobot Unitree G1 MuJoCo as a standalone DDS simulator with a Tk viewer."""

from __future__ import annotations

import argparse
import os
import sys
import time
import tkinter as tk
from pathlib import Path

import numpy as np
from PIL import Image, ImageTk

os.environ["MUJOCO_GL"] = os.environ.get("LEROBOT_MUJOCO_GL", "egl")

import unitree_sdk2py.core.channel as unitree_channel


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lerobot-root", default="/home/dwei/lerobot-sim/lerobot")
    parser.add_argument("--hz", type=float, default=250.0)
    parser.add_argument("--view-fps", type=float, default=20.0)
    parser.add_argument("--no-view", action="store_true", help="Run without the Tk/X viewer.")
    return parser.parse_args()


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


class TkViewer:
    def __init__(self, env, fps: float) -> None:
        self.env = env
        self.inner = env.simulator.sim_env
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
        self.root.mainloop()


def main() -> int:
    args = parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(line_buffering=True)

    sys.path.insert(0, str(Path(args.lerobot_root).expanduser().resolve()))
    patch_unitree_dds_config()
    patch_g1_hub_env_factory(enable_view=not args.no_view)

    from lerobot.envs import make_env

    print("Starting standalone G1 MuJoCo DDS sim on loopback")
    wrapper = make_env("lerobot/unitree-g1-mujoco", trust_remote_code=True)
    env = wrapper["hub_env"][0].envs[0]
    print("G1 MuJoCo DDS sim is running. Ctrl+C to stop.")

    try:
        if not args.no_view:
            print("Opening Tk/X viewer window")
            TkViewer(env, args.view_fps).run()
        else:
            period_s = 1.0 / args.hz
            steps = 0
            while True:
                t0 = time.perf_counter()
                env.step(None)
                steps += 1
                if steps % max(1, int(args.hz * 5)) == 0:
                    print(f"sim steps={steps}")
                time.sleep(max(0.0, period_s - (time.perf_counter() - t0)))
    except KeyboardInterrupt:
        print("\nStopping G1 MuJoCo DDS sim")
    finally:
        env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
