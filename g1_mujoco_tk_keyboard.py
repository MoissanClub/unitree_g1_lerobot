#!/usr/bin/env python3
"""SSH-X friendly Tk keyboard viewer/controller for LeRobot Unitree G1 MuJoCo.

This avoids MuJoCo's GLFW onscreen viewer, which often fails over ssh -Y/ssh -X.
MuJoCo renders offscreen with EGL; Tk displays the RGB frames and owns keyboard
focus.
"""

from __future__ import annotations

import os
import time
import tkinter as tk
from dataclasses import dataclass, field

os.environ["MUJOCO_GL"] = os.environ.get("LEROBOT_MUJOCO_GL", "egl")

import numpy as np
import pinocchio as pin
import unitree_sdk2py.core.channel as unitree_channel
from PIL import Image, ImageTk

from lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config
from lerobot.robots.unitree_g1.g1_kinematics import G1_29_ArmIK
from lerobot.robots.unitree_g1.g1_utils import G1_29_JointArmIndex, G1_29_JointIndex


def patch_unitree_dds_config() -> None:
    unitree_channel.ChannelConfigHasInterface = """<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
    <Domain Id="any">
        <General>
            <Interfaces>
                <NetworkInterface name="$__IF_NAME__$" priority="default" multicast="default"/>
            </Interfaces>
        </General>
    </Domain>
</CycloneDDS>"""


def patch_g1_hub_env_factory() -> None:
    import lerobot.envs.factory as env_factory

    original_call_make_env = env_factory._call_make_env

    def call_make_env_offscreen(module, n_envs, use_async_envs, cfg):
        module_file = str(getattr(module, "__file__", ""))
        if "models--lerobot--unitree-g1-mujoco" not in module_file:
            return original_call_make_env(module, n_envs, use_async_envs, cfg)

        original_safe_load = module.yaml.safe_load

        def safe_load_offscreen(stream):
            config = original_safe_load(stream)
            config["ENABLE_ONSCREEN"] = False
            config["ENABLE_OFFSCREEN"] = True
            return config

        module.yaml.safe_load = safe_load_offscreen
        try:
            env = module.make_env(
                n_envs=n_envs,
                use_async_envs=use_async_envs,
                publish_images=False,
                cameras=["head_camera"],
            )
            original_step = env.step

            def step_without_render(action=None):
                env.sim_env.sim_step()
                env.step_count += 1
                obs = env._get_obs()
                return obs, 0.0, False, False, {}

            env.step = step_without_render
            env._step_with_render = original_step
            return env
        finally:
            module.yaml.safe_load = original_safe_load

    env_factory._call_make_env = call_make_env_offscreen


def build_action(q_g1: np.ndarray) -> dict[str, float]:
    action = {f"{j.name}.q": 0.0 for j in G1_29_JointIndex}
    action.update({f"{j.name}.q": float(q_g1[k]) for k, j in enumerate(G1_29_JointArmIndex)})
    return action


@dataclass
class ControlState:
    pressed: set[str] = field(default_factory=set)
    steps: int = 0
    last_key: str = "none"
    target_offset: np.ndarray = field(default_factory=lambda: np.zeros(3))


class G1MujocoTkKeyboard:
    def __init__(self, fps: int = 20) -> None:
        self.fps = fps
        self.dt_ms = int(1000 / fps)
        self.hand_step_m = 0.006
        self.state = ControlState()
        self.photo = None
        self.closed = False

        patch_unitree_dds_config()
        patch_g1_hub_env_factory()

        self.ik = G1_29_ArmIK()
        self.model = self.ik.reduced_robot.model
        self.data = self.model.createData()
        self.ik.reduced_robot.data = self.data
        pin.forwardKinematics(self.model, self.data, np.zeros(self.model.nq))
        pin.updateFramePlacements(self.model, self.data)
        self.left_rest = self.data.oMf[self.ik.L_hand_id].homogeneous.copy()
        self.right_rest = self.data.oMf[self.ik.R_hand_id].homogeneous.copy()
        self.reorder = np.asarray(self.ik._arm_reorder_pin_to_g1)
        self.q_pin = np.zeros(self.model.nq)

        self.robot = UnitreeG1(UnitreeG1Config(is_simulation=True))
        print("connecting G1 MuJoCo sim...", flush=True)
        self.robot.connect()
        print("connected. creating Tk window...", flush=True)

        self.env = self.robot.sim_env
        self.inner = self.env.simulator.sim_env

        self.root = tk.Tk()
        self.root.title("G1 MuJoCo Keyboard")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.bind_all("<KeyPress>", self.on_key_press)
        self.root.bind_all("<KeyRelease>", self.on_key_release)

        self.image_label = tk.Label(self.root, takefocus=True)
        self.image_label.pack()
        self.image_label.bind("<Button-1>", self.focus_input)
        self.status = tk.StringVar()
        tk.Label(self.root, textvariable=self.status, font=("TkDefaultFont", 12), justify="left").pack(
            fill="x", padx=8, pady=6
        )

        self.root.after(250, self.focus_input)
        self.root.after(self.dt_ms, self.tick)

    def focus_input(self, _event: tk.Event | None = None) -> None:
        self.root.lift()
        self.root.focus_force()
        self.image_label.focus_force()

    def on_key_press(self, event: tk.Event) -> None:
        key = event.keysym
        self.state.last_key = key
        print(f"key press: {key}", flush=True)
        if key in {"Escape", "q"}:
            self.close()
            return
        if key == "r":
            self.reset_targets()
            return
        self.state.pressed.add(key)

    def on_key_release(self, event: tk.Event) -> None:
        self.state.last_key = f"{event.keysym} released"
        self.state.pressed.discard(event.keysym)

    def reset_targets(self) -> None:
        self.state.target_offset[:] = 0.0
        self.q_pin[:] = 0.0
        self.robot.send_action(build_action(np.zeros(len(G1_29_JointArmIndex))))

    def update_targets_from_keys(self) -> None:
        keys = self.state.pressed
        delta = np.zeros(3)
        if "w" in keys:
            delta[0] += self.hand_step_m
        if "s" in keys:
            delta[0] -= self.hand_step_m
        if "a" in keys:
            delta[1] += self.hand_step_m
        if "d" in keys:
            delta[1] -= self.hand_step_m
        if "e" in keys:
            delta[2] += self.hand_step_m
        if "c" in keys:
            delta[2] -= self.hand_step_m
        self.state.target_offset[:] = np.clip(self.state.target_offset + delta, [-0.10, -0.12, -0.10], [0.10, 0.12, 0.16])

    def tick(self) -> None:
        if self.closed:
            return

        t0 = time.perf_counter()
        keys = self.state.pressed
        for key, sim_key in (("Up", "up"), ("Down", "down"), ("Left", "left"), ("Right", "right")):
            if key in keys:
                self.inner.handle_keyboard_button(sim_key)

        self.update_targets_from_keys()
        left_target = self.left_rest.copy()
        right_target = self.right_rest.copy()
        left_target[:3, 3] += self.state.target_offset
        right_target[:3, 3] += self.state.target_offset
        self.q_pin, _ = self.ik.solve_ik(left_target, right_target, self.q_pin)
        self.robot.send_action(build_action(np.asarray(self.q_pin)[self.reorder]))

        self.state.steps += 1
        self.update_image()

        elapsed_ms = int((time.perf_counter() - t0) * 1000)
        self.root.after(max(1, self.dt_ms - elapsed_ms), self.tick)

    def update_image(self) -> None:
        frames = self.inner.update_render_caches()
        image_array = frames.get("global_view_image")
        if image_array is None:
            image_array = frames.get("head_camera_image")
        if image_array is None:
            self.status.set("No camera frame yet. Press q/Esc to quit.")
            return

        image = Image.fromarray(image_array).resize((720, 720), Image.Resampling.NEAREST)
        self.photo = ImageTk.PhotoImage(image)
        self.image_label.configure(image=self.photo)

        controls = "Click image first. Arrows: shove base   w/s/a/d/e/c: move both hands   r: reset arms   q/Esc: quit"
        self.status.set(
            f"{controls}\n"
            f"steps={self.state.steps}  offset_m={np.array2string(self.state.target_offset, precision=3)}\n"
            f"last_key={self.state.last_key}  pressed={sorted(self.state.pressed)}"
        )

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        try:
            self.robot.disconnect()
        finally:
            self.root.destroy()
            os._exit(0)

    def run(self) -> None:
        print("G1 MuJoCo Tk keyboard controller started")
        print("Click the image once, then use arrows for base perturbation and w/s/a/d/e/c for both hands.")
        self.root.mainloop()


def main() -> None:
    G1MujocoTkKeyboard().run()


if __name__ == "__main__":
    main()
