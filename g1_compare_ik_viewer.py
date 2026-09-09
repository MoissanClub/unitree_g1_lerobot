#!/usr/bin/env python3
"""Side-by-side MuJoCo visual check for G1-29 IK vs G1-23-constrained IK."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np

from g1_embodiments import active_arm_q_to_visual_29, make_arm_ik, visual_29_arm_joint_names
from rung3_xr_to_g1_mujoco import fk, make_ready_targets, solve_ready_q


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lerobot-root", default="/home/dwei/lerobot-sim/lerobot")
    parser.add_argument("--duration-s", type=float, default=0.0, help="<= 0 runs until Ctrl+C.")
    parser.add_argument("--control-hz", type=float, default=20.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--no-view", action="store_true", help="Run offscreen only; useful for smoke tests.")
    return parser.parse_args()


def rot_x(theta: float) -> np.ndarray:
    c = float(np.cos(theta))
    s = float(np.sin(theta))
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float)


def solve_home_ready(ik) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    model = ik.reduced_robot.model
    data = model.createData()
    ik.reduced_robot.data = data
    q0 = np.zeros(model.nq, dtype=float)
    left_home, right_home = fk(model, data, ik.L_hand_id, ik.R_hand_id, q0)
    left_ready, right_ready = make_ready_targets(left_home, right_home, 0.03, 0.12, 0.03)
    q_home, _ = ik.solve_ik(left_home, right_home, q0)
    q_ready = solve_ready_q(ik, left_ready, right_ready, np.asarray(q_home, dtype=float))
    return q_ready, left_ready, right_ready


def trajectory(left_ready: np.ndarray, right_ready: np.ndarray, t: float) -> tuple[np.ndarray, np.ndarray]:
    a = 2.0 * np.pi * t / 5.0
    left = left_ready.copy()
    right = right_ready.copy()
    left[:3, 3] += np.array([0.04 * np.sin(a), 0.01 * np.sin(0.5 * a), 0.02 * np.cos(a)])
    right[:3, 3] += np.array([0.04 * np.sin(a), -0.01 * np.sin(0.5 * a), 0.02 * np.cos(a)])
    left[:3, :3] = left[:3, :3] @ rot_x(0.45 * np.sin(a))
    right[:3, :3] = right[:3, :3] @ rot_x(-0.45 * np.sin(a))
    return left, right


class MujocoPanelModel:
    def __init__(self, xml_path: Path, width: int, height: int) -> None:
        import mujoco

        self.mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, height=height, width=width)
        self.width = width
        self.height = height
        self.arm_qpos_addr = []
        for joint_name in visual_29_arm_joint_names():
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if jid < 0:
                raise RuntimeError(f"MuJoCo joint not found: {joint_name}")
            self.arm_qpos_addr.append(int(self.model.jnt_qposadr[jid]))
        free_jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "floating_base_joint")
        self.free_qpos_addr = int(self.model.jnt_qposadr[free_jid]) if free_jid >= 0 else None
        self.camera = "global_view"
        self.reset()

    def reset(self) -> None:
        self.mujoco.mj_resetData(self.model, self.data)
        if self.free_qpos_addr is not None:
            adr = self.free_qpos_addr
            self.data.qpos[adr : adr + 7] = np.array([0.0, 0.0, 0.793, 1.0, 0.0, 0.0, 0.0])
        self.mujoco.mj_forward(self.model, self.data)

    def set_arm(self, q29: np.ndarray) -> None:
        for value, adr in zip(np.asarray(q29, dtype=float), self.arm_qpos_addr, strict=True):
            self.data.qpos[adr] = value
        self.mujoco.mj_forward(self.model, self.data)

    def render(self) -> np.ndarray:
        self.renderer.update_scene(self.data, camera=self.camera)
        return self.renderer.render()

    def close(self) -> None:
        self.renderer.close()


def annotate(img: np.ndarray, title: str) -> np.ndarray:
    from PIL import Image, ImageDraw

    pil = Image.fromarray(img)
    draw = ImageDraw.Draw(pil)
    draw.rectangle((0, 0, img.shape[1], 32), fill=(15, 18, 22))
    draw.text((12, 9), title, fill=(240, 240, 240))
    return np.asarray(pil)


def run_view(args: argparse.Namespace, frames) -> None:
    import tkinter as tk
    from PIL import Image, ImageTk

    root = tk.Tk()
    root.title("G1-29 vs G1-23 IK comparison")
    label = tk.Label(root)
    label.pack()
    state = {"index": 0}

    def tick() -> None:
        if state["index"] >= len(frames):
            if args.duration_s > 0.0:
                root.destroy()
                return
            state["index"] = 0
        frame = frames[state["index"]]
        state["index"] += 1
        photo = ImageTk.PhotoImage(Image.fromarray(frame))
        label.configure(image=photo)
        label.image = photo
        root.after(max(1, int(1000.0 / args.control_hz)), tick)

    tick()
    root.mainloop()


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(line_buffering=True)
    args = parse_args()
    os.environ.setdefault("MUJOCO_GL", "egl")
    lerobot_root = Path(args.lerobot_root).expanduser().resolve()
    if not lerobot_root.is_dir():
        print(f"LeRobot checkout not found: {lerobot_root}", file=sys.stderr)
        return 2
    if str(lerobot_root) not in sys.path:
        sys.path.insert(0, str(lerobot_root))

    from huggingface_hub import snapshot_download

    repo_path = Path(snapshot_download("lerobot/unitree-g1-mujoco"))
    xml_path = repo_path / "assets" / "scene_29dof.xml"
    if not xml_path.is_file():
        print(f"MuJoCo XML not found: {xml_path}", file=sys.stderr)
        return 2

    print("Building G1-29 IK")
    ik29 = make_arm_ik("g1_29", lerobot_root)
    print("Building G1-23 IK")
    ik23 = make_arm_ik("g1_23", lerobot_root)
    q29, left29, right29 = solve_home_ready(ik29)
    q23, left23, right23 = solve_home_ready(ik23)
    reorder29 = np.asarray(ik29._arm_reorder_pin_to_g1)
    reorder23 = np.asarray(ik23._arm_reorder_pin_to_g1)

    sim29 = MujocoPanelModel(xml_path, args.width, args.height)
    sim23 = MujocoPanelModel(xml_path, args.width, args.height)
    frames = []
    deadline_frames = int(args.duration_s * args.control_hz) if args.duration_s > 0.0 else int(5 * args.control_hz)
    deadline_frames = max(1, deadline_frames)
    print("Rendering comparison frames: left=G1-29 IK, right=G1-23-constrained IK")
    try:
        for i in range(deadline_frames):
            t = i / args.control_hz
            l29, r29 = trajectory(left29, right29, t)
            l23, r23 = trajectory(left23, right23, t)
            q29, _ = ik29.solve_ik(l29, r29, q29)
            q23, _ = ik23.solve_ik(l23, r23, q23)
            q29_visual = active_arm_q_to_visual_29(np.asarray(q29)[reorder29], "g1_29")
            q23_visual = active_arm_q_to_visual_29(np.asarray(q23)[reorder23], "g1_23")
            sim29.set_arm(q29_visual)
            sim23.set_arm(q23_visual)
            img29 = annotate(sim29.render(), "G1-29 IK")
            img23 = annotate(sim23.render(), "G1-23 constrained IK on G1-29 visual")
            frames.append(np.concatenate([img29, img23], axis=1))
        mean_pixel = float(np.mean(frames[-1]))
        print(f"comparison render ok: frames={len(frames)} final_mean_pixel={mean_pixel:.2f}")
        if args.no_view:
            return 0
        run_view(args, frames)
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 0
    finally:
        sim29.close()
        sim23.close()


if __name__ == "__main__":
    raise SystemExit(main())
