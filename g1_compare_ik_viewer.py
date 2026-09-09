#!/usr/bin/env python3
"""Side-by-side MuJoCo visual check for G1-29 IK vs G1-23-constrained IK."""

from __future__ import annotations

import argparse
import os
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from g1_embodiments import G1_23_SPEC, active_arm_q_to_visual_29, make_arm_ik, visual_29_arm_joint_names
from rung3_xr_to_g1_mujoco import fk, make_ready_targets, solve_ready_q


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lerobot-root", default="/home/dwei/lerobot-sim/lerobot")
    parser.add_argument("--duration-s", type=float, default=0.0, help="<= 0 runs until Ctrl+C.")
    parser.add_argument("--control-hz", type=float, default=20.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--no-view", action="store_true", help="Run offscreen only; useful for smoke tests.")
    parser.add_argument("--save-final-frame", default=None, help="Optional PNG path for the final side-by-side frame.")
    parser.add_argument("--save-phase-frames", type=Path, help="Save sweep extrema as PNGs for visual review.")
    parser.add_argument(
        "--right-visual",
        choices=("g1_23_native", "g1_29_mapped"),
        default="g1_23_native",
        help="Right panel backend. g1_23_native compiles the G1-23 URDF to MuJoCo at runtime.",
    )
    parser.add_argument(
        "--motion-profile",
        choices=("static", "basic", "step4_sweep"),
        default="step4_sweep",
        help="Scripted target motion. basic preserves the Step 3 mapped-verifier motion.",
    )
    args = parser.parse_args()
    if not np.isfinite(args.control_hz) or args.control_hz <= 0 or not np.isfinite(args.duration_s):
        parser.error("control-hz must be positive and duration-s must be finite")
    return args


def rot_x(theta: float) -> np.ndarray:
    c = float(np.cos(theta))
    s = float(np.sin(theta))
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float)


def rot_y(theta: float) -> np.ndarray:
    c = float(np.cos(theta))
    s = float(np.sin(theta))
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)


def rot_z(theta: float) -> np.ndarray:
    c = float(np.cos(theta))
    s = float(np.sin(theta))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


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


def trajectory(
    left_ready: np.ndarray,
    right_ready: np.ndarray,
    t: float,
    motion_profile: str,
) -> tuple[np.ndarray, np.ndarray, str]:
    left = left_ready.copy()
    right = right_ready.copy()
    if motion_profile == "static":
        return left, right, "URDF / mesh: neutral pose"
    if motion_profile == "basic":
        a = 2.0 * np.pi * t / 5.0
        left[:3, 3] += np.array([0.04 * np.sin(a), 0.01 * np.sin(0.5 * a), 0.02 * np.cos(a)])
        right[:3, 3] += np.array([0.04 * np.sin(a), -0.01 * np.sin(0.5 * a), 0.02 * np.cos(a)])
        left[:3, :3] = left[:3, :3] @ rot_x(0.45 * np.sin(a))
        right[:3, :3] = right[:3, :3] @ rot_x(-0.45 * np.sin(a))
        return left, right, "basic xyz/orientation"

    cycle_s = 24.0
    phase_t = t % cycle_s
    phase = int(phase_t // 6.0)
    u = (phase_t % 6.0) / 6.0
    wave = np.sin(2.0 * np.pi * u)
    if phase == 0:
        delta = np.array([0.0, 0.0, 0.12 * wave])
        label = "up / down"
    elif phase == 1:
        delta = np.array([0.12 * wave, 0.0, 0.0])
        label = "forward / back"
    elif phase == 2:
        delta = np.array([0.0, 0.10 * wave, 0.0])
        label = "left / right"
    else:
        delta = np.zeros(3)
        roll = 0.65 * np.sin(2.0 * np.pi * u)
        pitch = 0.45 * np.sin(4.0 * np.pi * u)
        yaw = 0.45 * np.sin(2.0 * np.pi * u)
        left[:3, :3] = left[:3, :3] @ rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)
        right[:3, :3] = right[:3, :3] @ rot_z(-yaw) @ rot_y(pitch) @ rot_x(-roll)
        label = "hand orientation"
    left[:3, 3] += delta
    right[:3, 3] += delta
    return left, right, label


class MujocoPanelModel:
    def __init__(self, xml_path: Path, width: int, height: int, joint_names: tuple[str, ...], camera: str | None) -> None:
        import mujoco

        self.mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, height=height, width=width)
        self.width = width
        self.height = height
        self.arm_qpos_addr = []
        for joint_name in joint_names:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if jid < 0:
                raise RuntimeError(f"MuJoCo joint not found: {joint_name}")
            self.arm_qpos_addr.append(int(self.model.jnt_qposadr[jid]))
        free_jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "floating_base_joint")
        self.free_qpos_addr = int(self.model.jnt_qposadr[free_jid]) if free_jid >= 0 else None
        if camera in {"free", "g1_23_free"}:
            self.camera = mujoco.MjvCamera()
            self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            if camera == "g1_23_free":
                self.camera.lookat[:] = np.array([0.05, 0.0, -0.10], dtype=float)
            else:
                self.camera.lookat[:] = np.array([0.05, 0.0, 0.693], dtype=float)
            self.camera.distance = 2.2
            self.camera.azimuth = 25.0
            self.camera.elevation = -10.0
        else:
            self.camera = camera
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
        if self.camera is None:
            self.renderer.update_scene(self.data)
        else:
            self.renderer.update_scene(self.data, camera=self.camera)
        return self.renderer.render()

    def close(self) -> None:
        self.renderer.close()


def prepare_native_g1_23_urdf(mesh_source_dir: Path) -> tempfile.TemporaryDirectory:
    import mujoco

    temp_dir = tempfile.TemporaryDirectory(prefix="g1_23_mujoco_")
    temp_path = Path(temp_dir.name)
    urdf = ET.parse(G1_23_SPEC.urdf_path)
    urdf.getroot().find("mujoco/compiler").set("meshdir", ".")
    # Overlapping collision meshes obscure visual surfaces in this kinematic viewer.
    for link in urdf.getroot().findall("link"):
        for collision in link.findall("collision"):
            link.remove(collision)
    urdf.write(temp_path / "g1_body23.urdf")
    os.symlink(mesh_source_dir, temp_path / "meshes")
    model = mujoco.MjModel.from_xml_path(str(temp_path / "g1_body23.urdf"))
    scene_path = temp_path / "scene_23.xml"
    mujoco.mj_saveLastXML(str(scene_path), model)
    tree = ET.parse(scene_path)
    world = tree.getroot().find("worldbody")
    ET.SubElement(world, "light", pos="1 -1 3", dir="-0.2 0.2 -1", diffuse="0.8 0.8 0.8", ambient="0.4 0.4 0.4")
    ET.SubElement(world, "geom", name="floor", type="plane", size="4 4 0.1", pos="0 0 -0.793", rgba="0.3 0.33 0.35 1")
    tree.write(scene_path)
    return temp_dir


def annotate(img: np.ndarray, title: str, subtitle: str = "") -> np.ndarray:
    from PIL import Image, ImageDraw

    pil = Image.fromarray(img)
    draw = ImageDraw.Draw(pil)
    bar_h = 50 if subtitle else 32
    draw.rectangle((0, 0, img.shape[1], bar_h), fill=(15, 18, 22))
    draw.text((12, 9), title, fill=(240, 240, 240))
    if subtitle:
        draw.text((12, 29), subtitle, fill=(190, 205, 220))
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
            if len(frames) > 1:
                print("Restarting comparison cycle.")
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

    sim29 = MujocoPanelModel(xml_path, args.width, args.height, visual_29_arm_joint_names(), "free")
    native_temp_dir = None
    if args.right_visual == "g1_23_native":
        native_temp_dir = prepare_native_g1_23_urdf(repo_path / "assets" / "meshes")
        right_xml_path = Path(native_temp_dir.name) / "scene_23.xml"
        right_joint_names = tuple(ik23._arm_joint_names_g1)
        right_camera = "g1_23_free"
        right_title = "G1-23 native MuJoCo + constrained IK"
    else:
        right_xml_path = xml_path
        right_joint_names = visual_29_arm_joint_names()
        right_camera = "global_view"
        right_title = "G1-23 constrained IK on G1-29 visual"
    sim23 = MujocoPanelModel(right_xml_path, args.width, args.height, right_joint_names, right_camera)
    frames = []
    phase_positions = {}
    phase_rotations = {}
    if args.save_phase_frames:
        args.save_phase_frames.mkdir(parents=True, exist_ok=True)
    cycle_s = {"static": 1.0 / args.control_hz, "basic": 5.0, "step4_sweep": 24.0}[args.motion_profile]
    deadline_frames = int(args.duration_s * args.control_hz) if args.duration_s > 0.0 else int(cycle_s * args.control_hz)
    deadline_frames = max(1, deadline_frames)
    print(
        "Rendering comparison frames: left=G1-29 IK, right=G1-23 IK, "
        f"motion_profile={args.motion_profile}"
    )
    try:
        for i in range(deadline_frames):
            t = i / args.control_hz
            l29, r29, motion_label = trajectory(left29, right29, t, args.motion_profile)
            l23, r23, _ = trajectory(left23, right23, t, args.motion_profile)
            if args.motion_profile == "static":
                q29 = np.zeros_like(q29)
                q23 = np.zeros_like(q23)
            else:
                q29, _ = ik29.solve_ik(l29, r29, q29)
                q23, _ = ik23.solve_ik(l23, r23, q23)
            for name, ik, q in (("G1-29", ik29, q29), ("G1-23", ik23, q23)):
                if not np.all(np.isfinite(q)):
                    raise RuntimeError(f"{name}: non-finite IK solution")
                model = ik.reduced_robot.model
                if np.any(q < model.lowerPositionLimit - 1e-5) or np.any(q > model.upperPositionLimit + 1e-5):
                    raise RuntimeError(f"{name}: IK solution exceeds joint limits")
                actual = fk(model, ik.reduced_robot.data, ik.L_hand_id, ik.R_hand_id, q)
                phase_positions.setdefault((motion_label, name), []).append(
                    np.stack([pose[:3, 3] for pose in actual])
                )
                phase_rotations.setdefault((motion_label, name), []).append(
                    np.stack([pose[:3, :3] for pose in actual])
                )
            q29_visual = active_arm_q_to_visual_29(np.asarray(q29)[reorder29], "g1_29")
            q23_g1 = np.asarray(q23)[reorder23]
            q23_visual = active_arm_q_to_visual_29(q23_g1, "g1_23")
            sim29.set_arm(q29_visual)
            if args.right_visual == "g1_23_native":
                sim23.set_arm(q23_g1)
            else:
                sim23.set_arm(q23_visual)
            img29 = annotate(sim29.render(), "G1-29 IK", motion_label)
            img23 = annotate(sim23.render(), right_title, motion_label)
            frames.append(np.concatenate([img29, img23], axis=1))
            if args.save_phase_frames and (i == 0 or any(
                abs(t - instant) < 0.5 / args.control_hz
                for instant in (1.5, 4.5, 7.5, 10.5, 13.5, 16.5, 19.5, 22.5)
            )):
                from PIL import Image

                Image.fromarray(frames[-1]).save(args.save_phase_frames / f"frame_{i:04d}.png")
        for (phase, name), positions in phase_positions.items():
            span = np.ptp(np.asarray(positions), axis=0)
            print(f"{name} {phase}: actual hand xyz span (m), left={span[0].round(3)}, right={span[1].round(3)}")
            if phase == "hand orientation":
                rotations = np.asarray(phase_rotations[(phase, name)])
                relative = rotations @ np.swapaxes(rotations[0], -1, -2)
                angles = np.rad2deg(np.arccos(np.clip((np.trace(relative, axis1=-2, axis2=-1) - 1) / 2, -1, 1)))
                print(f"{name}: maximum hand rotation from phase start (degrees), left/right={angles.max(axis=0).round(1)}")
        final = frames[-1]
        left_mean = float(np.mean(final[:, : args.width]))
        right_mean = float(np.mean(final[:, args.width :]))
        right_bright_ratio = float(np.mean(final[:, args.width :] > 20))
        print(
            f"comparison render ok: frames={len(frames)} "
            f"left_mean_pixel={left_mean:.2f} right_mean_pixel={right_mean:.2f} "
            f"right_bright_ratio={right_bright_ratio:.3f}"
        )
        if left_mean < 20.0 or right_mean < 20.0 or right_bright_ratio < 0.20:
            print("comparison render failed blank-panel sanity check", file=sys.stderr)
            return 1
        if args.save_final_frame:
            from PIL import Image

            output_path = Path(args.save_final_frame).expanduser().resolve()
            output_path.parent.mkdir(parents=True, exist_ok=True)
            Image.fromarray(final).save(output_path)
            print(f"saved final frame: {output_path}")
        if args.no_view:
            return 0
        if args.duration_s <= 0:
            print(f"Looping {cycle_s:g}s comparison continuously. Close the window or press Ctrl+C to stop.")
        run_view(args, frames)
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 0
    finally:
        sim29.close()
        sim23.close()
        if native_temp_dir is not None:
            native_temp_dir.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
