#!/usr/bin/env python3
"""Show accepted XR calibration anchors as one posture/orientation matrix."""

from __future__ import annotations

import argparse
import base64
import html
import io
import json
import math
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import ttk


ROWS = ("down", "forward", "horizontal", "up")
COLUMNS = ("inward", "up", "down", "outward", "forward", "backward")
ROW_TITLES = {"down": "Arms down", "forward": "Arms forward", "horizontal": "Arms horizontal", "up": "Arms upward"}
COL_TITLES = {name: f"Palms {name}" for name in COLUMNS}


def _matrix_transpose(matrix):
    return tuple(zip(*matrix, strict=True))


def _matrix_vector(matrix, vector):
    return tuple(sum(row[index] * vector[index] for index in range(3)) for row in matrix)


def _matrix_multiply(left, right):
    columns = _matrix_transpose(right)
    return tuple(tuple(sum(a * b for a, b in zip(row, column, strict=True)) for column in columns) for row in left)


def quaternion_matrix(quaternion):
    x, y, z, w = (float(value) for value in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0:
        raise ValueError("zero-length quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def load_anchor_grid(path: Path):
    payload = json.loads(path.read_text())
    anchors = payload.get("anchors", {})
    grid = {}
    for label, anchor in anchors.items():
        if not label.startswith("calibration-"):
            continue
        suffix = label.removeprefix("calibration-")
        try:
            position, orientation = suffix.split("-", 1)
        except ValueError as error:
            raise ValueError(f"unrecognized calibration label: {label}") from error
        if position not in ROWS or orientation not in COLUMNS:
            raise ValueError(f"unsupported calibration matrix cell: {label}")
        poses = anchor.get("representative_poses")
        if not poses or any(name not in poses for name in ("head", "left", "right")):
            raise ValueError(f"missing representative head/controller poses: {label}")
        grid[(position, orientation)] = {"label": label, "poses": poses}
    return grid


def head_relative_pose(poses, side):
    head = poses["head"]
    hand = poses[side]
    head_rotation_inverse = _matrix_transpose(quaternion_matrix(head["quaternion_xyzw"]))
    relative_position = tuple(float(hand["position_m"][index]) - float(head["position_m"][index]) for index in range(3))
    position = _matrix_vector(head_rotation_inverse, relative_position)
    orientation = _matrix_multiply(head_rotation_inverse, quaternion_matrix(hand["quaternion_xyzw"]))
    return position, orientation


def retarget_robot_previews(grid, urdf_path: Path, mesh_dir: Path, *, max_iterations=80):
    """Fit static straight-arm observations and solve the native G1 Cartesian IK."""

    import numpy as np
    from PIL import Image
    from lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config

    reference_key = ("down", "inward")
    if reference_key not in grid:
        raise ValueError("retargeting preview requires calibration-down-inward")
    if ("up", "inward") not in grid:
        raise ValueError("reach estimation requires calibration-up-inward")
    from scipy.spatial.transform import Rotation

    # Fixed body heading from the reference headset, retaining gravity-up.
    down = grid[reference_key]["poses"]
    forward = -np.asarray(quaternion_matrix(down["head"]["quaternion_xyzw"]))[:, 2]
    forward[1] = 0
    if np.linalg.norm(forward) < 1e-6:
        raise ValueError("Down/inward headset orientation lacks a horizontal heading")
    forward /= np.linalg.norm(forward)
    vertical = np.array([0., 1., 0.])
    basis = np.stack([forward, np.cross(vertical, forward), vertical])
    neutral = np.zeros(14)
    neutral[[3, 10]] = np.pi / 2  # G1 forearms point forward at zero elbow.
    sides = ("left", "right")

    def position(poses, side):
        return basis @ (np.array(poses[side]["position_m"]) - poses["head"]["position_m"])

    def rotation(poses, side):
        return basis @ Rotation.from_quat(poses[side]["quaternion_xyzw"]).as_matrix()
    robot = UnitreeG1(
        UnitreeG1Config(
            id="g1-29-calibration-preview",
            embodiment="g1_29",
            is_simulation=True,
            simulation_urdf=str(urdf_path),
            simulation_mesh_dir=str(mesh_dir),
            gravity_compensation=True,
        )
    )
    previews = {}
    report = {
        "schema_version": 2,
        "reference_anchor": grid[reference_key]["label"],
        "virtual_clutch_override": False,
        "calibration_transform_status": "offline_anchor_fit",
        "interpretation": (
            "Static IK targets fitted from this operator's calibration observations."
        ),
        "anchors": {},
    }
    robot.connect()
    try:
        native = robot._native
        if native is None:
            raise RuntimeError("native G1-29 simulation did not initialize")
        ik = native.ik
        home = ik.fk(neutral)
        up = grid[("up", "inward")]["poses"]
        fits = {}
        for i, side in enumerate(sides):
            low, high = position(down, side), position(up, side)
            shoulder = (low + high) / 2
            span = np.linalg.norm(high - low)
            if span < 0.25:
                raise ValueError(f"Insufficient up/down separation for {side}")
            data = ik.model.createData()
            ik._pin.forwardKinematics(ik.model, data, ik._to_model(neutral))
            joint = ik.model.getJointId(f"{side}_shoulder_pitch_joint")
            reach = np.linalg.norm(home[i][:3, 3] - data.oMi[joint].translation)
            inward = np.array([0., -1. if i == 0 else 1., 0.])
            normals = dict(inward=inward, outward=-inward, up=[0,0,1], down=[0,0,-1],
                           forward=[1,0,0], backward=[-1,0,0])
            # Express each annotated normal in controller coordinates and average.
            normal = np.mean([rotation(e["poses"], side).T @ normals[col]
                              for (row, col), e in grid.items() if row in ("forward", "horizontal")], axis=0)
            if not np.isfinite(normal).all() or np.linalg.norm(normal) < 0.2:
                raise ValueError(f"Inconsistent palm orientation anchors for {side}")
            fits[side] = shoulder, 2 * reach / span, normal / np.linalg.norm(normal)
        report["fits"] = {s: {"shoulder_m": p.tolist(), "scale": float(k),
                              "controller_palm_normal": n.tolist()} for s,(p,k,n) in fits.items()}
        for row in ROWS:
            for column in COLUMNS:
                key = (row, column)
                entry = grid.get(key)
                if entry is None:
                    continue
                targets = []
                for i, side in enumerate(sides):
                    target = home[i].copy()
                    if key != reference_key:
                        shoulder, scale, normal = fits[side]
                        p = position(entry["poses"], side)
                        target[:3, 3] += scale * (p - position(down, side))
                        # Straight-arm protocol: fingers extend along the arm.
                        x = p - shoulder
                        if np.linalg.norm(x) < 1e-6:
                            raise ValueError(f"Undefined straight-arm direction for {side} at {key}")
                        x /= np.linalg.norm(x)
                        y = rotation(entry["poses"], side) @ normal * (-1 if i == 0 else 1)
                        y -= x * (x @ y)
                        if np.linalg.norm(y) < 1e-6:
                            raise ValueError(f"Ambiguous palm direction for {side} at {key}")
                        y /= np.linalg.norm(y)
                        target[:3, :3] = np.column_stack([x, y, np.cross(x, y)])
                    targets.append(target)
                previous = neutral.copy()
                for iteration in range(1, max_iterations + 1):
                    result = ik.solve(*targets, seed=previous)
                    current = result.q
                    stable = float(np.max(np.abs(current - previous))) < 1e-6
                    previous = current
                    if result.converged or stable:
                        break
                native.data.qpos[native.qadr] = result.q
                native.data.qvel[:] = 0.0
                native.target = native.data.qpos[native.qadr].copy()
                native.mj.mj_forward(native.model, native.data)
                rgb = robot.render_simulation(220, 150, head_camera=False)
                image_buffer = io.BytesIO()
                Image.fromarray(rgb).save(image_buffer, format="PNG")
                previews[key] = image_buffer.getvalue()
                report["anchors"][entry["label"]] = {
                    "iterations": iteration,
                    "action": ik.arm_action(result.q),
                    "diagnostics": {"ik_status": result.status, "converged": bool(result.converged),
                                    "position_error_m": result.position_error_m,
                                    "orientation_error_rad": result.orientation_error_rad},
                }
    finally:
        robot.disconnect()
    return previews, report


@dataclass
class Primitive:
    kind: str
    values: tuple
    options: dict


def build_scene(grid):
    width, height = 1800, 1000
    left, top, right, bottom = 190, 90, 25, 100
    cell_w = (width - left - right) / len(COLUMNS)
    cell_h = (height - top - bottom) / len(ROWS)
    primitives = []

    def line(x1, y1, x2, y2, **options):
        primitives.append(Primitive("line", (x1, y1, x2, y2), options))

    def circle(x, y, radius, **options):
        primitives.append(Primitive("circle", (x, y, radius), options))

    def text(x, y, value, **options):
        primitives.append(Primitive("text", (x, y, value), options))

    text(width / 2, 25, "XR calibration posture and controller-orientation anchors", size=20, weight="bold")
    text(width / 2, 52, "Head-relative recorded poses; grip is not required. RGB arrows are controller-local X/Y/Z axes.", size=12)

    for column, orientation in enumerate(COLUMNS):
        text(left + (column + 0.5) * cell_w, top - 20, COL_TITLES[orientation], size=13, weight="bold")
    for row, position in enumerate(ROWS):
        text(left - 18, top + (row + 0.5) * cell_h, ROW_TITLES[position], size=14, weight="bold", anchor="e")

    def projected(vector):
        x, y, z = vector
        # OpenXR is right-handed with Y up. Add a small Z contribution to make
        # forward/back displacement visible without obscuring vertical posture.
        return x + 0.28 * z, -y + 0.18 * z

    all_projected = []
    for entry in grid.values():
        for side in ("left", "right"):
            position, _ = head_relative_pose(entry["poses"], side)
            all_projected.append(projected(position))
    extent = max((max(abs(x), abs(y)) for x, y in all_projected), default=1.0)
    scale = min(cell_w * 0.34, cell_h * 0.34) / max(extent, 0.25)
    axis_length = 0.16

    for row, position in enumerate(ROWS):
        for column, orientation in enumerate(COLUMNS):
            x0 = left + column * cell_w
            y0 = top + row * cell_h
            line(x0, y0, x0 + cell_w, y0, fill="#c9ced6")
            line(x0, y0, x0, y0 + cell_h, fill="#c9ced6")
            entry = grid.get((position, orientation))
            center_x = x0 + cell_w / 2
            center_y = y0 + cell_h * 0.48
            if entry is None:
                text(center_x, center_y, "—", size=22, fill="#a8adb5")
                continue
            circle(center_x, center_y, 5, fill="#20242b", outline="#20242b")
            text(center_x, center_y - 12, "head", size=8, fill="#4a4f58")
            for side, hand_color in (("left", "#2468d8"), ("right", "#e37a18")):
                hand_position, hand_rotation = head_relative_pose(entry["poses"], side)
                px, py = projected(hand_position)
                hand_x, hand_y = center_x + px * scale, center_y + py * scale
                line(center_x, center_y, hand_x, hand_y, fill=hand_color, dash=(4, 3), width=1)
                circle(hand_x, hand_y, 5, fill=hand_color, outline="#ffffff")
                text(hand_x + (8 if side == "right" else -8), hand_y - 8, side[0].upper(), size=9, weight="bold", fill=hand_color, anchor="w" if side == "right" else "e")
                for axis, axis_color in enumerate(("#d72b2b", "#189447", "#2358c7")):
                    direction = tuple(hand_rotation[index][axis] * axis_length for index in range(3))
                    dx, dy = projected(direction)
                    line(hand_x, hand_y, hand_x + dx * scale, hand_y + dy * scale, fill=axis_color, width=2, arrow=True)
            samples = min(int(entry["poses"][side].get("tracked_samples", 0)) for side in ("left", "right"))
            text(center_x, y0 + cell_h - 14, f"{samples} tracked samples", size=8, fill="#626872")

    line(left, top + len(ROWS) * cell_h, width - right, top + len(ROWS) * cell_h, fill="#c9ced6")
    line(width - right, top, width - right, top + len(ROWS) * cell_h, fill="#c9ced6")
    text(left, height - 55, "● Left controller", size=11, fill="#2468d8", anchor="w")
    text(left + 150, height - 55, "● Right controller", size=11, fill="#e37a18", anchor="w")
    text(left + 330, height - 55, "Axis colors: X red, Y green, Z blue", size=11, anchor="w")
    text(left, height - 30, "This verifies recorded XR observations; it does not apply the unsolved human-to-robot calibration transform or G1 IK.", size=11, fill="#6b3f00", anchor="w")
    return width, height, primitives


def build_robot_scene(grid, previews, report):
    width, height = 1800, 1000
    left, top, right, bottom = 190, 90, 25, 100
    cell_w = (width - left - right) / len(COLUMNS)
    cell_h = (height - top - bottom) / len(ROWS)
    primitives = []

    def line(x1, y1, x2, y2, **options):
        primitives.append(Primitive("line", (x1, y1, x2, y2), options))

    def text(x, y, value, **options):
        primitives.append(Primitive("text", (x, y, value), options))

    text(width / 2, 25, "G1-29 retargeting preview from XR calibration anchors", size=20, weight="bold")
    text(
        width / 2,
        52,
        "Arms-down robot reference, operator reach scaling and fitted controller palm normals.",
        size=12,
        fill="#7a4100",
    )
    for column, orientation in enumerate(COLUMNS):
        text(left + (column + 0.5) * cell_w, top - 20, COL_TITLES[orientation], size=13, weight="bold")
    for row, position in enumerate(ROWS):
        text(left - 18, top + (row + 0.5) * cell_h, ROW_TITLES[position], size=14, weight="bold", anchor="e")

    for row, position in enumerate(ROWS):
        for column, orientation in enumerate(COLUMNS):
            key = (position, orientation)
            x0 = left + column * cell_w
            y0 = top + row * cell_h
            line(x0, y0, x0 + cell_w, y0, fill="#c9ced6")
            line(x0, y0, x0, y0 + cell_h, fill="#c9ced6")
            entry = grid.get(key)
            if entry is None:
                text(x0 + cell_w / 2, y0 + cell_h / 2, "—", size=22, fill="#a8adb5")
                continue
            png = previews[key]
            primitives.append(
                Primitive(
                    "image",
                    (x0 + (cell_w - 220) / 2, y0 + 7, 220, 150, png),
                    {},
                )
            )
            diagnostics = report["anchors"][entry["label"]]["diagnostics"]
            status = diagnostics["ik_status"] or "held"
            converged = diagnostics["converged"] is True
            color = "#176b32" if converged else "#a23a24"
            position_error = diagnostics["position_error_m"]
            if position_error is None:
                detail = status
            else:
                detail = f"{status}; pos err {max(position_error):.3f} m"
            text(x0 + cell_w / 2, y0 + cell_h - 19, detail, size=9, fill=color)

    line(left, top + len(ROWS) * cell_h, width - right, top + len(ROWS) * cell_h, fill="#c9ced6")
    line(width - right, top, width - right, top + len(ROWS) * cell_h, fill="#c9ced6")
    text(
        left,
        height - 50,
        "Position error is the larger left/right IK residual for the fitted target.",
        size=11,
        fill="#7a260f",
        anchor="w",
    )
    text(
        left,
        height - 27,
        "Rendered at the retargeted joint target using the native G1-29 MuJoCo model. Live and trajectory grip behavior is unchanged.",
        size=11,
        anchor="w",
    )
    return width, height, primitives


def write_svg(path: Path, width, height, primitives):
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">', '<rect width="100%" height="100%" fill="white"/>']
    arrow_ids = {"#d72b2b": "arrow-red", "#189447": "arrow-green", "#2358c7": "arrow-blue"}
    for primitive in primitives:
        options = primitive.options
        if primitive.kind == "line":
            x1, y1, x2, y2 = primitive.values
            dash = ' stroke-dasharray="4 3"' if options.get("dash") else ""
            marker = f' marker-end="url(#{arrow_ids[options["fill"]]})"' if options.get("arrow") else ""
            lines.append(f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" stroke="{options.get("fill", "black")}" stroke-width="{options.get("width", 1)}"{dash}{marker}/>')
        elif primitive.kind == "circle":
            x, y, radius = primitive.values
            lines.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{radius}" fill="{options.get("fill", "none")}" stroke="{options.get("outline", "none")}"/>')
        elif primitive.kind == "image":
            x, y, image_width, image_height, png = primitive.values
            encoded = base64.b64encode(png).decode("ascii")
            lines.append(f'<image x="{x:.2f}" y="{y:.2f}" width="{image_width}" height="{image_height}" href="data:image/png;base64,{encoded}"/>')
        else:
            x, y, value = primitive.values
            anchor = {"w": "start", "e": "end"}.get(options.get("anchor"), "middle")
            lines.append(f'<text x="{x:.2f}" y="{y:.2f}" text-anchor="{anchor}" dominant-baseline="middle" font-family="sans-serif" font-size="{options.get("size", 12)}" font-weight="{options.get("weight", "normal")}" fill="{options.get("fill", "#20242b")}">{html.escape(str(value))}</text>')
    definitions = ["<defs>"]
    for color, marker_id in arrow_ids.items():
        definitions.append(f'<marker id="{marker_id}" markerWidth="5" markerHeight="5" refX="4" refY="2.5" orient="auto"><path d="M0,0 L5,2.5 L0,5 z" fill="{color}"/></marker>')
    definitions.append("</defs>")
    lines.insert(2, "".join(definitions))
    lines.append("</svg>")
    path.write_text("\n".join(lines) + "\n")


def show_tk(views):
    root = tk.Tk()
    root.title("XR calibration posture grid")
    notebook = ttk.Notebook(root)
    notebook.pack(fill="both", expand=True)
    for title, width, height, primitives in views:
        frame = tk.Frame(notebook)
        notebook.add(frame, text=title)
        canvas = tk.Canvas(frame, bg="white", scrollregion=(0, 0, width, height))
        x_scroll = tk.Scrollbar(frame, orient="horizontal", command=canvas.xview)
        y_scroll = tk.Scrollbar(frame, orient="vertical", command=canvas.yview)
        canvas.configure(xscrollcommand=x_scroll.set, yscrollcommand=y_scroll.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        y_scroll.grid(row=0, column=1, sticky="ns")
        x_scroll.grid(row=1, column=0, sticky="ew")
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)
        image_references = []
        for primitive in primitives:
            options = primitive.options
            if primitive.kind == "line":
                kwargs = {"fill": options.get("fill", "black"), "width": options.get("width", 1)}
                if options.get("dash"):
                    kwargs["dash"] = options["dash"]
                if options.get("arrow"):
                    kwargs["arrow"] = tk.LAST
                canvas.create_line(*primitive.values, **kwargs)
            elif primitive.kind == "circle":
                x, y, radius = primitive.values
                canvas.create_oval(x - radius, y - radius, x + radius, y + radius, fill=options.get("fill", ""), outline=options.get("outline", ""))
            elif primitive.kind == "image":
                x, y, image_width, image_height, png = primitive.values
                image = tk.PhotoImage(data=base64.b64encode(png), format="png")
                image_references.append(image)
                canvas.create_image(x, y, image=image, anchor=tk.NW)
            else:
                x, y, value = primitive.values
                anchor = {"w": tk.W, "e": tk.E}.get(options.get("anchor"), tk.CENTER)
                canvas.create_text(x, y, text=value, anchor=anchor, fill=options.get("fill", "#20242b"), font=("Sans", options.get("size", 12), options.get("weight", "normal")))
        canvas._image_references = image_references
    root.geometry(f"{min(width, root.winfo_screenwidth())}x{min(height, root.winfo_screenheight() - 80)}")
    root.mainloop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("anchor_set", type=Path)
    parser.add_argument("--output-svg", type=Path)
    parser.add_argument("--urdf", type=Path)
    parser.add_argument("--mesh-dir", type=Path)
    parser.add_argument("--output-retarget-svg", type=Path)
    parser.add_argument("--output-retarget-report", type=Path)
    parser.add_argument("--retarget-iterations", type=int, default=80)
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()
    grid = load_anchor_grid(args.anchor_set)
    if not grid:
        raise SystemExit("anchor set contains no calibration anchors")
    width, height, primitives = build_scene(grid)
    views = [("Recorded XR", width, height, primitives)]
    if args.output_svg:
        write_svg(args.output_svg, width, height, primitives)
        print(f"Calibration grid SVG: {args.output_svg}")
    print(f"Loaded {len(grid)} calibration anchors; {len(ROWS) * len(COLUMNS) - len(grid)} unavailable cells are blank.")
    if (args.urdf is None) != (args.mesh_dir is None):
        raise SystemExit("--urdf and --mesh-dir must be supplied together")
    if args.retarget_iterations <= 0:
        raise SystemExit("--retarget-iterations must be positive")
    if args.urdf is not None:
        previews, report = retarget_robot_previews(
            grid, args.urdf, args.mesh_dir, max_iterations=args.retarget_iterations
        )
        robot_width, robot_height, robot_primitives = build_robot_scene(
            grid, previews, report
        )
        views.append(("G1 retargeting preview", robot_width, robot_height, robot_primitives))
        if args.output_retarget_svg:
            write_svg(
                args.output_retarget_svg,
                robot_width,
                robot_height,
                robot_primitives,
            )
            print(f"G1 retargeting preview SVG: {args.output_retarget_svg}")
        if args.output_retarget_report:
            args.output_retarget_report.parent.mkdir(parents=True, exist_ok=True)
            args.output_retarget_report.write_text(
                json.dumps(report, indent=2, sort_keys=True) + "\n"
            )
            print(f"G1 retargeting report: {args.output_retarget_report}")
    if not args.no_show:
        show_tk(views)


if __name__ == "__main__":
    main()
