"""Run and replay paired G1 motor tests with plots and machine-readable results."""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
from datetime import datetime
import json
import os
from pathlib import Path
import time

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np

from unitree_g1_lerobot.diagnostics.motor_suite import SUITE
from unitree_g1_lerobot.robots.motor_configs import ROOT, load_profile, lerobot_profile
from unitree_g1_lerobot.simulation.motor_bench import DT, MESH_REVISION, MotorPlant, mesh_directory, run_trial


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--comparison", choices=("lerobot", "embodiments"), required=True)
    parser.add_argument("--tests", nargs="+", choices=[t.name for t in SUITE], help="Default: all ten tests")
    parser.add_argument("--no-view", action="store_true")
    parser.add_argument("--gui-seconds", type=float, default=0, help="Close viewer after N seconds; 0 loops until closed")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--camera-azimuth", type=float, default=-135)
    parser.add_argument("--camera-elevation", type=float, default=-10)
    parser.add_argument("--camera-distance", type=float, default=2.2)
    parser.add_argument("--width", type=int, default=540)
    parser.add_argument("--height", type=int, default=520)
    args = parser.parse_args()
    if not all(np.isfinite(v) for v in (args.camera_azimuth, args.camera_elevation, args.camera_distance, args.gui_seconds)) or args.camera_distance <= 0 or args.gui_seconds < 0:
        parser.error("camera parameters must be finite; distance positive and gui-seconds nonnegative")
    if not 240 <= args.width <= 1600 or not 240 <= args.height <= 1200:
        parser.error("panel dimensions must fit 240..1600 by 240..1200")
    if args.output_dir is None:
        args.output_dir = ROOT / "artifacts/motors" / f"{datetime.now():%Y%m%d_%H%M%S_%f}_{args.comparison}"
    return args


def write_report(pairs, output, comparison):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows = []
    metadata = {
        "comparison": comparison, "physics_dt_s": DT, "target_update_dt_s": 0.004,
        "support": "Pelvis, legs and waist rigidly supported; only arm joints simulated",
        "controller": "tau = clip(kp * (q_command - q) - kd * dq, torque_limit); no gravity feedforward",
        "scope": "Same G1-29 model and identical targets for gain A/B; different native models with common-joint targets for embodiment comparison. Not hardware or DDS validation.",
        "tests": [asdict(pair[0]["test"]) for pair in pairs],
        "profiles": [trial["profile"] for trial in pairs[0]],
        "mesh_source": {"repo": "lerobot/unitree-g1-mujoco", "revision": MESH_REVISION},
        "versions": {"mujoco": mujoco.__version__, "numpy": np.__version__},
        "integrity_limits": {"max_limit_violation_rad": 0.03, "peak_joint_speed_rad_s": 20},
        "results": rows,
    }
    fig, axes = plt.subplots(len(pairs), 3, figsize=(15, 2.5 * len(pairs)), squeeze=False)
    for row, pair in enumerate(pairs):
        test = pair[0]["test"]
        feature = "shoulder_roll" if test.name in {"shoulder_roll_step", "left_right"} else "elbow" if test.name == "forward_back" else "wrist_roll" if test.name == "wrist_rotation" else "shoulder_pitch"
        for side, trial in enumerate(pair):
            profile, log = trial["profile"], trial["log"]
            label = profile["name"]
            joint = trial["joints"].index(f"left_{feature}_joint")
            rows.append({"test": test.name, "profile": label, **trial["metrics"]})
            np.savez_compressed(output / f"{test.name}_{label}.npz", joints=np.array(trial["joints"]), **log)
            color = ("#007f80", "#c04659")[side]
            if side == 0:
                axes[row, 0].plot(log["time"], log["target"][:, joint], color="#555555", linestyle="--", label="joint target")
            axes[row, 0].plot(log["time"], log["q"][:, joint], color=color, label=label)
            axes[row, 1].plot(log["time"], 1000 * np.sqrt(np.mean(log["hand_error"] ** 2, axis=1)), color=color)
            limits = np.array([m["torque_limit_nm"] for m in profile["motors"] if m["arm"]])
            axes[row, 2].plot(log["time"], np.max(np.abs(log["raw_torque"]) / limits, axis=1), color=color)
        axes[row, 0].set_title(f"{test.name}: left {feature} (rad)", fontsize=9)
        axes[row, 1].set_title("Hand tracking error, RMS of two hands (mm)", fontsize=9)
        axes[row, 2].set_title("Peak requested torque / torque limit", fontsize=9)
        axes[row, 2].axhline(1, color="#555555", linestyle="--")
        for ax in axes[row]:
            ax.set_xlabel("Time (s)")
            ax.grid(alpha=0.25)
    axes[0, 0].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(output / "comparison.png", dpi=130)
    plt.close(fig)
    (output / "report.json").write_text(json.dumps(metadata, indent=2, allow_nan=False) + "\n")
    with (output / "summary.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    lines = ["# G1 Motor Comparison", "", metadata["scope"], "", metadata["support"], "",
             "PD feedback runs at 500 Hz; targets update at 250 Hz. All cases use gravity, with no gravity feedforward. Each test resets, then warms up for 0.75 s before measurement.", "",
             "Common-joint RMSE uses the same ten shoulder/elbow/wrist-roll joints in both robots. Hand error is measured relative to each model's own FK of commanded joints, not a common Cartesian path.", "",
             "| Test | Profile | Common joint RMS (rad) | Hand RMS (mm) | Peak torque (Nm) | Saturation (%) |", "|---|---|---:|---:|---:|---:|"]
    for r in rows:
        lines.append(f"| {r['test']} | {r['profile']} | {r['common_joint_rmse_rad']:.4f} | {1000*r['hand_rmse_m']:.1f} | {r['peak_torque_nm']:.2f} | {100*r['saturation_fraction']:.2f} |")
    steps = [r for r in rows if r["step_equilibrium_overshoot_rad"] is not None]
    if steps:
        lines.extend(["", "## Step Response", "", "| Test | Profile | Overshoot around equilibrium (deg) | Settling to equilibrium (s) |", "|---|---|---:|---:|"])
        for r in steps:
            settling = r["settling_to_equilibrium_s"]
            settling_text = "not settled" if settling is None else f"{settling:.3f}"
            lines.append(f"| {r['test']} | {r['profile']} | {np.rad2deg(r['step_equilibrium_overshoot_rad']):.2f} | {settling_text} |")
    lines.extend(["", "`comparison.png`: overlaid traces. `summary.csv` / `report.json`: metrics and source provenance. Each NPZ contains full 500 Hz target, delivered target, joint, velocity, requested/applied torque, and hand-error traces.", "",
                  "Lag is a mean-centered cross-correlation estimate within +/-0.2 s, not a network latency measurement. Stop excursion is measured from the actual joint positions when the target stops. Torque slew measures successive applied torque changes per second.", "",
                  "Step settling uses a 0.02 rad band around the commanded target and requires remaining inside for at least the final 0.5 s. Null means not settled within the trial (or not a step test), not zero settling time. Gravity-induced steady-state error may prevent settling.", "",
                  "Equilibrium overshoot and settling are also reported for step tests. They use the mean achieved position in the final 0.5 s and a 0.005 rad settling band, separating transient ringing from gravity bias.", "",
                  "This report does not automatically rank profiles. Compare tracking, overshoot, saturation, and payload/delay robustness before choosing gains. Passing the simulation integrity checks is not hardware validation."])
    (output / "report.md").write_text("\n".join(lines) + "\n")


class PairRenderer:
    def __init__(self, pair, meshes, args):
        self.pair, self.args = pair, args
        self.plants, self.renderers = [], []
        try:
            for trial in pair:
                plant = MotorPlant(trial["profile"], meshes, trial["test"].payload_kg)
                self.plants.append(plant)
                self.renderers.append(mujoco.Renderer(plant.model, height=args.height, width=args.width))
        except BaseException:
            self.close()
            raise
        self.camera = mujoco.MjvCamera()
        self.camera.lookat[:] = [0.05, 0, 0.73]
        self.camera.distance = args.camera_distance
        self.camera.azimuth = args.camera_azimuth
        self.camera.elevation = args.camera_elevation
        self.option = mujoco.MjvOption()
        self.option.geomgroup[3] = 0

    def frame(self, index):
        from PIL import Image, ImageDraw
        images = []
        for trial, plant, renderer in zip(self.pair, self.plants, self.renderers, strict=True):
            log = trial["log"]
            i = min(index, len(log["time"]) - 1)
            plant.data.qpos[plant.qadr] = log["q"][i]
            mujoco.mj_forward(plant.model, plant.data)
            renderer.update_scene(plant.data, camera=self.camera, scene_option=self.option)
            for position in log["target_hand"][i]:
                scene = renderer.scene
                mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                                   np.array([0.012, 0.012, 0.012]), position, np.eye(3).ravel(), np.array([1.0, 0.7, 0.1, 1.0]))
                scene.ngeom += 1
            pixels = renderer.render().copy()
            if np.std(pixels[self.args.height//5:]) < 5:
                raise RuntimeError("Blank comparison panel")
            image = Image.fromarray(pixels)
            draw = ImageDraw.Draw(image)
            draw.rectangle((0, 0, self.args.width, 54), fill=(20, 23, 26))
            draw.text((10, 8), trial["profile"]["name"], fill="white")
            error = 1000 * np.sqrt(np.mean(log["hand_error"][i] ** 2))
            draw.text((10, 28), f"{trial['test'].name} | t={log['time'][i]:.2f}s | hand error={error:.1f} mm", fill=(220, 220, 220))
            images.append(image)
        combined = Image.new("RGB", (2 * self.args.width, self.args.height))
        for side, image in enumerate(images):
            combined.paste(image, (side * self.args.width, 0))
        return combined

    def close(self):
        for renderer in self.renderers:
            renderer.close()
        self.renderers = []


def replay(pairs, meshes, args):
    import tkinter as tk
    from tkinter import ttk
    from PIL import ImageTk

    root = tk.Tk()
    root.title("G1 motor physics comparison")
    bar = ttk.Frame(root)
    bar.pack(fill="x")
    choice = ttk.Combobox(bar, values=[p[0]["test"].name for p in pairs], state="readonly", width=25)
    choice.current(0)
    choice.pack(side="left", padx=5, pady=5)
    canvas = tk.Label(root)
    canvas.pack()
    state = {"test": 0, "t": 0.0, "playing": True, "last": time.monotonic()}
    current = [PairRenderer(pairs[0], meshes, args)]
    errors = []

    def callback_error(exc_type, exc, traceback):
        errors.append(exc)
        root.destroy()

    root.report_callback_exception = callback_error

    def select(index):
        current[0].close()
        state.update(test=index % len(pairs), t=0.0, last=time.monotonic())
        choice.current(state["test"])
        current[0] = PairRenderer(pairs[state["test"]], meshes, args)

    def toggle():
        state["playing"] = not state["playing"]
        pause.configure(text="Pause" if state["playing"] else "Play")

    choice.bind("<<ComboboxSelected>>", lambda event: select(choice.current()))
    pause = ttk.Button(bar, text="Pause", command=toggle)
    pause.pack(side="left", padx=3)
    ttk.Button(bar, text="Restart", command=lambda: select(state["test"])).pack(side="left", padx=3)
    ttk.Button(bar, text="Previous", command=lambda: select(state["test"] - 1)).pack(side="left", padx=3)
    ttk.Button(bar, text="Next", command=lambda: select(state["test"] + 1)).pack(side="left", padx=3)
    speed = ttk.Combobox(bar, values=("0.5", "1", "2"), state="readonly", width=4)
    speed.set("1")
    speed.pack(side="left", padx=5)

    def tick():
        now = time.monotonic()
        elapsed = now - state["last"]
        state["last"] = now
        if state["playing"]:
            state["t"] += elapsed * float(speed.get())
        if state["t"] >= pairs[state["test"]][0]["test"].seconds:
            select(state["test"] + 1)
        photo = ImageTk.PhotoImage(current[0].frame(round(state["t"] / DT)))
        canvas.configure(image=photo)
        canvas.image = photo
        root.after(33, tick)

    if args.gui_seconds:
        root.after(round(1000 * args.gui_seconds), root.destroy)
    try:
        tick()
        root.mainloop()
    finally:
        current[0].close()
        try:
            root.destroy()
        except tk.TclError:
            pass
    if errors:
        raise RuntimeError("Viewer callback failed") from errors[0]


def main():
    args = parse_args()
    try:
        import matplotlib  # noqa: F401
    except ImportError as exc:
        raise SystemExit("Matplotlib is required. Install it in lerobot-g1 as documented in docs/motor-config-comparison.md") from exc
    if not args.no_view and not os.environ.get("DISPLAY"):
        raise SystemExit("No DISPLAY. Use an ssh -Y terminal, or pass --no-view for reports only.")
    output = args.output_dir.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    profiles = [load_profile("g1_29"), lerobot_profile() if args.comparison == "lerobot" else load_profile("g1_23")]
    if any(profile["control_hz"] != 250 for profile in profiles):
        raise ValueError("This benchmark requires both profiles at 250 Hz")
    meshes = mesh_directory()
    tests = [test for test in SUITE if not args.tests or test.name in args.tests]
    pairs = []
    for test in tests:
        pair = []
        for profile in profiles:
            trial = run_trial(MotorPlant(profile, meshes, test.payload_kg), test)
            m = trial["metrics"]
            if m["max_limit_violation_rad"] > 0.03 or m["peak_joint_speed_rad_s"] > 20:
                raise RuntimeError(f"{test.name}: exceeded simulation integrity limits: {m}")
            print(f"{test.name:22} {profile['name']:24} joint RMS={m['common_joint_rmse_rad']:.4f}rad hand RMS={1000*m['hand_rmse_m']:.1f}mm saturation={100*m['saturation_fraction']:.2f}%", flush=True)
            pair.append(trial)
        if args.comparison == "lerobot":
            np.testing.assert_array_equal(pair[0]["log"]["target"], pair[1]["log"]["target"])
        pairs.append(pair)
        renderer = PairRenderer(pair, meshes, args)
        try:
            renderer.frame(round(min(2, test.seconds/2) / DT)).save(output / f"preview_{test.name}.png")
        finally:
            renderer.close()
    write_report(pairs, output, args.comparison)
    print(f"Physics tests complete. Results: {output / 'report.md'}", flush=True)
    if not args.no_view:
        print("Replaying recorded physics trajectories continuously; close the window or Ctrl+C to stop.", flush=True)
        replay(pairs, meshes, args)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("Stopped.")
