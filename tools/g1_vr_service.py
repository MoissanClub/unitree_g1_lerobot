"""Private local services for the fork-based operator launcher; never opens robot DDS."""

import argparse
from functools import partial
import json
import math
import os
from pathlib import Path
import time


def ready(args, role, data=None):
    path = args.run_dir / f"{role}.ready"
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(data or {}))
    temporary.replace(path)


def endpoint(args):
    return "ipc://" + str(args.run_dir / "control.sock")


def stopped(args):
    return (args.run_dir / "stop").exists()


def validate_command(message, embodiment, size, lower, upper):
    import numpy as np

    if message.get("embodiment") != embodiment:
        raise ValueError("Simulator/bridge embodiment mismatch")
    age = time.monotonic() - float(message["sent_at"])
    if not 0 <= age <= 0.5:
        raise ValueError("Stale or future command")
    q = np.asarray(message["q"], dtype=float)
    if (
        q.shape != (size,)
        or not np.all(np.isfinite(q))
        or np.any(q < lower)
        or np.any(q > upper)
    ):
        raise ValueError("Invalid or out-of-limit arm command")
    return q


def simulator(args):
    import numpy as np
    import zmq
    from lerobot.cameras.frame_channel import FrameWriter
    from lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config

    robot = UnitreeG1(
        UnitreeG1Config(
            embodiment=args.embodiment,
            simulation_urdf=str(args.assets / f"{args.embodiment}.urdf"),
            simulation_mesh_dir=str(args.assets / "meshes"),
            gravity_compensation=True,
            control_dt=0.004,
        )
    )
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.setsockopt(zmq.LINGER, 0)
    writer = window = None
    try:
        robot.connect()
        sim = robot._native
        writer = FrameWriter(args.run_dir / "camera.rgb", 320, 240)
        socket.bind(endpoint(args))
        if not args.headless:
            import tkinter as tk
            from PIL import Image, ImageTk

            window = tk.Tk()
            window.title(f"LeRobot {args.embodiment} MuJoCo")
            label = tk.Label(window)
            label.pack()
            window.protocol("WM_DELETE_WINDOW", lambda: (args.run_dir / "stop").touch())
            window.update()
        q = np.zeros(sim.ik.size)
        q[[0, sim.ik.size // 2]] = -0.4
        q[[3, sim.ik.size // 2 + 3]] = 0.7
        robot.send_action(sim.ik.arm_action(q))
        for _ in range(500):
            robot.step_simulation()
        last_command = time.monotonic()
        holding, frame = True, 0
        while not stopped(args):
            start = time.monotonic()
            advanced = 0
            if socket.poll(0):
                message = socket.recv_json()
                try:
                    if "q" in message:
                        q = validate_command(
                            message,
                            args.embodiment,
                            sim.ik.size,
                            sim.ik.lower,
                            sim.ik.upper,
                        )
                        robot.send_action(sim.ik.arm_action(q))
                        advanced += 1
                        last_command, holding = time.monotonic(), False
                    socket.send_json(
                        {
                            "embodiment": args.embodiment,
                            "q": sim.data.qpos[sim.qadr].tolist(),
                            "captured_at": time.monotonic(),
                        }
                    )
                except (ValueError, KeyError, TypeError) as exc:
                    socket.send_json({"error": str(exc)})
            if not holding and time.monotonic() - last_command > 0.5:
                robot.send_action(sim.ik.arm_action(sim.data.qpos[sim.qadr].copy()))
                advanced += 1
                holding = True
                print("Command timeout: holding measured arm positions", flush=True)
            for _ in range(5 - advanced):
                robot.step_simulation()
            if frame % 2 == 0:
                captured = time.monotonic_ns()
                pixels = robot.render_simulation(320, 240)
                writer.publish(
                    pixels,
                    {"captured_monotonic_ns": captured, "embodiment": args.embodiment},
                )
                if window is not None:
                    photo = ImageTk.PhotoImage(
                        Image.fromarray(robot.render_simulation(320, 240, False))
                    )
                    label.configure(image=photo)
                    label.image = photo
                if frame == 0:
                    ready(args, "simulator")
                    print(
                        "Simulator ready; gravity and measured-pose gravity compensation ON",
                        flush=True,
                    )
            if window is not None:
                window.update()
            frame += 1
            time.sleep(max(0, 0.02 - (time.monotonic() - start)))
    finally:
        if window is not None:
            window.destroy()
        if writer is not None:
            writer.close()
        socket.close()
        context.term()
        robot.disconnect()


def cloudxr(args):
    if args.replay:
        ready(args, "cloudxr")
        print("Replay only: no CloudXR runtime or headset session launched", flush=True)
        while not stopped(args):
            time.sleep(0.1)
        return
    from isaacteleop.cloudxr import CloudXRLauncher
    import socket

    # The SDK can remove stale runtime state; never use that to take over a live service.
    for port in (48322, 49100):
        with socket.socket() as probe:
            probe.settimeout(0.3)
            if probe.connect_ex(("127.0.0.1", port)) == 0:
                raise RuntimeError(
                    f"Port {port} already in use. Stop your existing CloudXR session first."
                )

    config = Path(__file__).resolve().parents[1] / "configs/cloudxr_quest3.env"
    with CloudXRLauncher(
        env_config=os.environ.get("CLOUDXR_ENV_FILE", str(config)),
        accept_eula=args.accept_cloudxr_eula,
    ) as launcher:
        environment = {
            key: value
            for key, value in os.environ.items()
            if key.startswith(("XR_", "NV_", "CXR_", "XRT_"))
        }
        ready(args, "cloudxr", environment)
        print("CloudXR ready; waiting for headset", flush=True)
        while not stopped(args):
            launcher.health_check()
            time.sleep(0.1)


def bridge(args):
    import numpy as np
    import zmq
    from lerobot.cameras.frame_channel import read_frame
    from lerobot.robots.unitree_g1.g1_cartesian_control import (
        G1ArmKinematics,
        G1CartesianConfig,
    )
    from lerobot.robots.unitree_g1.g1_xr_control import G1XRControl
    from lerobot.teleoperators.xr_controllers import XRControllers, XRControllersConfig
    from lerobot.teleoperators.xr_controllers.camera_display import VideoConfig
    from lerobot.teleoperators.xr_controllers.video_session import (
        VideoControllerSession,
    )

    os.environ.update(json.loads((args.run_dir / "cloudxr.ready").read_text()))
    ik = G1ArmKinematics(
        G1CartesianConfig(args.embodiment, str(args.assets / f"{args.embodiment}.urdf"))
    )
    control = G1XRControl(ik)
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.RCVTIMEO, 2000)
    socket.setsockopt(zmq.SNDTIMEO, 2000)
    socket.connect(endpoint(args))
    reader = None

    def exchange(message):
        socket.send_json(message)
        response = socket.recv_json()
        if "error" in response:
            raise RuntimeError(response["error"])
        if response["embodiment"] != args.embodiment:
            raise RuntimeError("Simulator embodiment mismatch")
        if not 0 <= time.monotonic() - response["captured_at"] <= 0.5:
            raise RuntimeError("Stale simulator feedback")
        return np.array(response["q"])

    try:
        q = initial = exchange({})
        if not args.replay:
            reader = XRControllers(
                XRControllersConfig(),
                session_factory=partial(
                    VideoControllerSession,
                    video=VideoConfig(
                        channel=str(args.run_dir / "camera.rgb"),
                        expected_source=args.embodiment,
                    ),
                ),
            )
            reader.connect()
        ready(args, "bridge")
        motion, step, sequences = 0.0, 0, set()
        while not stopped(args) and (not args.steps or step < args.steps):
            start = time.monotonic()
            if reader is None:
                sample = {"captured_at": start}
                for side in ("left", "right"):
                    sample.update(
                        {
                            f"{side}.tracked": True,
                            f"{side}.grip_pos": [0, 0, 0.025 * math.sin(step / 35)],
                            f"{side}.grip_quat": [0, 0, 0, 1],
                            f"{side}.squeeze": 1.0,
                            f"{side}.trigger": 0.0,
                        }
                    )
            else:
                sample = reader.get_action()
            action = control.action(sample, q)
            target = [action[f"{joint.name}.q"] for joint in ik.embodiment.arm_index]
            q = exchange(
                {
                    "embodiment": args.embodiment,
                    "q": target,
                    "sent_at": time.monotonic(),
                }
            )
            motion = max(motion, float(np.max(abs(q - initial))))
            camera = read_frame(args.run_dir / "camera.rgb")
            if camera is not None:
                if camera[0]["embodiment"] != args.embodiment or np.ptp(camera[1]) == 0:
                    raise RuntimeError("Wrong or blank camera frame")
                sequences.add(camera[0]["sequence"])
            step += 1
            time.sleep(max(0, 0.02 - (time.monotonic() - start)))
        if args.replay and (motion < 0.01 or len(sequences) < 2):
            raise RuntimeError(
                f"Replay failed: motion={motion}, camera frames={len(sequences)}"
            )
        print(
            f"{'PASS' if args.replay else 'COMPLETED live startup'} {args.embodiment}: "
            f"{step} frames, motion={motion:.4f}rad, "
            f"{len(sequences)} distinct camera frames",
            flush=True,
        )
    finally:
        try:
            if reader is not None:
                reader.disconnect()
        finally:
            socket.close()
            context.term()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("role", choices=("simulator", "bridge", "cloudxr"))
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--assets", type=Path, required=True)
    parser.add_argument("--embodiment", choices=("g1_29", "g1_23"), required=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--replay", action="store_true")
    parser.add_argument("--steps", type=int, default=0)
    parser.add_argument("--accept-cloudxr-eula", action="store_true")
    args = parser.parse_args()
    os.environ.setdefault("MUJOCO_GL", "egl")
    globals()[args.role](args)


if __name__ == "__main__":
    main()
