"""Run actual three-launcher sessions without DISPLAY; no physical robot or headset."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--embodiment", choices=("g1_29", "g1_23"), action="append")
    parser.add_argument("--camera", action="store_true", help="Also verify local robot-camera frames while all three services run headlessly.")
    parser.add_argument("--video", action="store_true", help="Also submit camera frames through the real OpenXR graphics session.")
    args = parser.parse_args()
    args.camera = args.camera or args.video
    # Refuse to interfere with an existing CloudXR session.
    try:
        connection = socket.create_connection(("127.0.0.1", 48322), timeout=0.5)
    except OSError:
        pass
    else:
        connection.close()
        raise RuntimeError("Port 48322 is already in use; stop the existing CloudXR session before verification")
    output = Path(tempfile.mkdtemp(prefix="g1-xr-headless-"))
    print(f"Verification logs: {output}", flush=True)
    for variant in args.embodiment or ("g1_29", "g1_23"):
        env = dict(os.environ, PYTHONUNBUFFERED="1", HF_HUB_OFFLINE="1")
        for key in ("DISPLAY", "WAYLAND_DISPLAY", "SKIP_G1_STARTUP_DIAGNOSTIC"):
            env.pop(key, None)
        env["G1_DIAGNOSTIC_REQUEST_FILE"] = str(output / f"{variant}.request")
        env["G1_DIAGNOSTIC_ACK_FILE"] = str(output / f"{variant}.ack")
        processes = []

        def start(label, script, *options):
            path = output / f"{variant}-{label}.log"
            with path.open("w") as log:
                selection = ["--embodiment", variant] if script in ("run_g1_mujoco_dds_sim.sh", "run_xr_g1_mujoco.sh") else []
                process = subprocess.Popen([str(ROOT / script), *selection,
                    "--headless", *options], cwd=ROOT, env=env, stdin=subprocess.DEVNULL,
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            processes.append(process)
            return process, path

        def wait_text(item, marker, seconds=60):
            process, path = item
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                if marker in path.read_text():
                    return
                if process.poll() is not None:
                    raise RuntimeError(f"{path}: exited {process.returncode} before {marker}\n{path.read_text()[-5000:]}")
                time.sleep(0.1)
            raise TimeoutError(f"{path}: missing {marker}\n{path.read_text()[-5000:]}")

        def finish(item, seconds=45):
            process, path = item
            rc = process.wait(timeout=seconds)
            content = path.read_text()
            if rc or "Traceback (most recent call last)" in content:
                raise RuntimeError(f"{path}: exit {rc}\n{content[-6000:]}")

        def bridge(label, side, mock=True):
            report = output / f"{variant}-{label}.json"
            options = ["--external-g1-sim", "--external-cloudxr", "--wait-for-cloudxr",
                       "--duration-s", "12", "--hand-side", side,
                       "--verification-report", str(report),
                       "--diagnostic-request-file", env["G1_DIAGNOSTIC_REQUEST_FILE"],
                       "--diagnostic-ack-file", env["G1_DIAGNOSTIC_ACK_FILE"]]
            if mock:
                options.append("--mock-xr")
                if side == "left":
                    options.append("--gravity-compensation")
                elif side == "right":
                    options.append("--no-gravity-compensation")
            else:
                options.append("--skip-startup-diagnostic")
            if args.video:
                options.extend(["--video", "--camera-channel", str(channel)])
            return start(label, "run_xr_g1_mujoco.sh", *options), report

        def check_video(report):
            if not args.video:
                return
            result = json.loads(report.read_text())
            assert result["video_enabled"], report
            stats = [s["video"] for s in result["samples"] if s.get("video", {}).get("camera_uploads", 0)]
            assert stats and max(s["camera_uploads"] for s in stats) >= 20, report
            assert max(s["render_requested"] for s in stats) >= 20, report
            print(f"PASS {variant}: OpenXR graphics requested rendering and received "
                  f"{max(s['camera_uploads'] for s in stats)} camera uploads ({report.stem})", flush=True)

        def check_motion(report):
            check_video(report)
            result = json.loads(report.read_text())
            assert result["gravity_compensation"] == (result["hand_side"] != "right"), report
            samples = result["samples"]
            if len(samples) < 20 or not any(s["engaged"] for s in samples):
                raise AssertionError(f"No sustained mock XR engagement: {report}")
            n = 7 if variant == "g1_29" else 5
            samples = samples[len(samples)//4:]
            sides = ("left", "right") if result["hand_side"] == "both" else (result["hand_side"],)
            for hand in sides:
                side = slice(0, n) if hand == "left" else slice(n, 2*n)
                command = np.array([s["command"] for s in samples])[:, side]
                measured = np.array([s["measured"] for s in samples])[:, side]
                assert np.isfinite(command).all() and np.isfinite(measured).all()
                assert np.max(np.ptp(command, axis=0)) > 0.01, report
                assert np.max(np.ptp(measured, axis=0)) > 0.01, report
                assert any(s["hands"][hand]["engaged"] for s in samples), report
                print(f"PASS {variant} {result['hand_side']}/{hand}: mock XR/IK/DDS motion, "
                      f"command span={np.max(np.ptp(command, axis=0)):.3f} rad, "
                      f"feedback span={np.max(np.ptp(measured, axis=0)):.3f} rad", flush=True)

        try:
            channel = output / f"{variant}.rgb"
            camera_options = ["--camera", "--camera-channel", str(channel)] if args.camera else []
            sim = start("sim", "run_g1_mujoco_dds_sim.sh", "--duration-s", "180", *camera_options)
            wait_text(sim, "Startup diagnostic complete" if variant == "g1_29" else "entering steady-state listening")
            wrong = "g1_23" if variant == "g1_29" else "g1_29"
            mismatch = start("mismatch", "run_xr_g1_mujoco.sh", "--embodiment", wrong,
                             "--external-g1-sim", "--mock-xr", "--skip-startup-diagnostic", "--duration-s", "1")
            assert mismatch[0].wait(timeout=30) == 2
            assert "Simulator embodiment mismatch" in mismatch[1].read_text()
            print(f"PASS {variant}: mismatched bridge rejected before control", flush=True)
            real, real_report = bridge("real-openxr", "both", mock=False)
            wait_text(real, "Steady-state listening")
            cloud = start("cloudxr", "run_isaac_teleop.sh", "--duration-s", "100")
            wait_text(cloud, "CloudXR ready")
            wait_text(real, "XR teleop connected")
            finish(real)
            check_video(real_report)
            assert len(json.loads(real_report.read_text())["samples"]) >= 10
            right, right_report = bridge("mock-right", "right")
            finish(right)
            check_motion(right_report)
            left, left_report = bridge("mock-left", "left")
            finish(left)
            check_motion(left_report)
            if args.camera:
                camera_report = output / f"{variant}-camera.json"
                camera = start("camera", "view_g1_camera.sh", "--camera-channel", str(channel),
                               "--duration-s", "12", "--report", str(camera_report),
                               "--save-frame", str(output / f"{variant}-camera.png"))
            both, both_report = bridge("mock-both", "both")
            finish(both)
            check_motion(both_report)
            if args.camera:
                finish(camera)
                samples = json.loads(camera_report.read_text())["samples"]
                assert len(samples) >= 30, camera_report
                assert all(s["embodiment"] == variant and s["camera_id"] == "robot_head" for s in samples)
                assert all(b["sequence"] > a["sequence"] for a, b in zip(samples, samples[1:]))
                assert samples[-1]["simulation_time_s"] > samples[0]["simulation_time_s"]
                assert max(s["pixel_delta"] for s in samples) > .1, "No visible camera motion"
                assert min(s["pixel_std"] for s in samples) > 5, "Blank camera frame"
                age = float(np.percentile([s["age_ms"] for s in samples], 95))
                assert age < 500, f"Camera too stale: p95={age}ms"
                fps = (len(samples)-1) / ((samples[-1]["captured_monotonic_ns"]-samples[0]["captured_monotonic_ns"])/1e9)
                assert fps >= 5, f"Camera too slow: {fps} fps"
                print(f"PASS {variant}: camera {fps:.1f} fps, p95 frame age {age:.0f} ms, live pixels and simulation timestamps", flush=True)
            assert sim[0].poll() is None and cloud[0].poll() is None
            os.killpg(cloud[0].pid, signal.SIGTERM)
            finish(cloud)
            os.killpg(sim[0].pid, signal.SIGINT)
            finish(sim)
            if args.camera:
                assert not channel.exists(), "Camera channel leaked after simulator shutdown"
            print(f"PASS {variant}: all three headless launchers, real CloudXR/OpenXR, clean shutdown", flush=True)
        finally:
            for process in reversed(processes):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
    print("Headless verification complete. Headset tracking and video are NOT verified.", flush=True)


if __name__ == "__main__":
    main()
