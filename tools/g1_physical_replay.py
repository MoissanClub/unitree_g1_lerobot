"""Prepare calibrated G1-29 replay plans, or execute one reviewed segment on the robot host."""
import argparse
import hashlib
import json
import math
import runpy
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(args):
    from lerobot.robots.unitree_g1.g1_cartesian_control import G1ArmKinematics, G1CartesianConfig
    from lerobot_teleoperator_g1_vr.calibration import load_anchor_grid, load_profile, solve_anchor, verify_trace_anchor

    suite = args.suite
    anchors = suite / "calibration/anchor-set.json"
    ik = G1ArmKinematics(G1CartesianConfig("g1_29", args.urdf))
    profile = load_profile(anchors, ik)
    segments = {}
    if args.kind == "postures":
        for key, entry in load_anchor_grid(anchors).items():
            if args.label and args.label != entry["label"]:
                continue
            targets, result, _ = solve_anchor(ik, profile, key, entry)
            segments[entry["label"]] = [{"action": ik.arm_action(result.q), "time_s": 0.,
                                         "targets": [t.tolist() for t in targets], "ik_status": result.status}]
    else:
        manifest = suite / "manifest.json"
        for label, segment in json.loads(manifest.read_text())["segments"].items():
            if label.startswith("calibration-") or (args.label and label != args.label):
                continue
            if segment["status"] != "passed":
                raise ValueError(f"Unaccepted segment {label}")
            verify_trace_anchor(manifest, label, anchors)
            trace = Path(segment["selected_trace"])
            subprocess.run([sys.executable, str(Path(__file__).with_name("check_g1_vr_trace.py")),
                            str(trace), "--expect", label.split("-")[0]], check=True)
            with tempfile.TemporaryDirectory() as directory:
                telemetry = Path(directory) / "trajectory.json"
                subprocess.run([sys.executable, str(Path(__file__).with_name("run_supervised_g1_29_vr.py")),
                                "--source", "trace", "--trace", str(trace), "--anchor-set", str(anchors),
                                "--urdf", str(args.urdf), "--telemetry", str(telemetry)], check=True)
                rows = json.loads(telemetry.read_text())
            for r in rows:
                r["time_s"] = r["captured_at_s"] - rows[0]["captured_at_s"]
            segments[label] = rows
    if not segments:
        raise ValueError("No matching accepted segment")
    output = args.output or suite.parents[1] / "physical-plans" / f"{args.kind}-{time.time_ns()}.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    plan = {"version": 1, "kind": args.kind, "profile": profile.metadata(), "segments": segments}
    encoded = json.dumps(plan, allow_nan=False)
    with output.open("x") as stream:
        stream.write(encoded)
    print(f"Offline replay plan {output}\nSHA256 {digest(output)}")


def execute(args):
    # The robot host uses its reviewed Arm SDK checkout and Python environment.
    from lerobot.robots.unitree_g1 import g1_arm_sdk as sdk

    checkout = Path(sdk.__file__).resolve().parents[4]
    validation = runpy.run_path(str(checkout / "examples/unitree_g1/validate_arm_sdk.py"))
    contract = json.loads(args.contract.read_text())
    cfg = validation["validate_contract"](contract, args.urdf, True)
    raw = args.execute.read_bytes()
    if contract.get("replay_plan_sha256") != hashlib.sha256(raw).hexdigest():
        raise ValueError("Contract must identify the reviewed replay plan by SHA256")
    plan = json.loads(raw)
    if (plan["version"] != 1 or plan["kind"] != args.kind or plan["profile"]["embodiment"] != "g1_29"
            or plan["profile"]["urdf_sha256"] != digest(args.urdf)):
        raise ValueError("Replay plan model, version or task mismatch")
    rows = plan["segments"][args.label]
    keys = sdk.ARM_KEYS
    points = np.array([[row["action"][key] for key in keys] for row in rows])
    times = np.array([r["time_s"] for r in rows]) * args.stretch
    if (not np.isfinite(points).all() or not np.isfinite(times).all() or times[0] != 0
            or np.any(np.diff(times) <= 0) or np.any(points < cfg.lower) or np.any(points > cfg.upper)):
        raise ValueError("Invalid plan timestamps or joint envelope")
    if len(points) > 1:
        dt = np.diff(times)
        velocity = np.diff(points, axis=0) / dt[:, None]
        if np.max(np.abs(velocity)) > cfg.max_velocity:
            raise ValueError("Replay timing exceeds reviewed velocity")
        padded = np.vstack((np.zeros(14), velocity, np.zeros(14)))
        intervals = np.r_[dt[0], (dt[:-1] + dt[1:]) / 2, dt[-1]]
        if np.any(np.abs(np.diff(padded, axis=0)) > cfg.max_acceleration * intervals[:, None]):
            raise ValueError("Replay timing exceeds reviewed acceleration")
    robot = sdk.G1ArmSDK(cfg)
    with args.output.open("x", buffering=1) as log:
        try:
            log.write(json.dumps({"plan_sha256": contract["replay_plan_sha256"],
                                  "contract_sha256": digest(args.contract), "label": args.label, "stretch": args.stretch}) + "\n")
            robot.connect()
            measured = robot.observation()
            start = np.array([measured[k] for k in keys])
            if np.max(abs(points - start)) > cfg.max_displacement:
                raise ValueError("Segment exceeds reviewed displacement from measured start")
            if args.kind == "trajectories":
                expected = np.asarray(rows[0]["measured"])
                if np.max(abs(start - expected)) > cfg.max_tracking_error:
                    raise ValueError("Measured starting pose differs from simulation replay")
            robot.activate()
            distance = np.max(abs(points[0] - start))
            approach = max(cfg.period_s, 2 * distance / cfg.max_velocity,
                           math.sqrt(6 * distance / cfg.max_acceleration))
            begin = time.monotonic()
            while True:
                elapsed = time.monotonic() - begin
                u = np.clip((elapsed - cfg.blend_s) / approach, 0., 1.)
                t = elapsed - cfg.blend_s - approach
                if t <= 0:
                    target = start + (points[0] - start) * (u*u*(3-2*u))
                else:
                    target = [np.interp(t, times, col) for col in points.T]
                command = dict(zip(keys, map(float, target), strict=True))
                robot.send(command)
                observation = robot.observation()
                log.write(json.dumps({"elapsed_s": elapsed, "target": command,
                                      "measured": [observation[k] for k in keys]}) + "\n")
                if t >= times[-1] + 1.:
                    if max(abs(observation[k] - command[k]) for k in keys) > cfg.max_tracking_error:
                        raise RuntimeError("Final posture outside reviewed tracking tolerance")
                    break
                time.sleep(cfg.period_s)
        finally:
            robot.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("kind", choices=["postures", "trajectories"])
    parser.add_argument("--suite", type=Path)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--label")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--execute", type=Path, metavar="REVIEWED_PLAN")
    parser.add_argument("--contract", type=Path)
    parser.add_argument("--stretch", type=float, default=1., help="Replay duration multiplier (>=1) for hardware")
    args = parser.parse_args()
    if not math.isfinite(args.stretch) or args.stretch < 1:
        parser.error("--stretch must be finite and at least 1")
    if args.execute:
        if not (args.contract and args.label and args.output):
            parser.error("hardware execution requires --label, --contract and a new --output")
        def stop(*_):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM, stop)
        execute(args)
    elif args.suite:
        prepare(args)
    else:
        parser.error("offline preparation requires --suite")


if __name__ == "__main__":
    main()
