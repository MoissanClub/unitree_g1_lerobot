"""Owned loopback-only joint and IK/DDS acceptance sessions; no XR or hardware."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
LIMITS = dict(settled_joint_rad=.04, cross_joint_rad=.04, minimum_response_rad=.06,
              moving_joint_p95_rad=.08, actuator_position_p95_m=.035,
              actuator_rotation_p95_rad=.20, ik_position_p95_m=.04, ik_rotation_p95_rad=.50)


def pose_errors(reference, actual):
    return [dict(position_m=float(np.linalg.norm(a[:3, 3]-b[:3, 3])),
                 rotation_rad=float(np.arccos(np.clip((np.trace(a[:3, :3].T @ b[:3, :3])-1)/2, -1, 1))))
            for a, b in zip(reference, actual)]


def joint_result(samples, baseline, target, index):
    settled = np.median(np.array([s["measured"] for s in samples[-10:]]), axis=0)
    response = float((settled[index]-baseline[index]) * np.sign(target[index]-baseline[index]))
    error = float(np.max(np.abs(settled-target)))
    cross = float(np.max(np.abs(np.delete(settled-baseline, index))))
    return dict(response_rad=response, settled_error_rad=error, cross_joint_rad=cross,
                passed=bool(response >= LIMITS["minimum_response_rad"] and
                            error <= LIMITS["settled_joint_rad"] and cross <= LIMITS["cross_joint_rad"]))


def trajectory_result(rows):
    # Worst per-joint/per-hand p95 prevents an inactive arm diluting the metric.
    metrics = dict(joint_p95_rad=float(np.max(np.percentile(np.abs(
        np.array([s["command"] for s in rows])-np.array([s["measured"] for s in rows])), 95, axis=0))))
    for kind in ("ik", "actuator", "total"):
        for field in ("position_m", "rotation_rad"):
            metrics[f"{kind}_{field}"] = max(float(np.percentile([s[kind][hand][field] for s in rows], 95))
                                             for hand in range(2))
    passed = metrics["joint_p95_rad"] <= LIMITS["moving_joint_p95_rad"]
    for kind in ("ik", "actuator"):
        passed &= metrics[f"{kind}_position_m"] <= LIMITS[f"{kind}_position_p95_m"]
        passed &= metrics[f"{kind}_rotation_rad"] <= LIMITS[f"{kind}_rotation_p95_rad"]
    side = rows[0]["case"].split("/")[1]
    count = len(rows[0]["command"])//2
    for hand in ((0, 1) if side == "both" else (0,) if side == "left" else (1,)):
        indices = slice(hand*count, (hand+1)*count)
        for key in ("command", "measured"):
            span = float(np.max(np.ptp(np.array([s[key] for s in rows])[:, indices], axis=0)))
            metrics[f"{key}_span_{hand}_rad"] = span
            passed &= span > .005
    return dict(passed=bool(passed), **metrics)


def run_child(args):
    from ..robots.unitree_g1 import UnitreeG1, UnitreeG1Config, get_g1_embodiment
    from ..robots.control import action_from_arm_q, fk
    from ..simulation.dds import connect_unitree_g1_external_dds, patch_unitree_dds_config
    from lerobot.robots.unitree_g1 import unitree_g1 as g1_module
    import pinocchio as pin

    spec = get_g1_embodiment(args.child)
    ik = spec.make_ik()
    model = ik.reduced_robot.model
    data = model.createData()
    ik.reduced_robot.data = data
    reorder = np.asarray(ik._arm_reorder_pin_to_g1)
    inverse = np.argsort(reorder)
    joints = list(spec.arm_index)
    lower, upper = model.lowerPositionLimit[reorder], model.upperPositionLimit[reorder]
    base = np.zeros(len(joints))
    for i, joint in enumerate(joints):
        if "ShoulderPitch" in joint.name:
            base[i] = -.4
        elif "ShoulderRoll" in joint.name:
            base[i] = .15 if "Left" in joint.name else -.15
        elif "Elbow" in joint.name:
            base[i] = .8
    if np.any(base-.12 < lower) or np.any(base+.12 > upper):
        raise ValueError("Test pose lacks joint-limit margin")
    patch_unitree_dds_config()
    config = UnitreeG1Config(embodiment=args.child, is_simulation=True, gravity_compensation=True)
    robot = UnitreeG1(config)
    result = dict(embodiment=args.child, gravity_compensation=True, thresholds=LIMITS,
                  joints=[dict(name=j.name, dds_slot=j.value, pin_name=ik._arm_joint_names_g1[i],
                               lower_rad=float(lower[i]), upper_rad=float(upper[i])) for i, j in enumerate(joints)],
                  kp=list(config.kp), kd=list(config.kd), cases=[], samples=[])
    samples = result["samples"]
    raw = [None]
    observer = None
    unused = sorted(set(range(29)) - {j.value for j in spec.joint_index})

    def poses(q_g1):
        return fk(model, data, ik.L_hand_id, ik.R_hand_id, q_g1[inverse])

    def tick(command, case, targets=None, started=None):
        started = time.monotonic() if started is None else started
        if not np.isfinite(command).all() or np.any(command < lower-1e-6) or np.any(command > upper+1e-6):
            raise ValueError("Nonfinite or out-of-limit command rejected before publication")
        robot.send_action(action_from_arm_q(command, spec.joint_index, spec.arm_index))
        if any(any(getattr(robot.msg.motor_cmd[i], field) != 0 for field in ("mode", "q", "dq", "kp", "kd", "tau"))
               for i in unused):
            raise ValueError("Unused transport command slot is nonzero")
        time.sleep(max(0., 1/30-(time.monotonic()-started)))
        observation = robot.get_observation()
        measured = np.array([observation[f"{j.name}.q"] for j in joints])
        if not np.isfinite(measured).all():
            raise ValueError("Nonfinite DDS feedback")
        state = raw[0]
        if state is None or time.monotonic()-state[0] > .25:
            raise RuntimeError("Independent DDS observer has stale feedback")
        if any(state[1][i] != 0 for i in unused):
            raise ValueError("Unused transport feedback slot is nonzero")
        commanded_poses, measured_poses = poses(command), poses(measured)
        row = dict(case=case, monotonic_s=time.monotonic(), command=command.tolist(), measured=measured.tolist(),
                   loop_s=time.monotonic()-started, raw_dds_q=state[1], feedback_age_s=time.monotonic()-state[0],
                   actuator=pose_errors(commanded_poses, measured_poses))
        if targets is not None:
            row.update(target_poses=[p.tolist() for p in targets], command_poses=[p.tolist() for p in commanded_poses],
                       measured_poses=[p.tolist() for p in measured_poses],
                       ik=pose_errors(targets, commanded_poses), total=pose_errors(targets, measured_poses))
        samples.append(row)

    def hold(command, label, steps=36):
        for _ in range(steps):
            tick(command, label)
        return np.median(np.array([s["measured"] for s in samples[-10:]]), axis=0)

    try:
        connect_unitree_g1_external_dds(robot, g1_module, spec.joint_index, 12)
        observer = robot._ChannelSubscriber(g1_module.kTopicLowState, g1_module.hg_LowState)
        observer.Init(lambda msg: raw.__setitem__(0, (time.monotonic(), [float(m.q) for m in msg.motor_state])), 1)
        deadline = time.monotonic()+3
        while raw[0] is None and time.monotonic() < deadline:
            time.sleep(.01)
        hold(base, "initial_hold", 60)
        for i, joint in enumerate(joints):
            for sign in (1, -1):
                baseline = hold(base, "reset")
                target = base.copy()
                target[i] += sign*.12
                label = f"joint/{joint.name}/{sign:+d}"
                start = len(samples)
                hold(target, label)
                metrics = joint_result(samples[start:], baseline, target, i)
                result["cases"].append(dict(name=label, **metrics))
                print(f"{label}: {metrics}", flush=True)
        hold(base, "reset")
        homes = poses(base)
        q = base[inverse].copy()
        for side in ("left", "right", "both"):
            for axis in range(6):
                label = f"cartesian/{side}/{axis}"
                start = len(samples)
                for step in range(90):
                    started = time.monotonic()
                    targets = [p.copy() for p in homes]
                    phase = np.sin(2*np.pi*step/89)
                    for hand in range(2):
                        if side != "both" and hand != (0 if side == "left" else 1):
                            continue
                        if axis < 3:
                            targets[hand][axis, 3] += .015*phase
                        else:
                            rotation = np.zeros(3)
                            rotation[axis-3] = .08*phase
                            targets[hand][:3, :3] = homes[hand][:3, :3] @ pin.exp3(rotation)
                    q, _ = ik.solve_ik(*targets, q)
                    q = np.asarray(q)
                    if side != "both":
                        frozen = [i for i, j in enumerate(joints) if ("Left" in j.name) != (side == "left")]
                        q[reorder[frozen]] = base[frozen]
                    tick(q[reorder], label, targets, started)
                rows = samples[start:]
                metrics = trajectory_result(rows)
                result["cases"].append(dict(name=label, **metrics))
                print(f"{label}: {metrics}", flush=True)
        hold(base, "final_hold")
        result["passed"] = all(case["passed"] for case in result["cases"])
    except BaseException as exc:
        result["passed"] = False
        result["error"] = repr(exc)
        raise
    finally:
        args.report.write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
        if observer is not None:
            observer.Close()
        robot.disconnect()
    return 0 if result["passed"] else 1


def run_matrix(args):
    with open(f"/tmp/lerobot-g1-sim-{os.getuid()}.lock", "a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    def revision(path):
        return subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip()
    metadata = dict(project_revision=revision(ROOT), lerobot_revision=revision(ROOT.parent/"lerobot"),
                    diagnostic_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    thresholds=LIMITS, python=sys.version, outcomes={})
    print(f"Reports: {output}", flush=True)
    env = dict(os.environ, HF_HUB_OFFLINE="1", PYTHONUNBUFFERED="1")
    env.pop("DISPLAY", None)
    for variant in args.embodiment or ("g1_29", "g1_23"):
        sim = None
        try:
            with (output/f"{variant}-sim.log").open("w") as log:
                sim = subprocess.Popen([str(ROOT/"run_g1_mujoco_dds_sim.sh"), "--embodiment", variant,
                                        "--headless", "--skip-startup-diagnostic", "--duration-s", "240"],
                                       cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            time.sleep(5)
            if sim.poll() is not None:
                raise RuntimeError(f"Simulator exited; inspect {variant}-sim.log")
            with (output/f"{variant}-control.log").open("w") as log:
                child = subprocess.run([sys.executable, "-m", "unitree_g1_lerobot.diagnostics.verify_live_control",
                                        "--child", variant, "--report", str(output/f"{variant}.json")],
                                       cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=220)
            metadata["outcomes"][variant] = child.returncode
            if sim.poll() is not None:
                raise RuntimeError("Simulator stopped before acceptance completed")
            print(f"{variant}: {'PASS' if child.returncode == 0 else 'FAIL'}", flush=True)
        finally:
            if sim is not None and sim.poll() is None:
                os.killpg(sim.pid, signal.SIGINT)
                try:
                    sim.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(sim.pid, signal.SIGKILL)
                    sim.wait()
            if sim is not None:
                metadata.setdefault("simulator_exit_codes", {})[variant] = sim.returncode
            (output/"manifest.json").write_text(json.dumps(metadata, indent=2)+"\n")
    return int(any(metadata["outcomes"].values()) or any(metadata["simulator_exit_codes"].values()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--embodiment", action="append", choices=("g1_29", "g1_23"))
    parser.add_argument("--output", type=Path, default=ROOT/"outputs"/f"live-control-{time.strftime('%Y%m%d-%H%M%S')}")
    parser.add_argument("--child", choices=("g1_29", "g1_23"), help=argparse.SUPPRESS)
    parser.add_argument("--report", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    return run_child(args) if args.child else run_matrix(args)


if __name__ == "__main__":
    raise SystemExit(main())
