"""Owned loopback-only joint and IK/DDS acceptance sessions; no XR or hardware."""
import argparse
from collections import OrderedDict
from pathlib import Path
import time

import numpy as np

from .backends.simulation import run_matrix
from .shared.acceptance_metrics import LIMITS, joint_result, pose_errors, trajectory_result
from .shared.motion_cases import baseline_pose, cartesian_targets
from .shared.reporting import write_report

ROOT = Path(__file__).resolve().parents[2]


def run_child(args):
    from ..robots.unitree_g1 import UnitreeG1, UnitreeG1Config, get_g1_embodiment
    from ..robots.control import action_from_arm_q, fk
    from ..simulation.dds import connect_unitree_g1_external_dds, patch_unitree_dds_config
    from lerobot.robots.unitree_g1 import unitree_g1 as g1_module

    spec = get_g1_embodiment(args.child)
    ik = spec.make_ik()
    model = ik.reduced_robot.model
    data = model.createData()
    ik.reduced_robot.data = data
    reorder = np.asarray(ik._arm_reorder_pin_to_g1)
    inverse = np.argsort(reorder)
    joints = list(spec.arm_index)
    lower, upper = model.lowerPositionLimit[reorder], model.upperPositionLimit[reorder]
    base = baseline_pose([joint.name for joint in joints])
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
    received = OrderedDict()
    active_case = ["startup"]
    observer = None
    unused = sorted(set(range(29)) - {j.value for j in spec.joint_index})

    def poses(q_g1):
        return fk(model, data, ik.L_hand_id, ik.R_hand_id, q_g1[inverse])

    def tick(command, case, targets=None, started=None):
        active_case[0] = case
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
        def receive(msg):
            now = time.monotonic()
            values = [float(m.q) for m in msg.motor_state]
            raw[0] = (now, values)
            if args.geometry_log:
                from ..simulation.geometry_trace import state_key
                received[state_key(msg.tick, values)] = dict(q=values, time=now, case=active_case[0])
                if len(received) > 65536:
                    received.popitem(last=False)
        observer.Init(receive, 1)
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
                    targets = cartesian_targets(homes, side, axis, step)
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
        if args.geometry_log:
            from .simulation.geometry_acceptance import compare_trace
            result["geometry"] = compare_trace(args.geometry_log, dict(received), ik, joints, args.child,
                                                [case["name"] for case in result["cases"]])
            result["passed"] &= result["geometry"]["passed"]
            print(f"Independent geometry: passed={result['geometry']['passed']}, "
                  f"matched={len(result['geometry']['samples'])}", flush=True)
    except BaseException as exc:
        result["passed"] = False
        result["error"] = repr(exc)
        raise
    finally:
        write_report(args.report, result)
        if observer is not None:
            observer.Close()
        robot.disconnect()
    return 0 if result["passed"] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--embodiment", action="append", choices=("g1_29", "g1_23"))
    parser.add_argument("--geometry", action="store_true", help="Also verify publish-time MuJoCo hand geometry against exact received DDS states.")
    parser.add_argument("--geometry-log", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--output", type=Path, default=ROOT/"outputs"/f"live-control-{time.strftime('%Y%m%d-%H%M%S')}")
    parser.add_argument("--child", choices=("g1_29", "g1_23"), help=argparse.SUPPRESS)
    parser.add_argument("--report", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    return run_child(args) if args.child else run_matrix(args)


if __name__ == "__main__":
    raise SystemExit(main())
