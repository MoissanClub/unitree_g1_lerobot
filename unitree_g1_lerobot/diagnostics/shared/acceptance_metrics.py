"""Numerical acceptance calculations; budgets describe the simulation baseline."""
import numpy as np

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
