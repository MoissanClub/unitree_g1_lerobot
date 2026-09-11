"""Existing small-signal simulation cases, independent of transport and execution."""
import numpy as np


def baseline_pose(joint_names):
    pose = np.zeros(len(joint_names))
    for i, name in enumerate(joint_names):
        if "ShoulderPitch" in name:
            pose[i] = -.4
        elif "ShoulderRoll" in name:
            pose[i] = .15 if "Left" in name else -.15
        elif "Elbow" in name:
            pose[i] = .8
    return pose


def cartesian_targets(homes, side, axis, step):
    """One sample of the original 90-sample, 15 mm / 0.08 rad sine sweep."""
    import pinocchio as pin

    if side not in ("left", "right", "both") or axis not in range(6) or step not in range(90):
        raise ValueError("Invalid baseline Cartesian case or sample")
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
    return targets
