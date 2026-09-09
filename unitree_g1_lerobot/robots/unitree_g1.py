"""Register local G1-23 support for LeRobot's single configurable G1 implementation.

Requires the small LeRobot extension in patches/lerobot-g1-embodiments.patch.
No XR, MuJoCo, DDS connection, or IK solver is started on import.
"""
from enum import IntEnum

try:
    from lerobot.robots.unitree_g1.g1_runtime import G1RuntimeSpec, get_g1_embodiment, register_g1_embodiment
except ImportError as exc:
    raise ImportError("Apply patches/lerobot-g1-embodiments.patch to the adjacent LeRobot checkout first; see docs/architecture.md") from exc

from .g1_embodiments import G1_23_SPEC, G1_23_JointIndex, G1_23_JointArmIndex, make_arm_ik
from .motor_configs import load_profile


G1_23_ActiveJointIndex = IntEnum(
    "G1_23_ActiveJointIndex", {j.name: j.value for j in G1_23_JointIndex if "NotUsed" not in j.name}
)


def _make_g1_23_ik():
    return make_arm_ik("g1_23")


def register_local_embodiments():
    try:
        return get_g1_embodiment("g1_23")
    except ValueError:
        pass
    profile = load_profile("g1_23")
    kp, kd = [0.0] * 29, [0.0] * 29
    for motor in profile["motors"]:
        kp[motor["dds_index"]], kd[motor["dds_index"]] = motor["kp"], motor["kd"]
    spec = G1RuntimeSpec("g1_23", G1_23_ActiveJointIndex, G1_23_JointArmIndex,
                         tuple(kp), tuple(kd), _make_g1_23_ik,
                         model_resource=str(G1_23_SPEC.urdf_path))
    register_g1_embodiment(spec)
    return spec


register_local_embodiments()

# These are the existing LeRobot classes, not parallel variant implementations.
from lerobot.robots.unitree_g1.config_unitree_g1 import UnitreeG1Config  # noqa: E402,F401
from lerobot.robots.unitree_g1.unitree_g1 import UnitreeG1  # noqa: E402,F401
