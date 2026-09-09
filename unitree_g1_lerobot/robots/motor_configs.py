"""Reproducible motor profiles derived from pinned Unitree source data."""
from __future__ import annotations

import ast
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
SOURCES = ROOT / "assets/g1/motor_sources"
CONFIGS = ROOT / "configs/motors"
MODEL_REVISION = "1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d"
CONTROLLER_REVISION = "817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b"


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def derive_profile(variant: str) -> dict:
    if variant not in {"g1_29", "g1_23"}:
        raise ValueError(variant)
    prefix = variant.upper()
    source = SOURCES / "robot_arm.py.txt"
    classes = {n.name: n for n in ast.parse(source.read_text()).body if isinstance(n, ast.ClassDef)}
    controller = classes[f"{prefix}_ArmController"]
    methods = {n.name: n for n in controller.body if isinstance(n, ast.FunctionDef)}
    constants = {}
    control_hz = None
    for node in ast.walk(methods["__init__"]):
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Attribute):
            name = node.targets[0].attr
            if name.startswith(("kp_", "kd_")):
                constants[name] = float(ast.literal_eval(node.value))
            elif name == "control_dt" and isinstance(node.value, ast.BinOp) and isinstance(node.value.op, ast.Div):
                control_hz = ast.literal_eval(node.value.right) / ast.literal_eval(node.value.left)
    if control_hz is None:
        raise ValueError("Unrecognized controller timestep expression")

    def enum(name):
        return {n.targets[0].id: ast.literal_eval(n.value) for n in classes[name].body if isinstance(n, ast.Assign)}

    def category(method):
        values = next(n.value for n in methods[method].body if isinstance(n, ast.Assign) and isinstance(n.value, ast.List))
        return {n.value.attr for n in values.elts}

    indices = enum(f"{prefix}_JointIndex")
    arms = enum(f"{prefix}_JointArmIndex")
    wrists = category("_Is_wrist_motor")
    weak = category("_Is_weak_motor")
    xml_path = SOURCES / f"{variant}dof.xml"
    tree = ET.parse(xml_path)
    joints = {j.get("name"): j for j in tree.findall(".//worldbody//joint")}
    passive = {d.get("class"): d.find("joint").attrib for d in tree.findall("./default/default")}
    normalized = {name[1:].lower(): name for name in indices}
    motors = []
    for actuator in tree.findall("./actuator/motor"):
        name = actuator.get("joint")
        if variant == "g1_23" and name in {"waist_roll_joint", "waist_pitch_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"}:
            continue
        enum_name = normalized[name.removesuffix("_joint").replace("_", "").lower()]
        joint = joints[name]
        # The G1-23 XML has detached placeholder bodies for six unused DDS slots.
        if variant == "g1_23" and indices[enum_name] in {13, 14, 20, 21, 27, 28}:
            continue
        group = "wrist" if enum_name in arms and enum_name in wrists else "low" if enum_name in arms or enum_name in weak else "high"
        dynamics = passive[joint.get("class")]
        motors.append({
            "joint": name, "dds_index": indices[enum_name], "arm": enum_name in arms,
            "kp": constants[f"kp_{group}"], "kd": constants[f"kd_{group}"],
            "torque_limit_nm": max(abs(float(v)) for v in actuator.get("ctrlrange").split()),
            "range_rad": [float(v) for v in joint.get("range").split()],
            "damping": float(dynamics["damping"]), "armature": float(dynamics["armature"]),
            "frictionloss": float(dynamics["frictionloss"]),
        })
    return {
        "schema_version": 1, "name": f"{variant}_unitree_derived", "variant": variant,
        "control_hz": control_hz, "control_hz_basis": "Derived from ArmController.control_dt",
        "method": "Arm wrists use wrist gains; other arms and weak motors use low gains; remaining real joints use high gains. Physical properties come from MJCF, not controller gains.",
        "sources": {
            "model_revision": MODEL_REVISION, "controller_revision": CONTROLLER_REVISION,
            "model_file": str(xml_path.relative_to(ROOT)), "model_sha256": digest(xml_path),
            "controller_file": str(source.relative_to(ROOT)), "controller_sha256": digest(source),
        },
        "motors": motors,
    }


def load_profile(variant: str) -> dict:
    profile = json.loads((CONFIGS / f"{variant}.json").read_text())
    if profile != derive_profile(variant):
        raise ValueError(f"{variant} motor profile differs from pinned sources; regenerate explicitly")
    return profile


def lerobot_profile() -> dict:
    from lerobot.robots.unitree_g1.config_unitree_g1 import UnitreeG1Config
    import lerobot.robots.unitree_g1.config_unitree_g1 as config_module

    config = UnitreeG1Config()
    profile = load_profile("g1_29")
    profile["name"] = "g1_29_lerobot"
    profile["control_hz"] = 1.0 / config.control_dt
    profile["sources"]["lerobot_config_file"] = str(Path(config_module.__file__).resolve())
    profile["sources"]["lerobot_config_sha256"] = digest(Path(config_module.__file__))
    profile["method"] = "Live LeRobot kp/kd by DDS index on the SAME pinned G1-29 physics model as the derived profile. Gravity compensation disabled for both."
    for motor in profile["motors"]:
        i = motor["dds_index"]
        motor["kp"], motor["kd"] = float(config.kp[i]), float(config.kd[i])
    return profile


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write", action="store_true", help="Regenerate tracked JSON profiles from vendored sources")
    args = parser.parse_args()
    for variant in ("g1_29", "g1_23"):
        if args.write:
            CONFIGS.mkdir(parents=True, exist_ok=True)
            (CONFIGS / f"{variant}.json").write_text(json.dumps(derive_profile(variant), indent=2) + "\n")
        profile = load_profile(variant)
        print(f"{profile['name']}: {len(profile['motors'])} real motors; source derivation verified")


if __name__ == "__main__":
    main()
