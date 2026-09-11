"""Audit pinned G1-29 model families and compare common-frame FK without actuation."""
import argparse
import hashlib
import json
from pathlib import Path
import urllib.request
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pinocchio as pin

from .verify_live_control import pose_errors

HUB_REVISION = "a38dc8617f0fca51b38e9354dc58ee35ad850fb5"


def urdf_signature(content):
    root = ET.fromstring(content)
    names = ("waist_roll_joint", "waist_pitch_joint", "left_shoulder_pitch_joint", "right_shoulder_pitch_joint")
    origins = {name: list(map(float, root.find(f"joint[@name='{name}']/origin").get("xyz").split())) for name in names}
    return dict(origins=origins, torso_mesh=root.find("link[@name='torso_link']/visual/geometry/mesh").get("filename"),
                neutral_shoulder_pelvis_z=sum(origins[n][2] for n in names[:3]))


def compare_models(urdf, mjcf):
    pm = pin.buildModelFromUrdf(str(urdf))
    pd = pm.createData()
    mm = mujoco.MjModel.from_xml_path(str(mjcf))
    md = mujoco.MjData(mm)
    cases = {"neutral": {}}
    arm = {"left_shoulder_pitch_joint": -.4, "right_shoulder_pitch_joint": -.5,
           "left_elbow_joint": .8, "right_elbow_joint": .9,
           "left_wrist_roll_joint": .15, "right_wrist_yaw_joint": -.2}
    cases["arms_waist_zero"] = arm
    for axis in ("yaw", "roll", "pitch"):
        for sign in (-1, 1):
            cases[f"waist_{axis}_{sign:+d}"] = dict(arm, **{f"waist_{axis}_joint": sign*.3})
    result = []
    for label, values in cases.items():
        q = np.zeros(pm.nq)
        mujoco.mj_resetData(mm, md)
        for name, value in values.items():
            q[pm.idx_qs[pm.getJointId(name)]] = value
            md.qpos[mm.joint(name).qposadr[0]] = value
        pin.forwardKinematics(pm, pd, q)
        pin.updateFramePlacements(pm, pd)
        mujoco.mj_kinematics(mm, md)
        row = dict(name=label, joint_values=values)
        for frame in ("pelvis", "torso_link"):
            pbase = pd.oMf[pm.getFrameId(frame)].inverse()
            body = mm.body(frame).id
            r, t = md.xmat[body].reshape(3, 3), md.xpos[body]
            pp, mp = [], []
            for side in ("left", "right"):
                pj = pm.getJointId(f"{side}_wrist_yaw_joint")
                pp.append((pbase * pd.oMi[pj] * pin.SE3(np.eye(3), np.array([.05, 0, 0]))).homogeneous)
                hand = mm.body(f"{side}_wrist_yaw_link").id
                hr = md.xmat[hand].reshape(3, 3)
                pose = np.eye(4)
                pose[:3, :3] = r.T @ hr
                pose[:3, 3] = r.T @ (md.xpos[hand]+hr @ [.05, 0, 0]-t)
                mp.append(pose)
            row[frame] = pose_errors(pp, mp)
        result.append(row)
    return result


def upstream_audit():
    def fetch(url):
        with urllib.request.urlopen(url, timeout=30) as response:
            return response.read()
    repo = "unitreerobotics/unitree_ros"
    sha = json.loads(fetch(f"https://api.github.com/repos/{repo}/commits/master"))["sha"]
    result = dict(unitree_ros_revision=sha, files=[])
    for filename in ("g1_29dof.urdf", "g1_29dof_with_hand.urdf", "g1_29dof_rev_1_0.urdf", "g1_29dof_with_hand_rev_1_0.urdf"):
        path = f"robots/g1_description/{filename}"
        content = fetch(f"https://raw.githubusercontent.com/{repo}/{sha}/{path}")
        result["files"].append(dict(url=f"https://github.com/{repo}/blob/{sha}/{path}",
                                    sha256=hashlib.sha256(content).hexdigest(), **urdf_signature(content)))
    result["mode_table_url"] = f"https://github.com/{repo}/blob/{sha}/robots/g1_description/README.md"
    for repo, path in (("unitreerobotics/xr_teleoperate", "assets/g1/g1_body29_hand14.urdf"),
                       ("unitreerobotics/unitree_mujoco", "unitree_robots/g1/g1_29dof.xml")):
        commits = json.loads(fetch(f"https://api.github.com/repos/{repo}/commits?path={path}&per_page=10"))
        result[repo] = [dict(sha=c["sha"], date=c["commit"]["author"]["date"],
                             message=c["commit"]["message"], url=c["html_url"]) for c in commits]
        if repo.endswith("xr_teleoperate"):
            result["xr_urdf_versions"] = []
            for commit in commits:
                revision = commit["sha"]
                content = fetch(f"https://raw.githubusercontent.com/{repo}/{revision}/{path}")
                result["xr_urdf_versions"].append(dict(revision=revision,
                    url=f"https://github.com/{repo}/blob/{revision}/{path}",
                    sha256=hashlib.sha256(content).hexdigest(), **urdf_signature(content)))
    return result


def main():
    from huggingface_hub import snapshot_download
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upstream", action="store_true", help="Also fetch upstream file history and versioned URDF signatures.")
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    assets = Path(snapshot_download("lerobot/unitree-g1-mujoco", revision=HUB_REVISION, local_files_only=True))/"assets"
    urdf, mjcf = assets/"g1_body29_hand14.urdf", assets/"g1_29dof_with_hand.xml"
    report = dict(hub_revision=HUB_REVISION, urdf_signature=urdf_signature(urdf.read_bytes()),
                  local_files={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in (urdf, mjcf)},
                  cases=compare_models(urdf, mjcf))
    if args.upstream:
        report["upstream"] = upstream_audit()
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2)+"\n")
    for case in report["cases"]:
        print(case["name"], {frame: max(e["position_m"] for e in case[frame]) for frame in ("pelvis", "torso_link")})
    print(args.report)


if __name__ == "__main__":
    main()
