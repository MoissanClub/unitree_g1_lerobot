# G1Arm23LeRobot

Supporting G1Arm23 in LeRobot.

## Isaac Teleop CloudXR Scripts

Use these scripts for the Isaac Teleop / CloudXR part of the ladder.

### 1. One-Time Setup

Run this once after cloning the repo:

```bash
cd ~/lerobot-sim/G1Arm23LeRobot
./setup_isaac_teleop.sh
```

The setup script:

- creates a clean Isaac Teleop virtualenv at `/home/dwei/.venvs/isaacteleop`
- installs the local LeRobot checkout with the Isaac Teleop-compatible dependencies
- writes `cloudxr_quest3.env`
- verifies the CloudXR and LeRobot Isaac Teleop CLIs
- ends by asking you to run the smoke test

Defaults can be overridden:

```bash
LEROBOT_ROOT=/path/to/lerobot \
VENV_DIR=/path/to/venv \
PYTHON_BIN=/usr/bin/python3.12 \
./setup_isaac_teleop.sh
```

### 2. Start CloudXR For Each VR Session

Run this whenever you want to connect the VR headset:

```bash
cd ~/lerobot-sim/G1Arm23LeRobot
./run_isaac_teleop.sh
```

On the headset browser:

```text
https://nvidia.github.io/IsaacTeleop/client
```

Set the headset-side profile to `Quest3`, then connect to the workstation IP printed by
`run_isaac_teleop.sh`. The local CloudXR runtime also uses `Quest3` through
`cloudxr_quest3.env`; both sides need to match.

### 3. Smoke Test

With `run_isaac_teleop.sh` still running, open another terminal and run:

```bash
source /home/dwei/.venvs/isaacteleop/bin/activate
cd ~/lerobot-sim/G1Arm23LeRobot
python xr_controller_cloudxr_smoke_test.py --external-cloudxr
```

Expected result: the script connects to the existing CloudXR runtime and prints live
controller pose/squeeze values from the headset.

## Rung 3: XR Controller To G1 MuJoCo

Rung 3 joins the CloudXR controller path with the G1 IK/MuJoCo path. Use the
`lerobot-g1` conda environment through the repo-local wrapper; it keeps conda-forge
Pinocchio/CasADi first and appends the existing Isaac Teleop venv only for XR imports.

Before a headset run, verify the G1 MuJoCo visual path from an `ssh -Y` terminal:

```bash
cd ~/lerobot-sim/G1Arm23LeRobot
./run_g1_mujoco_keyboard.sh
```

Validate rung 3 without headset or CloudXR:

```bash
cd ~/lerobot-sim/G1Arm23LeRobot
./run_xr_g1_mujoco.sh --dry-run-ik
./run_xr_g1_mujoco.sh --mock-xr --duration-s 5 --no-wait
```

For the ergonomic real headset run, start the lightweight pieces before putting on the
headset.

Terminal A, start G1 MuJoCo as a standalone DDS simulator. It first prints
`raising robot arm` and pauses after the motion so you can verify the viewer before
steady-state simulation starts:

```bash
cd ~/lerobot-sim/G1Arm23LeRobot
./run_g1_mujoco_dds_sim.sh
```

Terminal C, start the XR-to-G1 bridge before CloudXR exists. It first simulates a
CloudXR/controller orientation change and pauses so you can verify the right arm changes
orientation. Then it attaches to DDS, sends the G1 to the raised-arm ready pose, and keeps
holding that pose while it retries XR attach:

```bash
cd ~/lerobot-sim/G1Arm23LeRobot
./run_xr_g1_mujoco.sh --external-g1-sim --external-cloudxr --wait-for-cloudxr --no-wait
```

This command runs until Ctrl+C. For bounded smoke tests, pass `--duration-s N`.

If the MuJoCo arms feel too slow, tune only the bridge command gains first:

```bash
./run_xr_g1_mujoco.sh --external-g1-sim --external-cloudxr --wait-for-cloudxr --no-wait --arm-kp-scale 2.0 --arm-kd-scale 1.5
```

Raise `--arm-kp-scale` for faster response. If the arm overshoots or shakes, raise
`--arm-kd-scale` or reduce `--arm-kp-scale`.

Terminal B, start CloudXR. It first simulates an XR device input that lowers both arms and
pauses for confirmation before CloudXR starts:

```bash
cd ~/lerobot-sim/G1Arm23LeRobot
./run_isaac_teleop.sh
```

For non-interactive smoke tests, prefix any launcher with
`SKIP_G1_STARTUP_DIAGNOSTIC=1` to skip these confirmation pauses.

Then put on the headset. In the headset browser, open
`https://nvidia.github.io/IsaacTeleop/client`, use the `Quest3` profile, enter the
workstation IP printed by `run_isaac_teleop.sh`, enter XR, and connect.

This rung uses the headset for controller input only. The MuJoCo robot view is the Tk/X
window opened by `run_g1_mujoco_dds_sim.sh`; the headset client may stay on its CloudXR
control/status screen because `run_xr_g1_mujoco.sh` creates a headless OpenXR session and
does not submit MuJoCo camera frames to VR yet.

Default behavior is conservative: before XR attaches, both arms hold the raised ready pose.
After XR attaches, the right controller drives the right wrist only, the left wrist holds
ready, and the clutch defaults to max(squeeze, trigger) as a hold-to-enable input.

### Rung 3 Runtime Model

`run_isaac_teleop.sh` starts CloudXR as a separate process. The rung 3 script attaches to
that existing CloudXR/OpenXR runtime with `--external-cloudxr`.

MuJoCo has two modes. Without `--external-g1-sim`, `rung3_xr_to_g1_mujoco.py` creates
`UnitreeG1(UnitreeG1Config(is_simulation=True))`, and `robot.connect()` launches the
LeRobot G1 MuJoCo simulation inside that Python process. With `--external-g1-sim`, the
bridge skips simulator creation and only publishes IK-generated G1 joint targets onto DDS;
`run_g1_mujoco_dds_sim.sh` owns the MuJoCo process.

Process split:

```text
A: cd ~/lerobot-sim/G1Arm23LeRobot && ./run_g1_mujoco_dds_sim.sh
C: cd ~/lerobot-sim/G1Arm23LeRobot && ./run_xr_g1_mujoco.sh --external-g1-sim --external-cloudxr --wait-for-cloudxr --no-wait
B: cd ~/lerobot-sim/G1Arm23LeRobot && ./run_isaac_teleop.sh
Headset: connect to CloudXR after A, C, and B are ready
```
