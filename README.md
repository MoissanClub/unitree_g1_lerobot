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
