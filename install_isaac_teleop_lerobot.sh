#!/usr/bin/env bash
set -euo pipefail

# Reproduce the Isaac Teleop installation performed for:
#   /home/dwei/lerobot-sim/lerobot
#
# This intentionally uses the LeRobot-compatible Isaac Teleop pin
# (~=1.3.131), whose CloudXR CLI entry point is:
#   python -m isaacteleop.cloudxr
# not the newer:
#   python -m isaacteleop.cloudxr.service

LEROBOT_ROOT="${LEROBOT_ROOT:-/home/dwei/lerobot-sim/lerobot}"
VENV_DIR="${VENV_DIR:-/home/dwei/.venvs/isaacteleop}"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3.12}"

echo "== Isaac Teleop LeRobot install =="
echo "LeRobot root: ${LEROBOT_ROOT}"
echo "Virtualenv:   ${VENV_DIR}"
echo "Python:       ${PYTHON_BIN}"

if [[ ! -d "${LEROBOT_ROOT}" ]]; then
  echo "LeRobot checkout not found: ${LEROBOT_ROOT}" >&2
  exit 1
fi

if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "Python interpreter not found or not executable: ${PYTHON_BIN}" >&2
  exit 1
fi

echo
echo "== Platform checks =="
uname -m
"${PYTHON_BIN}" -V
if command -v lsb_release >/dev/null 2>&1; then
  lsb_release -a
fi
if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi
else
  echo "nvidia-smi not found; verify NVIDIA driver/CUDA manually." >&2
fi

echo
echo "== Recreate clean venv =="
"${PYTHON_BIN}" -m venv --clear "${VENV_DIR}"

echo
echo "== Install dependencies =="
cd "${LEROBOT_ROOT}"
uv pip install \
  --python "${VENV_DIR}/bin/python" \
  -e ".[feetech,kinematics,dataset]" \
  "huggingface_hub>=1.5" \
  "isaacteleop[cloudxr,retargeters-lite]~=1.3.131" \
  "scipy>=1.14" \
  --extra-index-url https://pypi.nvidia.com \
  --prerelease=allow

echo
echo "== Verify install =="
"${VENV_DIR}/bin/python" - <<'PY'
import importlib.metadata as md
import ctypes
import torch
import isaacteleop
from isaacteleop.cloudxr import CloudXRLauncher

print(f"isaacteleop={md.version('isaacteleop')}")
print(f"torch={torch.__version__}")
print(f"torch_cuda_available={torch.cuda.is_available()}")
print("ctypes import ok")
print("CloudXRLauncher import ok")
PY

"${VENV_DIR}/bin/python" -m isaacteleop.cloudxr --help >/dev/null
"${VENV_DIR}/bin/python" -m examples.isaac_teleop_to_so101.teleoperate --help >/dev/null
echo "CloudXR CLI and LeRobot Isaac Teleop example CLI are runnable."

echo
echo "== EULA/firewall status =="
if [[ -f "${HOME}/.cloudxr/run/eula_accepted" ]]; then
  echo "CloudXR EULA marker exists: ${HOME}/.cloudxr/run/eula_accepted"
else
  cat <<EOF
CloudXR EULA marker is missing. Accept it interactively with:
  source "${VENV_DIR}/bin/activate"
  cd "${LEROBOT_ROOT}"
  python -m isaacteleop.cloudxr --accept-eula
EOF
fi

cat <<EOF

Open CloudXR firewall ports if they are not already open:
  sudo ufw allow 47998/udp
  sudo ufw allow 49100,48322/tcp

Run XR SO-101 teleop with:
  source "${VENV_DIR}/bin/activate"
  cd "${LEROBOT_ROOT}"
  python -m examples.isaac_teleop_to_so101.teleoperate \\
    --robot.type=so101_follower \\
    --robot.port=/dev/ttyACM0 \\
    --robot.id=so101_follower_arm \\
    --teleop.type=xr_controller

Workstation IP candidates:
EOF
hostname -I || true
