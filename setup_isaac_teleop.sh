#!/usr/bin/env bash
set -euo pipefail

# One-time Isaac Teleop + LeRobot setup for CloudXR headset testing.
# Defaults match the workstation setup used for the unitree_g1_lerobot ladder.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LEROBOT_ROOT="${LEROBOT_ROOT:-/home/dwei/lerobot-sim/lerobot}"
VENV_DIR="${VENV_DIR:-/home/dwei/.venvs/isaacteleop}"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3.12}"
CLOUDXR_ENV_FILE="${CLOUDXR_ENV_FILE:-${SCRIPT_DIR}/configs/cloudxr_quest3.env}"

echo "== Isaac Teleop one-time setup =="
echo "LeRobot root:       ${LEROBOT_ROOT}"
echo "Virtualenv:         ${VENV_DIR}"
echo "Python:             ${PYTHON_BIN}"
echo "CloudXR env file:   ${CLOUDXR_ENV_FILE}"

if [[ ! -d "${LEROBOT_ROOT}" ]]; then
  echo "LeRobot checkout not found: ${LEROBOT_ROOT}" >&2
  exit 1
fi

if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "Python interpreter not found or not executable: ${PYTHON_BIN}" >&2
  exit 1
fi

if ! command -v uv >/dev/null 2>&1; then
  echo "uv is required but was not found on PATH." >&2
  echo "Install uv first, then rerun this script." >&2
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
echo "== Recreate clean Isaac Teleop venv =="
"${PYTHON_BIN}" -m venv --clear "${VENV_DIR}"

echo
echo "== Install LeRobot + Isaac Teleop =="
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
echo "== Write Quest3 CloudXR env file =="
cat >"${CLOUDXR_ENV_FILE}" <<'EOF'
# CloudXR runtime profile for the browser headset client.
# Keep this set to Quest3 when the headset-side browser client profile is Quest3.
NV_DEVICE_PROFILE=Quest3
NV_CXR_ENABLE_PUSH_DEVICES=true
NV_CXR_ENABLE_TENSOR_DATA=true
NV_CXR_FILE_LOGGING=true
EOF
echo "Wrote ${CLOUDXR_ENV_FILE}"

echo
echo "== Verify install =="
"${VENV_DIR}/bin/python" - <<'PY'
import importlib.metadata as md
import torch
from isaacteleop.cloudxr import CloudXRLauncher

print(f"isaacteleop={md.version('isaacteleop')}")
print(f"torch={torch.__version__}")
print(f"torch_cuda_available={torch.cuda.is_available()}")
print("CloudXRLauncher import ok")
PY

"${VENV_DIR}/bin/python" -m isaacteleop.cloudxr --help >/dev/null
"${VENV_DIR}/bin/python" -m examples.isaac_teleop_to_so101.teleoperate --help >/dev/null
echo "CloudXR CLI and LeRobot Isaac Teleop example CLI are runnable."

echo
echo "== CloudXR EULA/firewall notes =="
if [[ -f "${HOME}/.cloudxr/run/eula_accepted" ]]; then
  echo "CloudXR EULA marker exists: ${HOME}/.cloudxr/run/eula_accepted"
else
  cat <<EOF
CloudXR EULA marker is missing. The run script passes --accept-eula, but the first
runtime launch may still print EULA-related output.
EOF
fi

cat <<EOF

Open CloudXR firewall ports if they are not already open:
  sudo ufw allow 47998/udp
  sudo ufw allow 49100,48322/tcp

Workstation IP candidates:
$(hostname -I || true)

One-time setup is complete.

Last step: run the smoke test before moving on:
  source "${VENV_DIR}/bin/activate"
  cd "${SCRIPT_DIR}"
  ./run_isaac_teleop.sh

In another terminal, after the headset browser is connected:
  source "${VENV_DIR}/bin/activate"
  cd "${SCRIPT_DIR}"
  python -m unitree_g1_lerobot.xr.xr_controller_cloudxr_smoke_test --external-cloudxr
EOF
