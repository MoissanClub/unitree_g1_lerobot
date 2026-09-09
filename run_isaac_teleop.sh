#!/usr/bin/env bash
set -euo pipefail

# Per-session CloudXR launcher for Isaac Teleop.
# Run this whenever you want the VR headset browser client to connect.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LEROBOT_ROOT="${LEROBOT_ROOT:-/home/dwei/lerobot-sim/lerobot}"
VENV_DIR="${VENV_DIR:-/home/dwei/.venvs/isaacteleop}"
CLOUDXR_ENV_FILE="${CLOUDXR_ENV_FILE:-${SCRIPT_DIR}/cloudxr_quest3.env}"
G1_PYTHON_BIN="${G1_PYTHON_BIN:-/home/dwei/miniforge3/envs/lerobot-g1/bin/python}"

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  echo "Isaac Teleop venv not found: ${VENV_DIR}" >&2
  echo "Run ./setup_isaac_teleop.sh first." >&2
  exit 1
fi

if [[ ! -d "${LEROBOT_ROOT}" ]]; then
  echo "LeRobot checkout not found: ${LEROBOT_ROOT}" >&2
  exit 1
fi

if [[ ! -f "${CLOUDXR_ENV_FILE}" ]]; then
  echo "CloudXR Quest3 env file not found: ${CLOUDXR_ENV_FILE}" >&2
  echo "Run ./setup_isaac_teleop.sh first, or set CLOUDXR_ENV_FILE." >&2
  exit 1
fi

if [[ "${SKIP_G1_STARTUP_DIAGNOSTIC:-0}" != "1" ]]; then
  "${G1_PYTHON_BIN}" "${SCRIPT_DIR}/g1_startup_diagnostic.py" lower --duration-s 3.0 --confirm
fi

echo "== Starting Isaac Teleop CloudXR =="
echo "LeRobot root:       ${LEROBOT_ROOT}"
echo "Virtualenv:         ${VENV_DIR}"
echo "CloudXR env file:   ${CLOUDXR_ENV_FILE}"
echo
echo "Headset browser:"
echo "  1. Open https://nvidia.github.io/IsaacTeleop/client"
echo "  2. Set/leave headset profile as Quest3"
echo "  3. Enter this workstation IP:"
hostname -I || true
echo "  4. If prompted, accept https://<workstation-ip>:48322/"
echo "  5. Enter XR and connect"
echo

cd "${LEROBOT_ROOT}"
exec "${VENV_DIR}/bin/python" -m isaacteleop.cloudxr --accept-eula   --cloudxr-env-config "${CLOUDXR_ENV_FILE}"
