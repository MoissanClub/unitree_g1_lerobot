#!/usr/bin/env bash
set -euo pipefail

# Per-session CloudXR launcher for Isaac Teleop.
# Run this whenever you want the VR headset browser client to connect.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LEROBOT_ROOT="${LEROBOT_ROOT:-/home/dwei/lerobot-sim/lerobot}"
VENV_DIR="${VENV_DIR:-/home/dwei/.venvs/isaacteleop}"
CLOUDXR_ENV_FILE="${CLOUDXR_ENV_FILE:-${SCRIPT_DIR}/configs/cloudxr_quest3.env}"
G1_PYTHON_BIN="${G1_PYTHON_BIN:-/home/dwei/miniforge3/envs/lerobot-g1/bin/python}"
G1_DIAGNOSTIC_REQUEST_FILE="${G1_DIAGNOSTIC_REQUEST_FILE:-/tmp/g1_mujoco_startup_diagnostic.request}"
G1_DIAGNOSTIC_ACK_FILE="${G1_DIAGNOSTIC_ACK_FILE:-/tmp/g1_mujoco_startup_diagnostic.ack}"

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

run_bridge_lower_diagnostic() {
  local request_id
  request_id="run_isaac_teleop_${EPOCHREALTIME}_$$"
  rm -f "${G1_DIAGNOSTIC_ACK_FILE}"
  printf '{"id":"%s","mode":"lower_hold"}\n' "${request_id}" > "${G1_DIAGNOSTIC_REQUEST_FILE}"

  echo "== Startup diagnostic: requesting XR bridge to lower both G1 arms =="
  echo "Waiting for ./run_xr_g1_mujoco.sh to acknowledge the diagnostic request..."

  local deadline=$((SECONDS + 8))
  while (( SECONDS < deadline )); do
    if [[ -f "${G1_DIAGNOSTIC_ACK_FILE}" ]] && [[ "$(<"${G1_DIAGNOSTIC_ACK_FILE}")" == *"${request_id}"* ]]; then
      if ! read -r -p "Press Enter after you verify the diagnostic motion of both arms lowering, or Ctrl+C to abort..."; then
        echo "Startup diagnostic confirmation could not read from stdin; aborting." >&2
        rm -f "${G1_DIAGNOSTIC_REQUEST_FILE}" "${G1_DIAGNOSTIC_ACK_FILE}"
        return 3
      fi
      rm -f "${G1_DIAGNOSTIC_REQUEST_FILE}" "${G1_DIAGNOSTIC_ACK_FILE}"
      echo "== User verification confirmed; entering steady-state listening =="
      return 0
    fi
    sleep 0.1
  done

  rm -f "${G1_DIAGNOSTIC_REQUEST_FILE}"
  return 1
}

if [[ "${SKIP_G1_STARTUP_DIAGNOSTIC:-0}" != "1" ]]; then
  diagnostic_rc=0
  run_bridge_lower_diagnostic || diagnostic_rc=$?
  if [[ ${diagnostic_rc} -eq 3 ]]; then
    exit 2
  elif [[ ${diagnostic_rc} -ne 0 ]]; then
    echo "XR bridge did not acknowledge the lowering diagnostic; falling back to direct DDS diagnostic." >&2
    (cd "${SCRIPT_DIR}" && "${G1_PYTHON_BIN}" -m unitree_g1_lerobot.diagnostics.g1_startup_diagnostic lower --duration-s 3.0 --confirm)
  fi
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
