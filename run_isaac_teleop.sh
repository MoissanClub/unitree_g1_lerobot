#!/usr/bin/env bash
set -euo pipefail

# CloudXR service only; robot control belongs to a separate teleoperation bridge.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${VENV_DIR:-/home/dwei/.venvs/isaacteleop}"
CLOUDXR_ENV_FILE="${CLOUDXR_ENV_FILE:-${SCRIPT_DIR}/configs/cloudxr_quest3.env}"
DURATION_S=0
while (( $# )); do
  case "$1" in
    --headless) shift ;; # Always runs without a local viewer or prompts.
    --duration-s) DURATION_S="${2:?Missing duration}"; shift 2 ;;
    --help|-h)
      echo "Usage: $0 [--headless] [--duration-s N]"
      exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  echo "Isaac Teleop venv not found: ${VENV_DIR}" >&2
  echo "Run ./setup_isaac_teleop.sh first." >&2
  exit 1
fi
if [[ ! -f "${CLOUDXR_ENV_FILE}" ]]; then
  echo "CloudXR Quest3 env file not found: ${CLOUDXR_ENV_FILE}" >&2
  echo "Run ./setup_isaac_teleop.sh first, or set CLOUDXR_ENV_FILE." >&2
  exit 1
fi

echo "== Starting Isaac Teleop CloudXR =="
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

cd "${SCRIPT_DIR}"
exec "${VENV_DIR}/bin/python" -u -m unitree_g1_lerobot.xr.cloudxr_session \
  --duration-s "${DURATION_S}" --cloudxr-env-config "${CLOUDXR_ENV_FILE}"
