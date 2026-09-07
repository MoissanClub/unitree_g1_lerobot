#!/usr/bin/env bash
set -euo pipefail

LEROBOT_ROOT="${LEROBOT_ROOT:-/home/dwei/lerobot-sim/lerobot}"
VENV_DIR="${VENV_DIR:-/home/dwei/.venvs/isaacteleop}"
CLOUDXR_ENV_FILE="${CLOUDXR_ENV_FILE:-${LEROBOT_ROOT}/examples/isaac_teleop_to_so101/default.env}"

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  echo "Isaac Teleop venv not found: ${VENV_DIR}" >&2
  exit 1
fi

if [[ ! -f "${CLOUDXR_ENV_FILE}" ]]; then
  echo "CloudXR Quest3 env file not found: ${CLOUDXR_ENV_FILE}" >&2
  exit 1
fi

echo "Starting CloudXR with Quest3 profile"
echo "  venv:             ${VENV_DIR}"
echo "  CloudXR env file: ${CLOUDXR_ENV_FILE}"
echo
echo "On the headset client, set/leave the headset profile as Quest3 too."
echo

cd "${LEROBOT_ROOT}"
exec "${VENV_DIR}/bin/python" -m isaacteleop.cloudxr --accept-eula \
  --cloudxr-env-config "${CLOUDXR_ENV_FILE}"
