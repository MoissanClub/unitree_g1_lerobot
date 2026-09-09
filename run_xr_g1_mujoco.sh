#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

PYTHON_BIN=/home/dwei/miniforge3/envs/lerobot-g1/bin/python

if [[ "${SKIP_G1_STARTUP_DIAGNOSTIC:-0}" != "1" ]]; then
  "${PYTHON_BIN}" -m unitree_g1_lerobot.diagnostics.g1_startup_diagnostic hands-up --confirm
fi

PYTHONUNBUFFERED=1 \
  "${PYTHON_BIN}" \
  -m unitree_g1_lerobot.xr.rung3_xr_to_g1_mujoco "$@"
