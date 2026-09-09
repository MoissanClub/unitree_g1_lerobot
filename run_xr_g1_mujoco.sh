#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

PYTHON_BIN=/home/dwei/miniforge3/envs/lerobot-g1/bin/python

if [[ "${SKIP_G1_STARTUP_DIAGNOSTIC:-0}" != "1" ]]; then
  "${PYTHON_BIN}" g1_startup_diagnostic.py hands-up --confirm
fi

PYTHONUNBUFFERED=1 \
  "${PYTHON_BIN}" \
  rung3_xr_to_g1_mujoco.py "$@"
