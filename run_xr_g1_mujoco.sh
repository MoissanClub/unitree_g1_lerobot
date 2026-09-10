#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

PYTHON_BIN=/home/dwei/miniforge3/envs/lerobot-g1/bin/python

PYTHONUNBUFFERED=1 \
  "${PYTHON_BIN}" \
  -m unitree_g1_lerobot.xr.rung3_xr_to_g1_mujoco "$@"
