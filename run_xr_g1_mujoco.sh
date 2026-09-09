#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

PYTHONUNBUFFERED=1 \
  /home/dwei/miniforge3/envs/lerobot-g1/bin/python \
  rung3_xr_to_g1_mujoco.py "$@"
