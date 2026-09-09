#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

LEROBOT_MUJOCO_GL=egl PYTHONUNBUFFERED=1 \
  /home/dwei/miniforge3/envs/lerobot-g1/bin/python \
  -m unitree_g1_lerobot.simulation.g1_mujoco_dds_sim "$@"
