#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
export MUJOCO_GL="${MUJOCO_GL:-egl}"

exec /home/dwei/miniforge3/envs/lerobot-g1/bin/python -m unitree_g1_lerobot.simulation.g1_compare_ik_viewer "$@"
