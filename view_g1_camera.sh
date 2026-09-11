#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
exec /home/dwei/miniforge3/envs/lerobot-g1/bin/python -m unitree_g1_lerobot.diagnostics.simulation.view_robot_camera "$@"
