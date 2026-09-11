#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
PYTHON_BIN="${G1_PYTHON_BIN:-${HOME}/miniforge3/envs/lerobot-g1/bin/python}"
exec "${PYTHON_BIN}" -m unitree_g1_lerobot.diagnostics.simulation.compare_motor_configs --comparison embodiments --gravity-compensation "$@"
