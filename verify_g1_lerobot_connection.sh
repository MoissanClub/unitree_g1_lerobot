#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
exec "${G1_INIT_PYTHON:-python3}" -m unitree_g1_lerobot.diagnostics.verify_lerobot_connection "$@"
