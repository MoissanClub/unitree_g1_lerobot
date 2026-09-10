#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export LEROBOT_ROOT="${LEROBOT_ROOT:-${PWD}/../lerobot}"
if [[ ! -f "${LEROBOT_ROOT}/src/lerobot/robots/unitree_g1/g1_runtime.py" ]]; then
  echo 'Set LEROBOT_ROOT to the audited LeRobot checkout with the embodiment patch applied.' >&2
  exit 2
fi
exec "${G1_INIT_PYTHON:-python3}" -m unittest discover -s tests -p test_lerobot_physical_initialization.py -v
