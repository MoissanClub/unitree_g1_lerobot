#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
bash -n run_g1_physical_preflight.sh
exec "${G1_PREFLIGHT_PYTHON:-python3}" -m unittest discover -s tests -p test_physical_preflight.py -v
