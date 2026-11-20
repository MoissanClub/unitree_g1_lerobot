#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"
case "${1:-}" in
    -h|--help|--execute) exec "${PYTHON_BIN}" "${REPO_ROOT}/tools/g1_physical_replay.py" postures "$@" ;;
esac
suite="$(dirname "$(dirname "$(current_anchor_set)")")"
exec "${PYTHON_BIN}" "${REPO_ROOT}/tools/g1_physical_replay.py" postures --suite "${suite}" --urdf "${G1_URDF}" "$@"
