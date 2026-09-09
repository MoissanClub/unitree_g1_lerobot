#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
LEROBOT_ROOT="${LEROBOT_ROOT:-${ROOT}/../lerobot}"
PATCH="${ROOT}/patches/lerobot-g1-embodiments.patch"
if git -C "$LEROBOT_ROOT" apply --reverse --check "$PATCH" 2>/dev/null; then
    printf '%s\n' 'LeRobot G1 embodiment patch is already applied.'
elif git -C "$LEROBOT_ROOT" apply --check "$PATCH"; then
    git -C "$LEROBOT_ROOT" apply "$PATCH"
    printf '%s\n' 'Applied LeRobot G1 embodiment patch.'
else
    printf '%s\n' 'Patch does not match this LeRobot checkout. No changes applied; inspect the checkout and patch.' >&2
    exit 1
fi
