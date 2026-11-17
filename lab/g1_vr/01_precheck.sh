#!/usr/bin/env bash
# Run all headset-free readiness checks and create a lab evidence session.

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

usage() {
    echo "usage: $0 [--skip-audio] [--skip-nvenc-probe] [--session-dir PATH]"
}

skip_audio=false
skip_nvenc=false
requested_session=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --skip-audio) skip_audio=true ;;
        --skip-nvenc-probe) skip_nvenc=true ;;
        --session-dir)
            requested_session="${2:?--session-dir requires a path}"
            shift
            ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

stamp="$(date +%Y%m%d-%H%M%S)"
directory="${requested_session:-${OUTPUT_ROOT}/${stamp}}"
mkdir -p "${directory}" "${OUTPUT_ROOT}"
if [[ -e "${CURRENT_SESSION_FILE}" ]]; then
    previous="$(sed -n '1p' "${CURRENT_SESSION_FILE}")"
    if [[ "${previous}" == "${directory}" ]]; then
        echo "Session directory is already current: ${directory}" >&2
        exit 2
    fi
fi
printf '%s\n' "${directory}" > "${CURRENT_SESSION_FILE}"
export G1_VR_LAB_SESSION_DIR="${directory}"
exec > >(tee "${directory}/precheck.log") 2>&1

failures=0
warnings=0
pass() { echo "PASS  $*"; }
warn() { echo "WARN  $*"; warnings=$((warnings + 1)); }
fail() { echo "FAIL  $*"; failures=$((failures + 1)); }

echo "G1-29 XR simulation-lab precheck"
echo "Session: ${directory}"
echo "Started: $(date --iso-8601=seconds)"
echo "Scope: simulation only; no physical DDS or robot motion"

for command in git ss spd-say nvidia-smi ffmpeg sha256sum; do
    if command -v "${command}" >/dev/null 2>&1; then
        pass "command available: ${command}"
    else
        fail "missing command: ${command}"
    fi
done

for item in "${PYTHON_BIN}" "${LEROBOT_ROOT}" "${PLUGIN_ROOT}" "${G1_URDF}" "${G1_MESH_DIR}"; do
    if [[ -e "${item}" ]]; then
        pass "required path exists: ${item}"
    else
        fail "required path missing: ${item:-<unset>}"
    fi
done
if [[ -n "${XRBT_SDK_SITE_PACKAGES}" && -d "${XRBT_SDK_SITE_PACKAGES}" ]]; then
    pass "XRoboToolkit SDK site-packages: ${XRBT_SDK_SITE_PACKAGES}"
else
    fail "XRBT_SDK_SITE_PACKAGES is unset or ambiguous; export its exact directory"
fi

if "${PYTHON_BIN}" - <<'PY'
import lerobot
import lerobot_teleoperator_g1_vr
import mujoco
import numpy
import pinocchio
import unitree_g1_lerobot
PY
then
    pass "pinned Python imports: LeRobot, plugin, MuJoCo, NumPy, Pinocchio, integration"
else
    fail "pinned Python import check failed"
fi

for repository in "${REPO_ROOT}" "${PLUGIN_ROOT}" "${LEROBOT_ROOT}"; do
    if revision="$(git -C "${repository}" rev-parse HEAD 2>/dev/null)"; then
        echo "REV   ${repository}: ${revision}"
        git -C "${repository}" status --short --branch || true
    else
        fail "not a readable Git checkout: ${repository}"
    fi
done

echo "HASH  g1_29.urdf: $(sha256sum "${G1_URDF}" | cut -d' ' -f1)"

if nvidia-smi --query-gpu=index,name,driver_version,memory.total --format=csv,noheader; then
    pass "NVIDIA GPU and driver visible"
else
    fail "NVIDIA GPU check failed"
fi
encoder_listing="$(ffmpeg -hide_banner -encoders 2>/dev/null || true)"
if [[ "${encoder_listing}" == *h264_nvenc* ]]; then
    pass "FFmpeg exposes h264_nvenc"
else
    fail "FFmpeg does not expose h264_nvenc"
fi
if [[ "${skip_nvenc}" == true ]]; then
    warn "direct NVENC probe skipped"
else
    if ffmpeg -hide_banner -loglevel error -f lavfi -i color=size=256x256:rate=1 \
        -frames:v 1 -c:v h264_nvenc -f null -; then
        pass "direct NVENC encode probe"
    else
        fail "direct NVENC encode probe failed"
    fi
fi

if [[ -n "${DISPLAY:-}" ]]; then
    pass "desktop display is ${DISPLAY}"
else
    fail "DISPLAY is unset; MuJoCo viewer and desktop video need a local display"
fi

listeners="$(ss -ltnp 2>/dev/null || ss -ltn 2>/dev/null || true)"
if grep -q ':60061' <<<"${listeners}" && grep -q ':63901' <<<"${listeners}"; then
    pass "XRoboToolkit PC Service listeners 60061 and 63901 are active"
else
    fail "XRoboToolkit PC Service is not listening on both 60061 and 63901"
fi
if grep -q ':13579' <<<"${listeners}"; then
    fail "Quest video port 13579 is already in use"
else
    pass "Quest video port 13579 is free"
fi

if ps -eo args= | grep -E '[r]un_g1_mujoco_dds_sim|[u]nitree.*dds.*sim' >/dev/null; then
    fail "a physical/DDS-oriented G1 process appears to be running"
else
    pass "no G1 DDS simulator/control process detected"
fi

available_kb="$(df -Pk "${OUTPUT_ROOT}" | awk 'NR==2 {print $4}')"
if [[ "${available_kb}" =~ ^[0-9]+$ ]] && (( available_kb >= 5 * 1024 * 1024 )); then
    pass "at least 5 GiB is free for lab evidence"
else
    fail "less than 5 GiB is free for lab evidence"
fi

if [[ "${skip_audio}" == true ]]; then
    warn "audio probe skipped"
elif "${TTS_BIN}" -w -r "${VOICE_RATE}" "Precheck audio is working."; then
    pass "voice prompt command completed; operator must confirm audibility"
else
    fail "voice prompt command failed"
fi

cat > "${directory}/session.env" <<EOF
G1_VR_LAB_SESSION_DIR=${directory}
G1_VR_PLUGIN_ROOT=${PLUGIN_ROOT}
G1_VR_LEROBOT_ROOT=${LEROBOT_ROOT}
G1_VR_PYTHON=${PYTHON_BIN}
G1_VR_ASSETS_ROOT=${ASSETS_ROOT}
XRBT_SDK_SITE_PACKAGES=${XRBT_SDK_SITE_PACKAGES}
EOF

record_event "precheck completed: failures=${failures}, warnings=${warnings}"
echo
echo "Precheck result: ${failures} failure(s), ${warnings} warning(s)"
echo "Evidence session: ${directory}"
if (( failures > 0 )); then
    echo "Do not put on the headset. Resolve failures and rerun precheck." >&2
    exit 1
fi
printf '%s\n' "$(date --iso-8601=seconds)" > "${directory}/precheck.passed"
echo "Headset-free readiness passed. The Quest app/link is checked after each gated handoff."
