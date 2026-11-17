#!/usr/bin/env bash
# Run a timed XRoboToolkit -> G1-29 MuJoCo -> Quest-video end-to-end test.

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

usage() {
    cat <<'EOF'
usage: 05_live_e2e.sh [--duration-s WHOLE_SECONDS] [--unguided]

The simulator, MuJoCo viewer, XR source, and Quest video listener become ready
before the script accepts "wearing the headset now". Ten seconds later, spoken
guidance releases the control gate. The run ends automatically and voice tells
the operator when it is safe to remove the headset.
EOF
}

duration_s=60
guided=true
while [[ $# -gt 0 ]]; do
    case "$1" in
        --duration-s) duration_s="${2:?--duration-s requires a value}"; shift ;;
        --unguided) guided=false ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done
if [[ ! "${duration_s}" =~ ^[1-9][0-9]*$ ]]; then
    echo "--duration-s must be a positive whole number" >&2
    exit 2
fi
if [[ "${guided}" == true && ${duration_s} -lt 24 ]]; then
    echo "Guided mode needs at least 24 seconds; use a longer duration or --unguided." >&2
    exit 2
fi

require_precheck
directory="$(session_dir)"
run_id="live-e2e-$(date +%Y%m%d-%H%M%S)"
run_dir="${directory}/live/${run_id}"
ready_file="${run_dir}/backend.ready"
start_gate="${run_dir}/operator.start"
backend_log="${run_dir}/backend.log"
telemetry="${run_dir}/telemetry.json"
video_root="${run_dir}/video"
mkdir -p "${run_dir}" "${video_root}"

backend_pid=""
cleanup() {
    if [[ -n "${backend_pid}" ]] && kill -0 "${backend_pid}" 2>/dev/null; then
        kill -TERM "${backend_pid}" 2>/dev/null || true
        wait "${backend_pid}" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

record_event "${run_id}: preparing MuJoCo viewer, XR input, and Quest video"
PYTHONUNBUFFERED=1 "${PYTHON_BIN}" "${REPO_ROOT}/tools/run_supervised_g1_29_vr.py" \
    --source xrobotoolkit \
    --urdf "${G1_URDF}" \
    --mesh-dir "${G1_MESH_DIR}" \
    --sdk-site-packages "${XRBT_SDK_SITE_PACKAGES}" \
    --duration-s "${duration_s}" \
    --rate-hz 60 \
    --telemetry "${telemetry}" \
    --viewer \
    --quest-video \
    --video-display "${DISPLAY}" \
    --video-evidence-root "${video_root}" \
    --ready-file "${ready_file}" \
    --start-gate-file "${start_gate}" \
    > "${backend_log}" 2>&1 &
backend_pid=$!

if ! wait_for_file "${ready_file}" "${backend_pid}" 120; then
    show_process_log_on_failure "${backend_log}"
    exit 1
fi
echo "MuJoCo viewer, XR input, and Quest video listener are ready and gated."
confirm_headset_handoff

if [[ "${guided}" == true ]]; then
    voice_async "Live test starts now. Confirm the MuJoCo robot is visible. Hold only the right grip and move the right controller."
else
    voice_async "Live end to end test starts now."
fi
printf '%s\n' "$(date --iso-8601=seconds)" > "${start_gate}"
record_event "${run_id}: operator gate released"

if [[ "${guided}" == true ]]; then
    segment=$((duration_s / 4))
    sleep "${segment}"
    if kill -0 "${backend_pid}" 2>/dev/null; then
        voice_async "Release the right grip. Hold only the left grip and move the left controller."
    fi
    sleep "${segment}"
    if kill -0 "${backend_pid}" 2>/dev/null; then
        voice_async "Now hold both grips and move both controllers slowly."
    fi
    sleep "${segment}"
    if kill -0 "${backend_pid}" 2>/dev/null; then
        voice_async "Release both grips. The arms should hold. Continue observing the video."
    fi
fi

set +e
wait "${backend_pid}"
backend_status=$?
set -e
backend_pid=""
if (( backend_status != 0 )); then
    show_process_log_on_failure "${backend_log}"
    confirm_headset_removed || true
    exit "${backend_status}"
fi

record_event "${run_id}: timed end-to-end run completed"
confirm_headset_removed
trap - EXIT INT TERM
echo "End-to-end run complete."
echo "Telemetry: ${telemetry}"
echo "Video evidence: ${video_root}"
echo "Backend log: ${backend_log}"
