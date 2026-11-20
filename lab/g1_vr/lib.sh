#!/usr/bin/env bash
# Shared configuration and operator handoff helpers for G1-29 VR simulation labs.

set -euo pipefail

LAB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${LAB_DIR}/../.." && pwd)"
OUTPUT_ROOT="${G1_VR_LAB_OUTPUT_ROOT:-${REPO_ROOT}/outputs/g1-vr-lab}"
CURRENT_SESSION_FILE="${OUTPUT_ROOT}/current-session"
PLUGIN_ROOT="${G1_VR_PLUGIN_ROOT:-$(cd "${REPO_ROOT}/../lerobot_teleoperator_g1_vr" 2>/dev/null && pwd || true)}"
LEROBOT_ROOT="${G1_VR_LEROBOT_ROOT:-${REPO_ROOT}/.vr-sim/clean-install-verification/lerobot}"
PYTHON_BIN="${G1_VR_PYTHON:-${REPO_ROOT}/.vr-sim/clean-install-verification/env/bin/python}"
ASSETS_ROOT="${G1_VR_ASSETS_ROOT:-${REPO_ROOT}/.vr-sim/clean-install-verification/assets}"
G1_URDF="${G1_VR_URDF:-${ASSETS_ROOT}/g1_29.urdf}"
G1_MESH_DIR="${G1_VR_MESH_DIR:-${ASSETS_ROOT}/meshes}"
XRBT_WORKSPACE="${XRBT_WORKSPACE:-/home/dwei/meta_quest_xrobotoolkit}"
XRBT_PC_SERVICE_DIR="${G1_VR_XRBT_PC_SERVICE_DIR:-}"
VOICE_RATE="${G1_VR_VOICE_RATE:--35}"
DONNING_SECONDS="${G1_VR_DONNING_SECONDS:-10}"
TTS_BIN="${G1_VR_TTS_BIN:-spd-say}"

if [[ -z "${XRBT_SDK_SITE_PACKAGES:-}" ]]; then
    shopt -s nullglob
    sdk_candidates=(
        "${XRBT_WORKSPACE}"/xrbt-quest-mujoco.*/venv/lib/python3.12/site-packages
    )
    shopt -u nullglob
    if [[ ${#sdk_candidates[@]} -eq 1 ]]; then
        XRBT_SDK_SITE_PACKAGES="${sdk_candidates[0]}"
    else
        XRBT_SDK_SITE_PACKAGES=""
    fi
fi

if [[ -z "${XRBT_PC_SERVICE_DIR}" ]]; then
    shopt -s nullglob
    service_candidates=(
        "${XRBT_WORKSPACE}"/xrbt-quest-mujoco.*/pc-service-root/opt/apps/roboticsservice
        /opt/apps/roboticsservice
    )
    shopt -u nullglob
    for candidate in "${service_candidates[@]}"; do
        if [[ -x "${candidate}/RoboticsServiceProcess" && -x "${candidate}/runService.sh" ]]; then
            XRBT_PC_SERVICE_DIR="${candidate}"
            break
        fi
    done
fi

export PYTHONPATH="${REPO_ROOT}:${PLUGIN_ROOT:+${PLUGIN_ROOT}/src:}${LEROBOT_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"

session_dir() {
    if [[ -n "${G1_VR_LAB_SESSION_DIR:-}" ]]; then
        printf '%s\n' "${G1_VR_LAB_SESSION_DIR}"
        return
    fi
    if [[ ! -f "${CURRENT_SESSION_FILE}" ]]; then
        echo "No current lab session. Run 01_precheck.sh first." >&2
        return 1
    fi
    sed -n '1p' "${CURRENT_SESSION_FILE}"
}

require_precheck() {
    local directory
    directory="$(session_dir)"
    if [[ ! -f "${directory}/precheck.passed" ]]; then
        echo "Current session has no passing precheck: ${directory}" >&2
        return 1
    fi
}

record_event() {
    local message="$1"
    local directory
    directory="$(session_dir)"
    printf '%s\t%s\n' "$(date --iso-8601=seconds)" "${message}" >> "${directory}/timeline.tsv"
}

voice() {
    local message="$1"
    echo "VOICE: ${message}"
    record_event "voice: ${message}"
    "${TTS_BIN}" -w -r "${VOICE_RATE}" "${message}"
}

voice_async() {
    local message="$1"
    echo "VOICE: ${message}"
    record_event "voice: ${message}"
    "${TTS_BIN}" -r "${VOICE_RATE}" "${message}"
}

validate_label() {
    if [[ ! "$1" =~ ^[a-z0-9][a-z0-9-]*$ ]]; then
        echo "Label must contain only lowercase letters, numbers, and hyphens: $1" >&2
        return 2
    fi
}

wait_for_file() {
    local path="$1"
    local process_id="$2"
    local timeout_s="$3"
    local started=$SECONDS
    while [[ ! -f "${path}" ]]; do
        if ! kill -0 "${process_id}" 2>/dev/null; then
            wait "${process_id}" || true
            return 1
        fi
        if (( SECONDS - started >= timeout_s )); then
            return 1
        fi
        sleep 0.1
    done
}

confirm_headset_handoff() {
    local response
    echo
    echo "Desktop setup is ready and the headset must still be off."
    echo "Type exactly: wearing the headset now"
    echo "Then press Enter and immediately put on the headset."
    read -r response
    if [[ "${response}" != "wearing the headset now" ]]; then
        echo "Handoff cancelled; confirmation did not match." >&2
        return 2
    fi
    record_event "operator confirmed headset handoff"
    echo "You now have ${DONNING_SECONDS} seconds to put on the headset."
    sleep "${DONNING_SECONDS}"
}

confirm_headset_removed() {
    local response
    voice "The run is complete. Release both grips. Put down the controllers. Remove the headset, then return to the terminal."
    echo
    echo "After removing the headset, type exactly: headset off"
    read -r response
    if [[ "${response}" != "headset off" ]]; then
        echo "The run is preserved, but headset removal was not confirmed." >&2
        return 2
    fi
    record_event "operator confirmed headset removed"
}

show_process_log_on_failure() {
    local path="$1"
    echo "Backend failed or did not become ready. Last log lines:" >&2
    tail -n 80 "${path}" >&2 || true
}

xrobotoolkit_service_listeners_active() {
    local listeners
    listeners="$(ss -ltnp 2>/dev/null || ss -ltn 2>/dev/null || true)"
    grep -q ':60061' <<<"${listeners}" && grep -q ':63901' <<<"${listeners}"
}

ensure_xrobotoolkit_pc_service() {
    local log_path="$1"
    local attempt
    local service_unit="xrobotoolkit-pc-service"

    if xrobotoolkit_service_listeners_active; then
        return 0
    fi
    if pgrep -f '[R]oboticsServiceProcess' >/dev/null 2>&1; then
        echo "XRoboToolkit PC Service process exists without both required listeners; refusing to launch a duplicate." >&2
        return 1
    fi
    if [[ -z "${XRBT_PC_SERVICE_DIR}" || ! -x "${XRBT_PC_SERVICE_DIR}/runService.sh" || ! -x "${XRBT_PC_SERVICE_DIR}/RoboticsServiceProcess" ]]; then
        echo "XRoboToolkit PC Service installation was not found; set G1_VR_XRBT_PC_SERVICE_DIR." >&2
        return 1
    fi
    if ! command -v systemd-run >/dev/null 2>&1; then
        echo "systemd-run is required to keep XRoboToolkit PC Service alive across lab scripts." >&2
        return 1
    fi

    echo "Starting XRoboToolkit PC Service from ${XRBT_PC_SERVICE_DIR}"
    if ! systemd-run --user --quiet --collect \
        --unit="${service_unit}" \
        --working-directory="${XRBT_PC_SERVICE_DIR}" \
        --setenv="LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}:${XRBT_PC_SERVICE_DIR}:${XRBT_PC_SERVICE_DIR}/lib:${XRBT_PC_SERVICE_DIR}/SDK/x64" \
        --setenv="QT_PLUGIN_PATH=${XRBT_PC_SERVICE_DIR}/plugins/:${QT_PLUGIN_PATH:-}" \
        --setenv="QT_QML_PATH=${XRBT_PC_SERVICE_DIR}/qml/:${QT_QML_PATH:-}" \
        --property="StandardOutput=append:${log_path}" \
        --property="StandardError=append:${log_path}" \
        "${XRBT_PC_SERVICE_DIR}/RoboticsServiceProcess"; then
        echo "XRoboToolkit PC Service transient unit could not be started." >&2
        return 1
    fi
    for attempt in {1..50}; do
        if xrobotoolkit_service_listeners_active; then
            return 0
        fi
        sleep 0.2
    done
    echo "XRoboToolkit PC Service did not open both ports within 10 seconds." >&2
    return 1
}

current_anchor_set() {
    local directory suite
    directory="$(session_dir)" || return 1
    suite="$(head -1 "${directory}/calibration-suites/current-suite")" || return 1
    if [[ ! -s "${suite}/calibration/anchor-set.json" ]]; then
        echo "Complete the four calibration units in 02 first." >&2
        return 1
    fi
    echo "${suite}/calibration/anchor-set.json"
}
