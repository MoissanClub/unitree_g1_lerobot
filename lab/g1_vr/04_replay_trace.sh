#!/usr/bin/env bash
# Replay a validated trace through the exact G1-29 native MuJoCo simulation.

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

usage() {
    echo "usage: $0 (--label LABEL | --trace PATH) [--expect EXPECTATION] [--headless] [--rate-hz HZ]"
}

label=""
trace_path=""
expectation="any"
headless=false
rate_hz=60
while [[ $# -gt 0 ]]; do
    case "$1" in
        --label) label="${2:?--label requires a value}"; shift ;;
        --trace) trace_path="${2:?--trace requires a path}"; shift ;;
        --expect) expectation="${2:?--expect requires a value}"; shift ;;
        --headless) headless=true ;;
        --rate-hz) rate_hz="${2:?--rate-hz requires a value}"; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

require_precheck
directory="$(session_dir)"
if [[ -n "${label}" && -n "${trace_path}" ]] || [[ -z "${label}" && -z "${trace_path}" ]]; then
    usage >&2
    exit 2
fi
if [[ -n "${label}" ]]; then
    validate_label "${label}"
    trace_path="${directory}/traces/${label}"
else
    label="$(basename "${trace_path}")"
    validate_label "${label}"
fi

"${LAB_DIR}/03_check_trace.sh" --trace "${trace_path}" --expect "${expectation}"
telemetry="${directory}/telemetry/replay-${label}-$(date +%Y%m%d-%H%M%S).json"
log="${telemetry%.json}.log"
mkdir -p "${directory}/telemetry"
viewer_args=()
if [[ "${headless}" == false ]]; then
    viewer_args=(--viewer)
fi

record_event "trace ${label}: native MuJoCo replay started"
"${PYTHON_BIN}" "${REPO_ROOT}/tools/run_supervised_g1_29_vr.py" \
    --source trace \
    --trace "${trace_path}" \
    --urdf "${G1_URDF}" \
    --mesh-dir "${G1_MESH_DIR}" \
    --rate-hz "${rate_hz}" \
    --telemetry "${telemetry}" \
    "${viewer_args[@]}" \
    2>&1 | tee "${log}"
record_event "trace ${label}: native MuJoCo replay completed"
echo "Replay complete."
echo "Telemetry: ${telemetry}"
echo "Log: ${log}"
