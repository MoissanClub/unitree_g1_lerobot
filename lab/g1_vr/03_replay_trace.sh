#!/usr/bin/env bash
# Validate a trace, then replay it through the exact G1-29 native MuJoCo simulation.

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

usage() {
    echo "usage: $0 (--label LABEL | --trace PATH) [--expect EXPECTATION] [--validate-only] [--headless] [--rate-hz HZ]"
}

label=""
trace_path=""
expectation=""
validate_only=false
headless=false
rate_hz=60
while [[ $# -gt 0 ]]; do
    case "$1" in
        --label) label="${2:?--label requires a value}"; shift ;;
        --trace) trace_path="${2:?--trace requires a path}"; shift ;;
        --expect) expectation="${2:?--expect requires a value}"; shift ;;
        --validate-only) validate_only=true ;;
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

if [[ -z "${expectation}" ]]; then
    case "${label}" in
        *idle*) expectation="idle" ;;
        *right*) expectation="right" ;;
        *left*) expectation="left" ;;
        *bimanual*|*both*) expectation="bimanual" ;;
        *button*|*head*) expectation="buttons-head" ;;
        *reconnect*|*tracking-loss*) expectation="reconnect" ;;
        *) expectation="any" ;;
    esac
fi

report="${directory}/checks/${label}.json"
mkdir -p "${directory}/checks"
record_event "trace ${label}: validation started with expectation ${expectation}"
if "${PYTHON_BIN}" "${REPO_ROOT}/tools/check_g1_vr_trace.py" \
    "${trace_path}" --expect "${expectation}" --output "${report}"; then
    record_event "trace ${label}: validation passed"
    echo "Validation report: ${report}"
else
    record_event "trace ${label}: validation failed"
    echo "Validation failed; evidence was preserved at ${report}" >&2
    exit 1
fi
if [[ "${validate_only}" == true ]]; then
    echo "Validation-only run complete."
    exit 0
fi

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
