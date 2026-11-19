#!/usr/bin/env bash
# Validate a trace, then replay it through the exact G1-29 native MuJoCo simulation.

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

usage() {
    echo "usage: $0 [--label LABEL | --trace PATH] [--expect EXPECTATION] [--validate-only] [--headless] [--rate-hz HZ]"
    echo "       no trace option validates and replays every accepted trace in the current suite"
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
if [[ -n "${label}" && -n "${trace_path}" ]]; then
    usage >&2
    exit 2
fi

labels=()
trace_paths=()
if [[ -n "${trace_path}" ]]; then
    label="$(basename "${trace_path}")"
    validate_label "${label}"
    labels+=("${label}")
    trace_paths+=("${trace_path}")
else
    suite_pointer="${directory}/calibration-suites/current-suite"
    if [[ ! -f "${suite_pointer}" ]]; then
        echo "No current calibration suite. Run 02_record_trace.sh first." >&2
        exit 1
    fi
    suite_dir="$(sed -n '1p' "${suite_pointer}")"
    manifest="${suite_dir}/manifest.json"
    if [[ ! -f "${manifest}" ]]; then
        echo "Current calibration suite has no manifest: ${manifest}" >&2
        exit 1
    fi

    if [[ -n "${label}" ]]; then
        validate_label "${label}"
        selected_trace="$("${PYTHON_BIN}" - "${manifest}" "${label}" <<'PY'
import json
import sys

manifest = json.load(open(sys.argv[1]))
entry = manifest.get("segments", {}).get(sys.argv[2], {})
print(entry.get("selected_trace") or "")
PY
)"
        if [[ -z "${selected_trace}" ]]; then
            echo "The current suite has no accepted ${label} trace. Complete script 2 first." >&2
            exit 1
        fi
        labels+=("${label}")
        trace_paths+=("${selected_trace}")
    else
        if [[ -n "${expectation}" ]]; then
            echo "--expect requires --label or --trace when selecting one trace" >&2
            exit 2
        fi
        suite_status="$("${PYTHON_BIN}" - "${manifest}" <<'PY'
import json
import sys

print(json.load(open(sys.argv[1])).get("status", "unknown"))
PY
)"
        if [[ "${suite_status}" != "complete" ]]; then
            echo "Current calibration suite is ${suite_status}; complete script 2 before replaying all traces." >&2
            exit 1
        fi
        while IFS=$'\t' read -r selected_label selected_trace; do
            [[ -n "${selected_label}" && -n "${selected_trace}" ]] || continue
            validate_label "${selected_label}"
            labels+=("${selected_label}")
            trace_paths+=("${selected_trace}")
        done < <("${PYTHON_BIN}" - "${manifest}" <<'PY'
import json
import sys

manifest = json.load(open(sys.argv[1]))
for label, entry in manifest.get("segments", {}).items():
    trace = entry.get("selected_trace")
    if entry.get("status") == "passed" and trace:
        print(f"{label}\t{trace}")
PY
)
        if [[ ${#labels[@]} -eq 0 ]]; then
            echo "Current calibration suite contains no accepted traces." >&2
            exit 1
        fi
        echo "Selected ${#labels[@]} accepted traces from the current suite."
    fi
fi

infer_expectation() {
    case "$1" in
        calibration-*|*idle*) echo "idle" ;;
        right-*|*right*) echo "right" ;;
        left-*|*left*) echo "left" ;;
        *bimanual*|*both*) echo "bimanual" ;;
        *button*|*head*) echo "buttons-head" ;;
        *reconnect*|*tracking-loss*) echo "reconnect" ;;
        *) echo "any" ;;
    esac
}

viewer_args=()
if [[ "${headless}" == false ]]; then
    viewer_args=(--viewer)
fi
mkdir -p "${directory}/checks"
if [[ "${validate_only}" == false ]]; then
    mkdir -p "${directory}/telemetry"
fi

for index in "${!labels[@]}"; do
    current_label="${labels[index]}"
    current_trace="${trace_paths[index]}"
    current_expectation="${expectation:-$(infer_expectation "${current_label}")}"
    report="${directory}/checks/${current_label}.json"
    echo "Trace $((index + 1))/${#labels[@]}: ${current_label}"
    record_event "trace ${current_label}: validation started with expectation ${current_expectation}"
    if "${PYTHON_BIN}" "${REPO_ROOT}/tools/check_g1_vr_trace.py" \
        "${current_trace}" --expect "${current_expectation}" --output "${report}"; then
        record_event "trace ${current_label}: validation passed"
        echo "Validation report: ${report}"
    else
        record_event "trace ${current_label}: validation failed"
        echo "Validation failed; evidence was preserved at ${report}" >&2
        exit 1
    fi
    if [[ "${validate_only}" == true ]]; then
        continue
    fi

    telemetry="${directory}/telemetry/replay-${current_label}-$(date +%Y%m%d-%H%M%S).json"
    log="${telemetry%.json}.log"
    record_event "trace ${current_label}: native MuJoCo replay started"
    "${PYTHON_BIN}" "${REPO_ROOT}/tools/run_supervised_g1_29_vr.py" \
        --source trace \
        --trace "${current_trace}" \
        --urdf "${G1_URDF}" \
        --mesh-dir "${G1_MESH_DIR}" \
        --rate-hz "${rate_hz}" \
        --telemetry "${telemetry}" \
        "${viewer_args[@]}" \
        2>&1 | tee "${log}"
    record_event "trace ${current_label}: native MuJoCo replay completed"
    echo "Replay complete: ${current_label}"
    echo "Telemetry: ${telemetry}"
    echo "Log: ${log}"
done

if [[ "${validate_only}" == true ]]; then
    echo "Validation-only run complete for ${#labels[@]} trace(s)."
else
    echo "Validation and replay complete for ${#labels[@]} trace(s)."
fi
