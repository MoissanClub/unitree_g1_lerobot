#!/usr/bin/env bash
# Verify trace checksums, schema, timing, tracking, grips, and expected motion.

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

usage() {
    echo "usage: $0 (--label LABEL | --trace PATH) [--expect any|idle|right|left|bimanual|buttons-head|reconnect]"
}

label=""
trace_path=""
expectation=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --label) label="${2:?--label requires a value}"; shift ;;
        --trace) trace_path="${2:?--trace requires a path}"; shift ;;
        --expect) expectation="${2:?--expect requires a value}"; shift ;;
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
