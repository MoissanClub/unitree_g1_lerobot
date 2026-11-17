#!/usr/bin/env bash
# Record one labeled XRoboToolkit trace with a gated headset handoff.

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

usage() {
    cat <<'EOF'
usage: 02_record_trace.sh --label LABEL [options]

Options:
  --duration-s SECONDS       Recording duration (default: 10)
  --instruction TEXT        Spoken movement instruction
  --gate-timeout-s SECONDS  Time allowed for the handoff (default: 600)

The script prepares the backend with the headset off, accepts the exact phrase
"wearing the headset now", waits ten seconds, and then uses voice only until it
asks the operator to remove the headset.
EOF
}

label=""
duration_s=10
instruction="Move as planned for this trace."
gate_timeout_s=600
while [[ $# -gt 0 ]]; do
    case "$1" in
        --label) label="${2:?--label requires a value}"; shift ;;
        --duration-s) duration_s="${2:?--duration-s requires a value}"; shift ;;
        --instruction) instruction="${2:?--instruction requires text}"; shift ;;
        --gate-timeout-s) gate_timeout_s="${2:?--gate-timeout-s requires a value}"; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

[[ -n "${label}" ]] || { usage >&2; exit 2; }
validate_label "${label}"
require_precheck
directory="$(session_dir)"
trace_dir="${directory}/traces/${label}"
run_dir="${directory}/recordings/${label}-$(date +%Y%m%d-%H%M%S)"
ready_file="${run_dir}/backend.ready"
start_gate="${run_dir}/operator.start"
backend_log="${run_dir}/backend.log"
mkdir -p "${run_dir}" "${directory}/traces"
if [[ -e "${trace_dir}" ]]; then
    echo "Trace already exists and will not be overwritten: ${trace_dir}" >&2
    exit 2
fi

adapter_revision="$(git -C "${PLUGIN_ROOT}" rev-parse HEAD)"
backend_pid=""
cleanup() {
    if [[ -n "${backend_pid}" ]] && kill -0 "${backend_pid}" 2>/dev/null; then
        kill -TERM "${backend_pid}" 2>/dev/null || true
        wait "${backend_pid}" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

record_event "trace ${label}: preparing XRoboToolkit recorder"
"${PYTHON_BIN}" "${REPO_ROOT}/tools/record_xrobotoolkit_trace.py" \
    "${trace_dir}" \
    --label "${label}" \
    --duration-s "${duration_s}" \
    --rate-hz 60 \
    --sdk-site-packages "${XRBT_SDK_SITE_PACKAGES}" \
    --adapter-revision "${adapter_revision}" \
    --ready-file "${ready_file}" \
    --start-gate-file "${start_gate}" \
    --gate-timeout-s "${gate_timeout_s}" \
    > "${backend_log}" 2>&1 &
backend_pid=$!

if ! wait_for_file "${ready_file}" "${backend_pid}" 90; then
    show_process_log_on_failure "${backend_log}"
    exit 1
fi
echo "Recorder backend is connected and waiting. No samples are being recorded yet."
confirm_headset_handoff
voice "Recording starts now. ${instruction} Continue until I say recording complete."
printf '%s\n' "$(date --iso-8601=seconds)" > "${start_gate}"
record_event "trace ${label}: recording gate released"

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

record_event "trace ${label}: capture completed"
confirm_headset_removed
trap - EXIT INT TERM
echo "Trace recorded: ${trace_dir}"
echo "Next: ${LAB_DIR}/03_check_trace.sh --label ${label}"
