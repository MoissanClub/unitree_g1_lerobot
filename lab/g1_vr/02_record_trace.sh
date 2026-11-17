#!/usr/bin/env bash
# Collect, validate, and selectively retry the complete XR calibration suite.

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

usage() {
    cat <<'EOF'
usage: 02_record_trace.sh [options]

Options:
  --dry-run                 Speak the full process without a headset or data writes
  --dry-run-only LABEL      In dry-run mode, practice only one named segment
  --max-attempts NUMBER     Attempts per failed unit/trace in this run (default: 3)
  --new-suite               Start a new suite instead of resuming the current one
  --prepare-s SECONDS       Preparation time before each trace (default: 7)
  --subphase-prepare-s SEC  Settling time within one arm position (default: 2)
  --gate-timeout-s SECONDS  Time allowed for the headset handoff (default: 600)

One headset run collects ten calibration captures across four arm
positions, followed by forward, upward, and outward movements for each arm.
Each calibration position unit is checked after all its palm subphases. A
failed position unit is repeated together up to --max-attempts times. Motion
traces are checked individually. Running this script again resumes the current
suite and skips every position unit or motion trace already passed.

The script prepares one persistent XRoboToolkit connection while the headset is
off, accepts the exact phrase "wearing the headset now", waits ten seconds, and
then uses voice only until it asks the operator to remove the headset.

Dry-run mode does not require a passing precheck, prompt for headset handoff,
connect to XRoboToolkit, create a suite, or make quality decisions.
EOF
}

dry_run=false
dry_run_only=""
max_attempts=3
new_suite=false
prepare_s=7
subphase_prepare_s=2
gate_timeout_s=600
while [[ $# -gt 0 ]]; do
    case "$1" in
        --dry-run) dry_run=true ;;
        --dry-run-only) dry_run_only="${2:?--dry-run-only requires a label}"; shift ;;
        --max-attempts) max_attempts="${2:?--max-attempts requires a value}"; shift ;;
        --new-suite) new_suite=true ;;
        --prepare-s) prepare_s="${2:?--prepare-s requires a value}"; shift ;;
        --subphase-prepare-s) subphase_prepare_s="${2:?--subphase-prepare-s requires a value}"; shift ;;
        --gate-timeout-s) gate_timeout_s="${2:?--gate-timeout-s requires a value}"; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

recorder="${REPO_ROOT}/tools/record_xrobotoolkit_calibration.py"
if [[ "${dry_run}" == true ]]; then
    dry_run_args=(
        "${PYTHON_BIN}" "${recorder}" "${OUTPUT_ROOT}/dry-run-not-written"
        --dry-run
        --prepare-s "${prepare_s}"
        --subphase-prepare-s "${subphase_prepare_s}"
        --tts-bin "${TTS_BIN}"
        --voice-rate "${VOICE_RATE}"
    )
    if [[ -n "${dry_run_only}" ]]; then
        dry_run_args+=(--dry-run-only "${dry_run_only}")
    fi
    exec "${dry_run_args[@]}"
fi
if [[ -n "${dry_run_only}" ]]; then
    echo "--dry-run-only requires --dry-run" >&2
    exit 2
fi

require_precheck
directory="$(session_dir)"
suite_root="${directory}/calibration-suites"
current_suite_file="${suite_root}/current-suite"
mkdir -p "${suite_root}"

if [[ "${new_suite}" == true || ! -f "${current_suite_file}" ]]; then
    suite_dir="${suite_root}/calibration-$(date +%Y%m%d-%H%M%S)"
    mkdir -p "${suite_dir}"
    printf '%s\n' "${suite_dir}" > "${current_suite_file}"
else
    suite_dir="$(sed -n '1p' "${current_suite_file}")"
fi

pending="$(${PYTHON_BIN} "${recorder}" "${suite_dir}" --list-pending)"
if [[ -z "${pending}" ]]; then
    echo "Calibration suite is already complete: ${suite_dir}"
    echo "Use --new-suite only if you deliberately want to collect it again."
    exit 0
fi

echo "Calibration suite: ${suite_dir}"
echo "Traces needed in this run:"
printf '%s\n' "${pending}" | sed 's/^/  - /'
echo "Maximum attempts for each needed trace: ${max_attempts}"

run_dir="${suite_dir}/runs/$(date +%Y%m%d-%H%M%S)"
ready_file="${run_dir}/backend.ready"
start_gate="${run_dir}/operator.start"
backend_log="${run_dir}/backend.log"
mkdir -p "${run_dir}"

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

record_event "calibration suite: preparing persistent XRoboToolkit recorder"
"${PYTHON_BIN}" "${recorder}" "${suite_dir}" \
    --max-attempts "${max_attempts}" \
    --prepare-s "${prepare_s}" \
    --subphase-prepare-s "${subphase_prepare_s}" \
    --rate-hz 60 \
    --sdk-site-packages "${XRBT_SDK_SITE_PACKAGES}" \
    --adapter-revision "${adapter_revision}" \
    --ready-file "${ready_file}" \
    --start-gate-file "${start_gate}" \
    --gate-timeout-s "${gate_timeout_s}" \
    --tts-bin "${TTS_BIN}" \
    --voice-rate "${VOICE_RATE}" \
    --timeline "${directory}/timeline.tsv" \
    > >(tee -a "${backend_log}") 2>&1 &
backend_pid=$!

if ! wait_for_file "${ready_file}" "${backend_pid}" 90; then
    show_process_log_on_failure "${backend_log}"
    exit 1
fi
echo "Recorder backend is connected and will stay connected for the full suite."
confirm_headset_handoff
printf '%s\n' "$(date --iso-8601=seconds)" > "${start_gate}"
record_event "calibration suite: headset gate released"

set +e
wait "${backend_pid}"
backend_status=$?
set -e
backend_pid=""

confirm_headset_removed || true
trap - EXIT INT TERM

echo "Calibration suite manifest: ${suite_dir}/manifest.json"
remaining="$(${PYTHON_BIN} "${recorder}" "${suite_dir}" --list-pending)"
if [[ -z "${remaining}" ]]; then
    record_event "calibration suite: all traces passed immediate quality checks"
    echo "PASS: all calibration traces passed. No redo is needed."
    exit 0
fi

record_event "calibration suite: incomplete; failed traces retained for selective retry"
echo "INCOMPLETE: run script 2 again to retry only these traces:"
printf '%s\n' "${remaining}" | sed 's/^/  - /'
if (( backend_status != 0 && backend_status != 3 )); then
    show_process_log_on_failure "${backend_log}"
    exit "${backend_status}"
fi
exit 3
