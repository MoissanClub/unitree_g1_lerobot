#!/usr/bin/env bash
# Show all accepted static XR calibration anchors in one posture/orientation grid.

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

usage() {
    echo "usage: $0 [--anchor-set PATH] [--output-svg PATH] [--no-show]"
    echo "       defaults to the current suite and opens one 4-row calibration grid"
}

anchor_set=""
output_svg=""
no_show=false
while [[ $# -gt 0 ]]; do
    case "$1" in
        --anchor-set) anchor_set="${2:?--anchor-set requires a path}"; shift ;;
        --output-svg) output_svg="${2:?--output-svg requires a path}"; shift ;;
        --no-show) no_show=true ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

require_precheck
directory="$(session_dir)"
if [[ -z "${anchor_set}" ]]; then
    suite_pointer="${directory}/calibration-suites/current-suite"
    if [[ ! -f "${suite_pointer}" ]]; then
        echo "No current calibration suite. Run 02_record_trace.sh first." >&2
        exit 1
    fi
    suite_dir="$(sed -n '1p' "${suite_pointer}")"
    anchor_set="${suite_dir}/calibration/anchor-set.json"
fi
if [[ ! -f "${anchor_set}" ]]; then
    echo "Calibration anchor set not found: ${anchor_set}" >&2
    exit 1
fi

mkdir -p "${directory}/checks"
if [[ -z "${output_svg}" ]]; then
    output_svg="${directory}/checks/calibration-posture-grid-$(date +%Y%m%d-%H%M%S).svg"
fi
viewer_args=()
if [[ "${no_show}" == true ]]; then
    viewer_args=(--no-show)
fi

record_event "calibration posture grid opened from ${anchor_set}"
"${PYTHON_BIN}" "${REPO_ROOT}/tools/view_xr_calibration_grid.py" \
    "${anchor_set}" --output-svg "${output_svg}" "${viewer_args[@]}"
record_event "calibration posture grid completed; SVG ${output_svg}"
echo "Calibration posture view complete."
echo "Saved view: ${output_svg}"
