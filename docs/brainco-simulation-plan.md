# BrainCo Simulation and Handshake Replay

## Goal and Upstream Boundary

Replay `davidwei79/g1-handshake-data` with articulated BrainCo hands in MuJoCo,
using LeRobot's Robot/dataset interfaces. Maximize upstream-compatible work and
keep each review small. This is a simulation milestone, not physical acceptance.

Follow the end-effector/Hub ownership introduced by LeRobot commit
`77de23890bec5fb3887c5ba8404d6e51be7bfb72` and the shared sim/real direction in
[roadmap #3832](https://github.com/huggingface/lerobot/issues/3832).
Do not resurrect a parallel native G1 simulator, embed dataset decoding in the
driver, require XR for replay, or introduce a speculative generic hand hierarchy.

## Steps and Status (2026-09-24)

| Step | Deliverable | Status / verification gate |
| --- | --- | --- |
| 1 | Minimal ownership and command/feedback contract | Implemented as draft v1; upstream maintainer agreement still pending |
| 2 | Licensed articulated BrainCo model in the existing Hub simulator | G1-29 model compiles; independent motor, coupling, collision-limited closure and headless render tests pass |
| 3 | LeRobot selection, feature dispatch and measured feedback | Local implementation and real headless Robot API test pass; experimental Hub snapshot published; bounded viewer option added |
| 4 | Audit dataset schema and add explicit replay mapping | Next: verify embodiment, hand version, field order, units, timestamps and action/state meaning; do not infer equivalence from field names |
| 5 | Time-aligned dataset replay using standard LeRobot interfaces | Pending: deterministic replay, body/hand tracking metrics, limits and missing-data handling |
| 6 | Visual acceptance and upstream submissions | Pending: user inspection of complete replay, calibrated wrist mount and small model/core/replay PRs |

Work stopped at Step 3 as requested. No dataset episode has been replayed yet.
No physical hand, headset, G1-23 BrainCo model, calibrated contact dynamics or
real-hardware validation is implied by these results.

## Implementation Locations

Work is isolated from the user's `~/lerobot-dev/lerobot` checkout and its conda
editable install. That checkout was left on `g1/simulation`, including its
pre-existing `MUJOCO_LOG.TXT`.

| Repository | Local folder | Working branch | Base |
| --- | --- | --- | --- |
| LeRobot fork | `/home/dwei/lerobot-sim/brainco-development/lerobot` | `work/brainco-simulation-replay` | `e624f3f7f8411ec3a02635d06e79373341e5ef35` (fork main) |
| Hub simulator | `/home/dwei/lerobot-sim/brainco-development/unitree-g1-mujoco` | `work/brainco-simulation` | `68459ed68f6f68e1f661091dfcb6ebce44681aec` |
| BrainCo asset reference | `/home/dwei/lerobot-sim/brainco-development/revo2_description` | pinned source checkout | `92cc697c7fa691db59404ce52344f3969a5ef7a6` |

LeRobot changes remain local working changes, not a merged integration release.
The Hub simulator is published as an experimental fork (see below), not upstream.
`dev/g1-integration` remains the product integration branch; this focused
work starts on current fork main to avoid inheriting the older custom hand stack.
Integrating it into the product branch is a separate merge/reconciliation step.

## Contract

`UnitreeG1Config(end_effector="brainco", is_simulation=True)` advertises twelve
`hands.{left,right}.motor_{0..5}.pos` fields alongside body/controller features.
They are **model-normalized** fractions, not a verified dataset or SDK mapping.
The Hub owns the six-motor-to-eleven-joint mapping for each hand. Body commands
continue over DDS; hands use the existing in-process environment with a versioned
interface. No BrainCo serial SDK is imported and no serial device is opened.
Physical BrainCo selection is rejected. Default `dex1` behavior is preserved.

The Hub's `BRAINCO.md` and `reference/brainco/README.md` define motor order,
source licensing, invalid-command handling, held targets, stale feedback,
simulation gains and the provisional wrist mounting transform. Core adds
`sim_hub_path` so an eventual published Hub fork/revision can be explicitly pinned
without exposing a recursively parsed environment config.

## Reproduce Locally

Use the existing conda `lerobot-dev` dependencies without reinstalling its editable
LeRobot. Set the source path explicitly so the test cannot use the wrong checkout.
The live test also asserts its imported Robot class comes from this source tree.

```bash
source ~/lerobot-dev/environment/activate.sh
export PYTHON=/home/dwei/miniforge3/envs/lerobot-dev/bin/python
export WORK=/home/dwei/lerobot-sim/brainco-development
export PYTHONPATH="$WORK/lerobot/src"
export HF_HOME="$WORK/cache/huggingface"
export HF_LEROBOT_HOME="$WORK/cache/lerobot"
export MUJOCO_GL=egl
cd "$WORK/lerobot"
"$PYTHON" -m pytest tests/robots/test_unitree_g1.py \
  tests/robots/test_unitree_g1_brainco.py \
  tests/robots/test_unitree_g1_utils.py tests/robots/test_sonic_whole_body.py \
  tests/envs/test_dispatch.py -q
G1_BRAINCO_HUB_CHECKOUT="$WORK/unitree-g1-mujoco" \
  "$PYTHON" -m pytest tests/robots/test_unitree_g1_brainco_sim.py -v
cd "$WORK/unitree-g1-mujoco"
"$PYTHON" build_brainco_model.py
BRAINCO_RENDER_DIR="$WORK/results" "$PYTHON" -m unittest discover -s tests -v
"$PYTHON" tests/smoke_live.py --end-effector brainco
```

Run live DDS checks sequentially in a dedicated simulation session. The activation
file selects the workspace's pinned CycloneDDS native library. Omitting it loaded
a different native library during the first attempt and the process failed to exit;
this is not evidence that every CycloneDDS installation has been validated.

The acceptance test redirects **only the download resolver** to the local Hub
checkout. It does not mock Robot construction, factory/import, MuJoCo, DDS body
commands, hand commands or feedback. Published mode instead uses the normal
Hub downloader with no resolver patch. It repeats in isolated processes
to exercise startup/control/shutdown, with a finite timeout.

## Reviewer Verification

The existing acceptance file now supports both a bounded viewer and an immutable
remote revision; no new teleoperation script or framework was added.

Published simulator:
[davidwei79/unitree-g1-mujoco-brainco](https://huggingface.co/davidwei79/unitree-g1-mujoco-brainco/tree/99806d8c7e9b8dc4c1cd500450c3f563a52a8953),
commit `99806d8c7e9b8dc4c1cd500450c3f563a52a8953`.
This is an experimental model/code snapshot, not physical acceptance or upstream
approval. The matching LeRobot source changes are still required.

After the terminal setup above:

```bash
cd "$WORK/lerobot"
unset G1_BRAINCO_HUB_CHECKOUT
export G1_BRAINCO_HUB_PATH=davidwei79/unitree-g1-mujoco-brainco@99806d8c7e9b8dc4c1cd500450c3f563a52a8953
"$PYTHON" -m pytest tests/robots/test_unitree_g1_brainco_sim.py -v
"$PYTHON" tests/robots/test_unitree_g1_brainco_sim.py \
  --viewer
```

The default downloads a pinned HF simulator, with no dataset or local model path.
Each cycle exercises all six controls on each hand individually, reopening each
before the next moves. It takes approximately 38 seconds without commanded arm
movement. Numerical checks still run. `--motion combined` retains the arm/hand
regression sequence used by the opt-in subprocess tests.

BrainCo-specific mocked unit tests now live in
`tests/robots/test_unitree_g1_brainco.py`; `unitree_g1_test_utils.py` shares their
fixtures with the generic G1 tests. These unit tests need no HF download.
The viewer option selects GLFW and runs for a bounded duration (one to three
cycles). Window closure/Ctrl+C cancels and does not report a pass.

Native viewer execution requires a working OpenGL display. `ssh -Y` alone does
not guarantee GLX support; use `--headless` if unavailable. GUI execution remains
a user verification step; development verification stays headless.

The published-source test uses an initially empty Hugging Face cache and the
normal factory/downloader. It exposed and fixed a Hub entry-point bug: missing
assets must hydrate from the selected **fork, commit and cache directory**, not
from a hardcoded upstream repository. Two new Hub tests cover that and rejection
of incomplete local checkouts. Remote acceptance allows 180 seconds per process
for cold downloads; local mode allows 60 seconds. Neither changes the bounded
motion duration.

## Findings and Next Gate

Final local verification on 2026-09-24 (MuJoCo 3.12.0, Python 3.12.14):

| Check | Result |
| --- | --- |
| Targeted LeRobot robot/controller/env-dispatch regressions | 131 passed |
| Local-checkout acceptance and CLI checks (two real subprocesses) | 6 passed |
| Published final-pin acceptance and CLI checks (two real subprocesses; first starts with an empty cache) | 6 passed in 54.03 seconds |
| Hub physics/coupling/independent motor/reset/validation/render, fork hydration and previous-model regression tests | 12 passed |
| Real Hub DDS body and ZMQ camera smoke tests | Passed separately for brainco, dummy, dex1 and dex3 |
| Regenerating the two BrainCo MJCF files | Byte-identical in the tested MuJoCo version |
| Vendored URDFs and meshes vs pinned BrainCo checkout | Identical; license retained |
| LeRobot changed Python files and new Hub Python files | Ruff checks passed; git diff whitespace checks passed |

Final published-pin evidence: `$WORK/results/published-hub-final.xml`.
Initial Step 3 baseline: `$WORK/results/lerobot.xml`. Visual evidence:
`$WORK/results/brainco-open-closed.png` (top: open, bottom: closing; left/right
hands in columns). The render was inspected headlessly; no manual X/headset
session was run.

The tested final published pin is
`8a9ca7c0aeb114fda5cbf8b0d4846da7ce377835`; its first acceptance subprocess
downloaded into the initially empty `$WORK/cache/final-pin-XdZboa` cache. This
verifies remote simulator retrieval with the matching local LeRobot source and
existing conda dependencies, not a fresh conda installation.

- Source follower limits conflict with some master upper limits. The generator
  derives feasible normalized ranges; real SDK endpoint calibration is pending.
- Full closure causes thumb/index contact, so measured hand state correctly
  differs from requested targets. No-contact and contact tests are separate.
- Live gravity-on hand tracking error was below 0.004 normalized units in the
  initial three-phase test. Arm commands also reached the simulated body.
- Headless shutdown exposed a pre-existing `image_publish_process=None`
  dereference in LeRobot. The small guard fix has its own regression test and
  can be submitted independently of the BrainCo feature.
- The wrist adapter uses a provisional 41.5 mm offset and axis transform. It is
  visually inspectable but not measured; resolve it before asserting Cartesian
  or handshake replay fidelity.
- Step 4 must compare the actual dataset's thumb channels and normalization
  against the pinned model. Existing `xr_teleoperate` conventions and newer
  REVO2 source conventions must not be silently treated as identical.

### HF-Default Verification Update

The current test default is simulator revision `99806d8c7e9b8dc4c1cd500450c3f563a52a8953`.
It includes a MuJoCo 3.12 passive-viewer teardown workaround: wait for native
cleanup after requesting close. The original exit crash also reproduced with a
minimal MuJoCo scene, independently of BrainCo and DDS. The workaround isolates
a private viewer-handle compatibility check and has bounded timeout tests.
Earlier results above retain their original model pins for traceability.

Verification after the split and HF-default change:
- G1, BrainCo, utilities, and HF-backed acceptance: 94 tests passed, including
  two independent combined arm/hand subprocess runs.
- Default finger sequence: passed after downloading into an empty cache;
  all 25 phases passed with maximum normalized error below 0.004.
- The same HF revision with `--viewer` under virtual X: all 25 phases passed,
  process exit code 0, no native shutdown crash. Human visual review is separate.
- Ruff and diff whitespace checks passed for the changed test files.
