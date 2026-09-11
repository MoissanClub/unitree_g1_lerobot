# Live Control Acceptance

## Run

Stop existing simulator and bridge sessions, then run:

```bash
./run_verify_live_control.sh
# Or one embodiment:
./run_verify_live_control.sh --embodiment g1_23
```

This launches the real standalone simulator plus a separate LeRobot DDS command process,
one embodiment at a time. There is no XR, viewer, headset, or physical robot connection.
Transport is loopback, simulation identity is checked, and an existing simulator lock
causes refusal. Do not start another simulator or command sender during the matrix.
Each session uses fresh processes to contain the known DDS callback teardown problem.
The launcher uses the existing `lerobot-g1` environment and cached models (offline mode).

Logs, raw JSON samples, and a manifest go to a new timestamped `outputs/live-control-*`
directory in the repository. `--output PATH` selects a new directory; an existing path is
rejected to preserve evidence. These generated full reports are ignored by Git but persist
locally. Compact reviewed summaries can be archived under `docs/verification/`.

## Cases and Evidence

- Every active arm joint receives +0.12 and -0.12 rad steps about a fixed supported-arm pose,
  returning to the baseline between steps. There are 28 G1-29 and 20 G1-23 joint cases.
- Eighteen target-space sweeps exercise x/y/z translation (+/-15 mm) and local hand
  orientation (+/-0.08 rad around each axis), separately for left, right, and both hands.
  The opposite arm's joint command is frozen for single-arm cases. Targets return to the
  origin and reverse direction during each approximately three-second cycle.
- Normal runtime gains and gravity compensation are used. Commands are checked against
  IK-model joint limits before publication. This check does not establish simulator limit
  clipping or full-workspace acceptance.
- A separate DDS subscriber observes raw transport slots and checks feedback freshness.
  Unused command and state slots must remain zero. Named observations still use the
  runtime mapping; this is not an independent geometric calibration of that mapping.
- Reports separate target-to-commanded-FK (IK error), commanded-to-measured-FK (actuator
  error), and target-to-measured-FK (total error), with translation and rotation separate.
  FK uses the same Pinocchio model as IK; it is not independent MuJoCo end-effector truth.
  Add `--geometry` for the separate [exact-state MuJoCo comparison](independent-geometry.md).
  It uses the common pelvis frame and actual received waist state. The initial 10 mm
  torso-only finding is qualified by the [full-chain audit](g1-model-source-audit.md).
  Existing Pinocchio-only control metrics are preserved.
- Reports include joint names/slots/limits, gains, raw DDS q, targets/commands/measured
  poses, feedback age, loop duration including IK, revisions, and diagnostic source hash.

## Smoke Acceptance Budgets

These initial budgets were stated before the first run and are regression smoke guards,
not hardware specifications or final task tolerances. They are stored with every report.

| Metric | Budget |
|---|---:|
| Settled maximum joint error, median of last 10 samples | 0.04 rad |
| Other-joint displacement from baseline | 0.04 rad |
| Signed driven-joint response | At least 0.06 rad |
| Worst per-joint moving p95 error | 0.08 rad |
| Worst per-hand p95 actuator position / rotation error | 35 mm / 0.20 rad |
| Worst per-hand p95 IK position / rotation error | 40 mm / 0.50 rad |
| Commanded and measured span on each requested arm | Greater than 0.005 rad |
| Independent DDS feedback age | At most 250 ms |

All cases must pass; failures remain in reports and produce a nonzero process exit.
Total pose error is reported, not gated against an unstated task tolerance. Per-hand and
per-joint p95 metrics are computed separately before taking the worst value, so an idle
arm cannot dilute error. Broad IK budgets accommodate G1-23's five-joint pose compromise;
inspect the actual measurements rather than treating a PASS as precise six-DOF tracking.

## Verified Checkpoint: 2026-09-10

The final matrix passed: **G1-29 46/46 cases; G1-23 38/38 cases**. The difference is
joint count, not failures: 14 versus 10 arm joints, each tested in two directions,
plus the same 18 Cartesian/orientation cases. Both simulator exit codes were zero.
Full reports: `outputs/live-control-20260910-154756/`. Durable case-level evidence:
[numerical summary](verification/live-control-20260910.json).

| Worst case across the tested cases | G1-29 | G1-23 |
|---|---:|---:|
| Settled maximum joint error | 0.00385 rad | 0.00087 rad |
| Other-joint displacement | 0.00239 rad | 0.00069 rad |
| Per-joint moving p95 error | 0.01567 rad | 0.01371 rad |
| Per-hand p95 actuator position error | 4.25 mm | 2.55 mm |
| Per-hand p95 IK position error | 9.63 mm | 7.85 mm |
| Per-hand p95 IK orientation error | 0.0235 rad | 0.0817 rad |
| Per-hand p95 total position error | 10.55 mm | 8.46 mm |

These are worst per-case p95 values, except the two settled-step metrics. G1-23's
larger orientation residual illustrates why IK compromise must remain separate from
actuator tracking. The different models/gains are not a controlled gain A/B experiment;
these results do not establish that one embodiment or motor profile is universally better.

## Remaining Coverage

This is the first automated live-path acceptance checkpoint, not completion of the
[non-physical backlog](future-non-physical-work.md). Remaining extensions include
near-limit/workspace-boundary trajectories, larger bilateral sweeps and holds/reversals,
explicit timeout/command-loss fault injection, headset/service recovery, video off/on
timing comparisons, sustained runs, and native DDS/OpenXR cleanup investigations.

Independent simulator geometry is already implemented and passing for both embodiments.
After migration `bbc08f4`, extend `diagnostics/shared/motion_cases.py` and
`shared/acceptance_metrics.py`, keeping this baseline intact. The [end-of-day backlog](future-non-physical-work.md)
defines the next task and acceptance boundaries; do not loosen budgets to conceal IK limits.
