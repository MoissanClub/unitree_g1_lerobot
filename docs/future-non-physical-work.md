# Future Non-Physical Work

## End-of-Day Checkpoint: 2026-09-10

Work is concluded for today. Resume with item 1 below; do not restart camera bring-up,
the organization migration, or the completed small-signal acceptance work.

Completed baseline:
- Configurable G1-29/G1-23 simulation, embodiment mapping/IK/motor settings, gravity
  compensation, DDS feedback, and bilateral XR control.
- User-reviewed dual-arm headset control and basic robot-camera video. The video review
  did not identify an embodiment or establish exhaustive reconnect/latency acceptance.
- Live small-signal joint and Cartesian/orientation acceptance: G1-29 46/46 cases;
  G1-23 38/38 cases. Independent publish-time geometry checks pass for both embodiments.
- Model-source audit: neutral-waist arm geometry agrees in pelvis coordinates; legacy
  and rev_1_0 waist layouts differ under roll/pitch. Do not shift the shoulders.
- Diagnostic/test organization migrated in `bbc08f4`; source audit/geometry work in
  `e0b95bd`. The migration passed the 72-test regression run, separate environment-gated
  checks, both real headless XR/video matrices, and all seven Tk/EGL comparison launchers.

See [migration evidence](verification-organization.md#migration-verification-2026-09-10),
[live acceptance](live-control-acceptance.md), and [model audit](g1-model-source-audit.md).
Keep physical robot actuation out of this backlog.

## Remaining Work, In Order

1. **Broader motion acceptance, both embodiments.** Preserve the existing baseline and
   add larger Cartesian sweeps, conservative joint-limit approaches, holds, reversals,
   single-arm/bilateral motion, and orientation changes. Separate known-reachable target
   acceptance from workspace characterization. Report target, commanded, and measured
   poses; distinguish IK residual, actuator error, inactive-arm drift, and limit margins.
   Retain independent pelvis-frame geometry checks. Establish case-specific budgets
   before acceptance runs; do not relax thresholds merely to make G1-23 pass. Its five-DoF
   arms cannot generally satisfy arbitrary six-DoF hand poses. Simulation workspace
   tests do not establish collision safety, maximum speed, payload rating, or hardware limits.
2. **Failure and recovery.** Test independent clutch engagement/release, invalid/lost
   tracking, stale commands, headset disconnect/reconnect, and simulator/camera/CloudXR
   restarts. Define and verify hold/release behavior, recovery time, and absence of stale
   replay or unexpected motion. Camera loss should show a placeholder without blocking
   control. Preserve one command publisher and simulation-only safeguards. Automate
   fault injection first; actual headset reconnect/presentation still needs observation.
3. **Performance and endurance.** Compare identical trajectories with video off/on.
   Measure physics real-time factor, control-loop timing/jitter, tracking error, video
   frame rate/drops/age, and CPU/GPU/memory usage. Set session durations and thresholds
   explicitly, then test for resource growth, stalls, and degraded tracking. Local frame
   age is not end-to-end headset latency; measure headset latency separately.
4. **Native lifecycle cleanup.** Build a minimal Unitree-SDK-only reproducer for the DDS
   callback teardown crash; compare explicit cleanup before assigning responsibility to
   CycloneDDS, the SDK, or LeRobot. Fresh-process tests remain containment, not a fix.
   Investigate `XR_ERROR_SESSION_NOT_STOPPING` separately. Clean process exit does not
   prove correct OpenXR lifecycle handling. Bring this work forward if it blocks item 2.
5. **Matched model families, conditional on waist control.** Before articulated-waist
   experiments, select a matched G1-29 URDF/MJCF family and reverify full-chain FK,
   gravity/inertias, limits, meshes, and control. This is not a prerequisite for continuing
   the supported-arm baseline. Physical model selection must use actual hardware identity.
6. **Upstream contribution preparation.** Separate reusable LeRobot embodiment/control
   changes from simulation and XR tooling; review dependencies, API boundaries, tests,
   provenance/licensing, and reproducible setup. Prepare focused patches aligned with
   LeRobot subsystem/script conventions. This is separate from experimental acceptance.

## First Task Implementation Scope

Use the migrated structure, not a second simulation/physical copy of the suite:
- `diagnostics/shared/motion_cases.py`: broader case definitions alongside the baseline.
- `diagnostics/shared/acceptance_metrics.py`: explicit per-case metrics and budgets.
- `diagnostics/shared/reporting.py`: case metadata and provenance as needed.
- `diagnostics/verify_live_control.py`: execute selected cases and assemble evidence.
- `diagnostics/backends/simulation.py`: session durations and orchestration as needed.
- `tests/diagnostics/`: generation, bounds, inactive-arm, metric, and reporting checks.

These diagnostic paths are under `unitree_g1_lerobot/`. Keep root launchers stable.
Do not change runtime gains, IK objectives, model assets, or hardware gates just to improve
scores. Preserve LeRobot-default G1-29 runtime gains and the derived G1-23 runtime gains,
with gravity compensation on. Derived G1-29 gains remain a separate motor comparison.

Expected deliverable: a selectable broader suite through `run_verify_live_control.sh`,
automated tests, and archived case-by-case results for both embodiments, with limitations
clearly distinguished from failures. Keep the original suite reproducible.

## Acceptance Boundaries

Rung 4's core simulation/control capability, visual review, and initial numerical checks
work; broader motion and lifecycle acceptance remain open. Rung 4V has basic headset-video
verification; reconnect/performance acceptance remains open. Neither is fully accepted
solely from visual review or the passing baseline. No physical safety or full-body balance
claim follows. No mandatory new keyboard-control stage is needed.

Archive configuration, revisions, source hashes, numerical reports, and logs. Historical
verification JSON is immutable; retain compact summaries under `docs/verification/` and
reference larger local artifacts explicitly. Physical G1-29 baseline (5a), then G1-23 (5b),
remains the separate [sim-to-real workstream](sim-to-real-plan.md).
