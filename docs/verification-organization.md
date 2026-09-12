# Verification Code Organization

Diagnostics are runnable project tools; tests verify those tools and other package
code. Test folders follow LeRobot's subsystem-oriented layout rather than separating
all unit tests from all integration tests. This repository remains a checkout-based
integration workspace; this migration does not add distribution packaging.

## Separate Fork Verification

The contribution branches are tested against their own `src/lerobot` and
`tests/robots`, `tests/teleoperators`, and `tests/integration` suites, not this
experimental package's tests. `tools/verify_lerobot_branch_stack.py` is a new
repository-level orchestrator: it clones the fork, tests branches independently,
merges them sequentially, and records results after each merge. It does not import
the experimental runtime or replace any diagnostic/launcher below. See
[headless verification and evidence](branch-stack-verification.md).

## Experimental Layout

```text
unitree_g1_lerobot/diagnostics/
  shared/        # Baseline motion cases, motor suite, metrics, JSON/provenance
  backends/      # Owned simulation process management
  simulation/    # Geometry, source audit, motor comparisons, startup and camera checks
  xr/            # Three-service headless verification and bridge diagnostic requests
  physical/      # Passive preflight and read-only connection verification
  verify_live_control.py
tests/
  robots/        # Configuration, mapping, constructor and fake-DDS read-only checks
  simulation/    # Physics, camera IPC/rendering, loopback DDS and viewer checks
  xr/            # Bilateral input, video display and bridge contracts
  diagnostics/   # Metrics, case generation, reports, geometry and preflight tools
  fixtures/      # Reserved for shared test-only fixtures
```

## Execution and Dependencies

Tests import package code; package code never imports tests. Core robot implementation
does not import diagnostics. Domain-specific startup hooks in the simulator and XR
launcher still call diagnostics deliberately.

`shared/motion_cases.py` preserves the existing small-motion trajectories;
`shared/acceptance_metrics.py` preserves their existing simulation acceptance budgets.
`backends/simulation.py` owns the isolated simulator/acceptance child sessions.
The live runner still implements simulation-specific DDS measurement and command checks;
it is not a hardware-ready generic executor. There is no physical actuation backend yet.
Reusable trajectories and metrics do not authorize hardware execution or establish
hardware safety limits. Physical diagnostics remain read-only, with existing gates intact.

Root shell launcher names and arguments remain unchanged. Direct module commands now
use domain-qualified paths, for example:

```bash
python -m unitree_g1_lerobot.diagnostics.simulation.audit_g1_sources --help
python -m unitree_g1_lerobot.diagnostics.xr.verify_xr_headless --help
python -m unitree_g1_lerobot.diagnostics.physical.physical_preflight --help
./run_verify_live_control.sh --geometry
```

The old flat diagnostic module paths are replaced, not compatibility aliases. Published
historical verification JSON remains unchanged, including its original paths and hashes.
New live manifests hash the extracted diagnostic modules as well as the entry point.

## Test Discovery

Every test subfolder has `__init__.py` so Python 3.12 unittest discovery recurses.
Run from the repository root in the appropriate environment:

```bash
python -m unittest discover -s tests -t . -v
python -m unittest discover -s tests -t . -p test_motion_cases.py -v
G1_TEST_DDS=1 G1_TEST_CAMERA=1 MUJOCO_GL=egl python -m unittest discover -s tests -t . -v
```

The previous `discover -s tests` form also works. DDS tests retain fresh-process
isolation and require an idle loopback simulation session. `G1_TEST_VIEWER=1` enables
Tk/EGL checks; `G1_TEST_VIDEO=1` enables SDK/CUDA display checks in the Isaac Teleop
environment. These gates describe execution requirements, not physical actuation approval.
Constructor/read-only regression tests use fake DDS and an explicitly selected patched
LeRobot checkout. They do not contact hardware.

For an upstream contribution, retain subsystem ownership for reusable robot code;
adapt diagnostic entry points to LeRobot's scripts/examples conventions with maintainers.

## Migration Verification: 2026-09-10

The migration is complete. [Machine-readable evidence](verification/verification-layout-migration-20260910.json)
records commands, source hashes, and numerical summaries.

- Full regression with real DDS, EGL camera rendering, and Tk interaction:
  unittest reported `Ran 72 tests`, `OK (skipped=3)`. The skipped entries were two
  physical test classes requiring the audited checkout and one SDK video test.
- Separate SDK/CUDA display and mailbox checks: 2 tests passed.
- Offline physical preflight: 11 tests passed. Audited constructor checks: 3 passed;
  fake-DDS read-only lifecycle checks: 7 passed, including inherited constructor checks.
  The adjacent checkout was correctly rejected by the unchanged audit hashes. These
  checks then passed against revision `3f2c29ef7e44b1ddccbcda3b6a63939e53639e9e`
  plus the existing repository patch in `/tmp/g1-migration-audited`. The adjacent
  checkout was not modified; no physical connection or actuation was attempted.
- `run_verify_live_control.sh --geometry`: G1-29 46/46 cases and 3,953 paired geometry
  samples; G1-23 38/38 cases and 3,452 paired samples. Both suites and simulator exits passed.
- The relocated `diagnostics.xr.verify_xr_headless --video` ran the real simulator,
  XR bridge, CloudXR, and camera-reader launchers for both embodiments. Real OpenXR,
  left/right/bilateral mock motion, video uploads, mismatch rejection, and shutdown passed.
  Camera rates were approximately 18.8/18.7 fps. This did not repeat a headset review.
- Both URDF/mesh and IK comparison launchers completed real Tk/EGL runs from `/tmp`.
  The IK viewer exercised the complete 24-second trajectory cycle.
- All four paired motor launchers and `compare_motor_config.sh` completed their full
  34-case suites, image/report generation, and bounded Tk replay under Xvfb.
  The six-panel image was inspected; separate viewer tests checked nonblank/moving
  panels, pause/resume, case selection, restart, and playback controls.
- Relocated pure IK, legacy IK-to-MuJoCo, DDS smoke, and local model-source audit
  tools completed. Legacy smoke output is not a substitute for numerical acceptance.

Local detailed outputs are under `outputs/live-control-20260910-212656/`,
`outputs/migration-launchers-20260910/`, and `outputs/migration-xr-20260910/`.
These larger logs/images are not tracked; the compact evidence JSON is retained in docs.
Package/launcher help and shell syntax checks passed. Verification processes were stopped.
The broader-workspace motion suite remains future work, not part of this migration.
