# Independent MuJoCo Geometry Verification

## Run and Interpretation

```bash
./run_verify_live_control.sh --geometry
```

The owned headless matrix runs the existing joint and Cartesian/orientation cases on both
embodiments, adding MuJoCo-versus-Pinocchio checks. No headset or hardware is used. G1-29
uses LeRobot's default runtime gains, not the derived G1-29 benchmark gains; G1-23 uses
the derived runtime gains. Both send gravity feedforward through the normal runtime path.

**Audit correction:** the initial 10 mm G1-29 torso-frame result compared different
torso origins. The complete waist chain cancels this offset at neutral waist. See the
[source audit](g1-model-source-audit.md). The checker now uses the common pelvis frame
and actual received waist states. The model families still differ for articulated waist
roll/pitch; neutral-waist arm agreement does not establish full-body model agreement.
No controller gains, physical model, cached Hub files, or IK offsets have been changed.

## Exact-State Comparison

- The simulator's optional `--geometry-log NEW_FILE` wraps the actual lowstate publisher.
  G1-29 publishes before physics integration; G1-23 publishes afterward. Sampling at the
  publish call avoids assuming those two orders are identical.
- Every fifth publication copies qpos and mocap state into private `MjData` and runs
  `mj_kinematics`. This avoids the potentially pre-integration live `xpos`/`xmat` cache,
  without modifying the live model, state, forces, or outgoing DDS message.
- The journal records actual hand transforms, named MuJoCo joint values, tick, float32
  joint-payload fingerprint, simulation time, pelvis reference, raw torso poses, and shoulder mount origins.
- The command process independently receives DDS lowstate. Only exact tick and serialized
  joint-payload matches are compared; no nearest-time interpolation or latest-frame pairing
  is used. Each case requires at least ten matched geometry samples.
- Full-URDF Pinocchio FK uses received body joint values, including the waist, and the
  IK-defined hand frames. It is compared to MuJoCo's transforms in `pelvis` coordinates.
  This removes floating-base motion without assuming the two torso origins coincide or
  fitting any transform to residuals. Schema version 2 rejects old torso-only traces.
- G1-29's reference is 0.05 m along wrist-yaw +X; G1-23's is 0.20 m along wrist-roll +X.
  Both use the joint/body orientation. These are the audited IK operational frames, not
  fingertip or camera frames. Nonzero wrist joint-to-body origin offsets fail the audit.

Maximum error budgets, not p95: **1 mm translation, 0.002 rad orientation, and 1e-6 rad
between named MuJoCo joint values and received DDS values**. Joint tolerance accounts for
float32 wire quantization. All tested cases must meet the budgets and match-count requirement.

Geometry logging is opt-in and synchronous diagnostic I/O at about 50 Hz. It is disabled
in ordinary simulator runs and is not a performance benchmark. Full geometry journals
and reports are written under the matrix's printed `outputs/` directory. The receipt
history is bounded to 65,536 states; the existing child-session timeout bounds the run.

## Historical Torso Observation

The pinned Hub snapshot `a38dc8617f0fca51b38e9354dc58ee35ad850fb5` contains different
shoulder mount heights in its URDF and MJCF:

| Source | Both shoulder-pitch origins, torso-relative z |
|---|---:|
| `assets/g1_body29_hand14.urdf`, used by LeRobot IK | 0.24778 m |
| `assets/g1_29dof_with_hand.xml`, used by live MuJoCo | 0.23778 m |

The local pinned motor-benchmark `assets/g1/motor_sources/g1_29dof.xml` also uses
0.23778 m. Both shoulders differ by exactly +0.01000 m in URDF relative to MJCF.
The observed hand discrepancy is correspondingly approximately +10 mm in torso Z on
both arms through the tested motion, while orientation and DDS joint mapping agree.
The reports include a parsed URDF source hash and actual compiled MuJoCo shoulder origins.

The numbers above are real, but treating them alone as an arm placement error was
incorrect: neutral torso height is 0.044 m in the URDF and 0.054 m in MJCF. Both place
the shoulder at pelvis-relative z=0.29178 m. The [audit](g1-model-source-audit.md) records
the remaining articulated-waist model difference and canonical-model recommendation.

## Historical Torso-Frame Evidence: 2026-09-10

This section is retained as the original measurement, not the current acceptance result.

Superseded torso-only reports: `outputs/live-control-20260910-195017/`. Durable summary, source hashes,
and example paired transforms: [geometry evidence](verification/independent-geometry-20260910.json).

| Metric | G1-29 | G1-23 |
|---|---:|---:|
| Exactly paired geometry samples | 3,905 | 3,418 |
| Maximum hand position discrepancy | 10.00018 mm | 0.00054 mm |
| Maximum orientation discrepancy | 0.00000146 rad | 0.00000146 rad |
| Maximum named-joint / DDS discrepancy | 0.000000030 rad | 0.000000030 rad |
| Existing motor-control cases | 46/46 pass | 38/38 pass |
| Independent geometry | Fail | Pass |
| Simulator exit code | 0 | 0 |

The historical matrix exit code was 1 because its torso-frame comparison failed, not
because the simulator crashed. This is not the current pelvis-frame acceptance result.
G1-23's very small residual reflects agreement between two model
computations at matched states; it is not a claim of submicron physical robot accuracy.

Regression verification: 64 tests ran, 60 passed and 4 opt-in tests were skipped.
The original tests detected the torso offset. Current tests also require neutral-waist
pelvis agreement and detection of the nonzero-waist model-family difference.

## Corrected Pelvis-Frame Evidence

Both embodiments pass with unchanged 1 mm / 0.002 rad / 1e-6 rad budgets. Full logs:
`outputs/live-control-20260910-204619/`; [durable corrected report](verification/independent-geometry-pelvis-20260910.json).

| Metric | G1-29 | G1-23 |
|---|---:|---:|
| Exact DDS-state geometry matches | 3,961 | 3,496 |
| Maximum pelvis-relative position discrepancy | 0.13527 mm | 0.00054 mm |
| Maximum orientation discrepancy | 0.00000146 rad | 0.00000146 rad |
| Existing control cases | 46/46 pass | 38/38 pass |
| Independent geometry / simulator exit | Pass / 0 | Pass / 0 |

G1-29's raw torso-relative discrepancy remains 10.00019 mm in the same run. Its small
pelvis-relative residual is evaluated at actual received waist states, not an assumption
that the dynamic waist stays exactly at zero. The offline nonzero-waist audit still
demonstrates the legacy/rev_1_0 difference; this arm-motion run is not full-body acceptance.

Current regression verification: 65 tests ran, 61 passed and 4 opt-in tests were skipped.

## Next Experiments

The source audit is complete. Follow its matched-model recommendation before enabling
articulated waist control or selecting hardware models. The exact-state comparison covers
the small-signal live suite; larger workspace/limit-boundary cases and service-failure
experiments remain in the [non-physical backlog](future-non-physical-work.md).

Code: `simulation/geometry_trace.py`, `diagnostics/geometry_acceptance.py`, and
`tests/test_geometry_acceptance.py`. The tests cover torso-frame invariance, private-data
kinematics, wire quantization/ticks, pinned source mismatch, wrong offsets, wrong mapping,
and missing pairing. They do not claim real-world geometric calibration.
