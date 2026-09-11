# G1-29 Model Source Audit

## Conclusion and Correction

The earlier torso-frame test did **not** establish a 10 mm arm placement error.
It compared identically named torso frames from different G1 model families. Their
origins differ by 10 mm at neutral waist, and their shoulder offsets cancel that shift.
No shoulder translation or fitted residual correction should be applied.

There is still a real mixed-model issue: the IK URDF uses the rev_1_0 waist layout;
the simulation MJCF uses the legacy layout. They agree for neutral-waist arm kinematics,
but not for articulated waist roll/pitch. The checker now uses the shared pelvis frame
and full-URDF FK at received body joint states, retaining torso-only errors as diagnostics.

## Frame Chain

| Torso/waist quantity | Hub IK URDF (rev_1_0 layout) | Hub MJCF (legacy layout) |
|---|---:|---:|
| Waist-roll joint z relative to waist-yaw | 0.044 m | 0.035 m |
| Waist-pitch joint z relative to waist-roll | 0 m | 0.019 m |
| Neutral torso origin z relative to pelvis | 0.044 m | 0.054 m |
| Shoulder-pitch origin z relative to torso | 0.24778 m | 0.23778 m |
| Neutral shoulder z relative to pelvis | **0.29178 m** | **0.29178 m** |

Full-chain numerical audit, using the pinned Hub models without modification:
- Neutral and tested arm configurations with zero waist: pelvis-relative discrepancy below 0.0005 mm.
- Waist yaw +/-0.3 rad: still below 0.0005 mm.
- Waist roll +/-0.3 rad: approximately 2.69 mm position discrepancy.
- Waist pitch +/-0.3 rad: approximately 2.99 mm position discrepancy.
- Raw torso-relative hand discrepancy remains approximately 10 mm in all these cases.

These are kinematic source comparisons, not physical measurements or motor-gain effects.

## Provenance

The checked-in audit JSON pins source URLs, revisions, hashes, origins, and FK results:
[audit artifact](verification/g1-model-source-audit-20260910.json).

- LeRobot Hub snapshot: `a38dc8617f0fca51b38e9354dc58ee35ad850fb5`.
- `xr_teleoperate` introduced the legacy G1 arm model in commit
  [68549a7](https://github.com/unitreerobotics/xr_teleoperate/commit/68549a7920ffb23b77f0a6b9c2e2fcddff06df15).
  Commit [f7bdf9c](https://github.com/unitreerobotics/xr_teleoperate/commit/f7bdf9ce4819a050010ac745c720062cde9a2f63)
  on 2024-12-04 upgraded the URDF; its waist/shoulder signature matches the Hub IK model.
- Unitree's [legacy hand-equipped URDF](https://github.com/unitreerobotics/unitree_ros/blob/7d6075f7f58588b189b940130e3edab3c839b2df/robots/g1_description/g1_29dof_with_hand.urdf)
  has the same waist/shoulder signature as the Hub MJCF.
- Unitree's [rev_1_0 hand-equipped URDF](https://github.com/unitreerobotics/unitree_ros/blob/7d6075f7f58588b189b940130e3edab3c839b2df/robots/g1_description/g1_29dof_with_hand_rev_1_0.urdf)
  has the same signature as the Hub IK URDF, including the rev_1_0 torso mesh name.

Unitree's [version table](https://github.com/unitreerobotics/unitree_ros/blob/7d6075f7f58588b189b940130e3edab3c839b2df/robots/g1_description/README.md)
distinguishes the deprecated legacy hand-equipped model (mode_machine 2), rev_1_0
hand-equipped model (mode_machine 5), and other hardware configurations. Therefore
`g1_29` joint count alone does not uniquely select a physical robot model.

## Canonical-Model Recommendation

1. For the preserved simulation baseline, keep the pinned plant and existing gains.
   Use pelvis-relative geometry checks; do not shift the shoulders by 10 mm.
2. Before adding articulated waist control, use a matched model pair. Retaining the
   current legacy plant calls for a matching legacy simulation URDF; adopting rev_1_0
   calls for its matching MJCF as well. Treat either migration as explicit behavior change,
   with FK, gravity, limits, inertia, mesh, and control regression, not a two-number patch.
3. For physical work, identify mode_machine and the actual hand configuration in passive
   preflight, then select the corresponding official model family. Do not change global
   LeRobot G1-29 defaults or infer hardware identity from the simulator's message field.

No robot model, IK objective, runtime gain, or hardware configuration was changed in this audit.

The corrected live matrix passed both embodiments with exact DDS pairing: maximum
pelvis-relative position discrepancy 0.13527 mm for G1-29 and 0.00054 mm for G1-23.
See [corrected live evidence](verification/independent-geometry-pelvis-20260910.json).

## Reproduce

```bash
conda run --no-capture-output -n lerobot-g1 python -m unitree_g1_lerobot.diagnostics.audit_g1_sources --report outputs/model-audit.json
# Also refresh and pin upstream source signatures/history:
conda run --no-capture-output -n lerobot-g1 python -m unitree_g1_lerobot.diagnostics.audit_g1_sources --upstream --report outputs/model-audit-upstream.json
./run_verify_live_control.sh --geometry
```

The offline audit runs forward kinematics only, including nonzero waist configurations;
it sends no robot commands. Full-workspace and full-body dynamic acceptance remain open.
