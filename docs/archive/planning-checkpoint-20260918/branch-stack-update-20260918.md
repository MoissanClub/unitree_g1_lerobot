# G1 Stack Revision: 2026-09-18

> Historical snapshot, archived during the documentation consolidation. Not current
> setup, branch, or execution instructions. See the [current documentation](../../README.md).

Historical simulation-before-Cartesian checkpoint. The later
[keyboard-first revision](branch-stack-update-keyboard-20260918.md) supersedes
its branch order; preserve this note and its verification report as history.

## Dependency Reorder

The requested review sequence is now:

```text
bugfixes -> embodiments -> simulation -> cartesian-control -> xr -> xr-video
                                   \-> hand-support -> brainco-hands
```

Simulation no longer constructs or imports `G1ArmKinematics`. It derives joint
names from embodiment definitions, validates the URDF, reads limits from MuJoCo,
and computes gravity feedforward from zero-velocity MuJoCo bias forces. Cartesian
and XR callers construct their own solver. The existing simulation example is
joint-controlled; Cartesian verification remains in the Cartesian branch.

The model downloader moves to simulation without changing its historical
`prepare_cartesian_assets.py` filename, preserving existing setup tooling.
The ordinary `lerobot-teleoperate` command now supports the local native viewer
when supplied `simulation_urdf`, `simulation_mesh_dir`, `sim_onscreen=true` and
`sim_publish_images=false`. No new viewer launcher is required.

This is fixed-base arm simulation, not G1-23 locomotion. The external Hub simulator
was not modified. Native simulation is explicitly selected by a local model path;
the existing G1-29 Hub workflow and physical gates are unchanged. Zero joystick
input holds the native posture; nonzero locomotion input fails explicitly.

## Verification

The updated [verification runner](../../../tools/verify_lerobot_branch_stack.py) tests
the new dependency order in a fresh clone. It includes simulation/XR integration
tests in the XR and video branches now that simulation is their ancestor.
New tests cover the existing teleoperate CLI on both embodiments with Cartesian,
Pinocchio and CasADi imports blocked, plus viewer lifecycle without opening X.
Exact final results are in the [verification guide](../../branch-stack-verification.md).

Real X/viewer and headset review remain manual. A mocked viewer lifecycle test
does not establish OpenGL/GLX support in an SSH session.

The operator service accepts joint metadata from both the pinned release's IK
object and the reordered simulator directly. Its 27 contract/process tests passed
against the existing installation; headless dual-arm replay against the new
integration passed for G1-23 and G1-29 with moving joints and distinct camera
frames. These replay runs skip CloudXR and do not establish live headset acceptance.
The installer pin remains unchanged.

## Existing Checkouts

The six reordered feature refs require a history rewrite; ordinary merges cannot
remove Cartesian from simulation's ancestry. Their previous tips are preserved
on origin under `archive/20260918/g1/<name>`. Publication uses explicit expected-tip
leases so concurrent collaborator updates cannot be overwritten.

`g1/bugfixes` and `g1/embodiments` retain their tips. The old integration branches
remain available. The new combined branch is `integration/g1-acceptance-20260918`.
The operator installer pin is not promoted by this change.

The existing `~/lerobot-dev/lerobot` checkout was fetched and its inactive
`g1/simulation` ref updated. Its old local tip is preserved as
`archive/local-g1-simulation-before-20260918-reorder`. The active
`g1/embodiments` checkout and its environment were left unchanged. In that
workspace, `lerobot-switch g1/simulation` now selects the reordered branch;
the rename example below is for other, not-yet-migrated checkouts.

For an existing clean development checkout, preserve each old local branch before
creating its replacement. Example (do not run while an application uses the checkout):

```bash
git fetch origin
git switch g1/embodiments
git branch -m g1/simulation archive/local-g1-simulation-before-reorder
lerobot-switch g1/simulation
```

Repeat the rename for old local Cartesian/XR/video/hand/BrainCo branches that exist.
Choose a different archive name if already used. The switch helper creates a new
tracking branch when the old local name is absent; it intentionally does not reset
or silently rebase an existing branch. Preserve and transplant unpublished local
commits deliberately. Never merge the pre-reorder simulation history into its new
branch, as that would reintroduce the dependency being removed.
