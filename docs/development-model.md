# Development and Contribution Model

## Repository Ownership

| Repository / area | Responsibility |
|---|---|
| MoissanClub/lerobot | Authoritative reusable LeRobot implementation and combined development |
| MoissanClub/unitree_g1_lerobot | Pinned installation, operator launch configuration, acceptance tools and retained experiments |
| Existing Hub simulator | Simulation environment and model/assets, with placement agreed with its maintainers |
| Isaac Teleop | XR device/session and graphics infrastructure; LeRobot should keep its adapter narrow |

Do not maintain two independently evolving production copies of the same feature.
Legacy experimental code stays only while serving a migration or diagnostic purpose.

## Branches

The non-archived branches at the cleanup checkpoint are:

| Branch | Role |
|---|---|
| `main` | Clean upstream mirror; fast-forward updates, no project development |
| `dev/g1-integration` | Authoritative combined working system; starts at `d89a1d0b` |
| `g1/bugfixes` | Existing PR #4664 head; preserve until merged or deliberately replaced |
| `g1/embodiments` | Existing PR #4651 head; scope/dependency cleanup remains necessary |
| `submit/keyboard-arm-control` | Tested keyboard contribution on the bugfix base |

PR state is a dated observation. Recheck before changing a head.
Create short-lived `work/<feature>` branches from integration as tasks start.
Create curated `submit/<feature>` branches when a contribution is ready for extraction;
do not create empty placeholders or rename active PR heads merely for consistency.

Old stages live under `archive/feature-stack-20260918/g1/*`; old combined checkpoints
under `archive/integration/*`. These are extraction/evidence inputs, not parallel
development lines. The [migration record](branch-reference-migration-20260918.md)
preserves exact SHAs. Commits remain available for cherry-picking without active
feature branches.

## Normal Change Flow

1. Implement and review a coherent task on `work/*`; preserve passing integration.
   An independent upstream fix may start on a submission branch instead.
2. Separate changes by concern. Use the existing robot, processor, device and
   environment contracts before adding abstractions.
3. Extract submissions onto the appropriate accepted upstream state. Cherry-pick
   self-contained commits; otherwise port selected changes deliberately.
   Do not copy entire integration files that contain unrelated sibling features.
4. If a PR is intentionally stacked, declare its submission parent. Review order
   is not runtime dependency: keyboard is not required by IK, and hands do not require XR.
5. Reconcile reviewer changes back into integration promptly. Limit concurrent
   review to maintainer capacity, not a rigid global one-PR rule.
6. After acceptance, merge upstream into shared integration and reconcile changes
   semantically. Audit the remaining fork delta and rerun affected integration gates.

After an upstream squash merge, record old parent and accepted SHA. Do not blindly
revert the old feature, overwrite complete files, or routinely rebase shared integration.
Use `rebase --onto` only for clean linear submission children; extract explicit patches
for mixed merge histories. Never rebuild the complete published stack merely because
the preferred PR order changes.

## Review Contract

Each submission states one behavior change, actual prerequisites, unchanged defaults,
optional-dependency behavior, exact test/demo commands, evidence and unsupported modes.
Prefer independently useful slices over infrastructure-only abstractions; a complete
product milestone may compose several small PRs. Preserve known G1-23 requirements
when publishing interfaces, without requiring a new generic solver protocol.

Maintain `docs/delivery-ledger.md` in the LeRobot integration branch as the next
administrative deliverable; it is not yet claimed to exist. Rows need owner, source and
accepted commits, dependencies, target repository/PR, exact evidence, remaining fork
delta, and a replacement condition or justified ongoing maintenance responsibility.

## Releases and Reproducibility

The operator installer uses tag `release/g1-vr-sim-d5e400bc` and enforces full commit
`d5e400bcefeccc93ba956ce876530e5283512df1`. It must not follow moving development.
Promote a new release only after explicit installation/runtime and manual acceptance.

Historical reports are immutable. Archived source is pinned by SHA in the historical
runner. Passing tests on development does not retroactively update those reports.
The release, development checkpoint, and experimental DDS workflow need separate evidence.

For setup, see [development workspace](development-workspace.md); for the preserved
nine-stage test series, see [historical commands](branch-stack-commands.md).
Existing user checkouts and unpublished work are not silently switched or reset.
