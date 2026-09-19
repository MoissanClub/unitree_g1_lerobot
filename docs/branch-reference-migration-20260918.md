# Integration Reference Migration

## Development Versus Installation

`dev/g1-integration` is the only ongoing combined development branch in
`MoissanClub/lerobot`. Its starting commit is
`d89a1d0b5f628045cc73293bcdfa2dc6c0808549`; later development must carry its own
verification evidence. `main` remains the upstream mirror. Existing PR heads
`g1/bugfixes` and `g1/embodiments` are preserved. Frozen feature branches
used by the historical regression runner now live under the archive prefix below.

The installer uses the fixed tag `release/g1-vr-sim-d5e400bc` and still enforces
commit `d5e400bcefeccc93ba956ce876530e5283512df1`. Changing the Git reference is
not a dependency upgrade. Existing installations, environment packages, assets,
and operator configuration are not rewritten by this migration.

## Archived Branches

| Previous branch | Frozen replacement | Exact commit |
|---|---|---|
| `integration/g1-acceptance` | `archive/integration/g1-acceptance` | `cd07fbbbe7f7ddaba26d74e595dca449d7bb0a8c` |
| `integration/g1-acceptance-20260917` | `archive/integration/g1-acceptance-20260917` | `da1c0fcd5f6936560536e93af7e1528dbaaf75be` |
| `integration/g1-acceptance-20260918` | `archive/integration/g1-acceptance-20260918` | `3cd00506cce95d54543f051605f1bd3ca28b7b58` |
| `integration/g1-acceptance-keyboard-20260918` | `archive/integration/g1-acceptance-keyboard-20260918` | `d89a1d0b5f628045cc73293bcdfa2dc6c0808549` |

The old generic branch tip differs from the installer's pinned release commit.
The release tag intentionally points to the installer commit, not that tip.
Historical JSON reports and dated notes retain the original branch names;
interpret them through this table and their recorded SHAs. Do not substitute
the current development tip for a historical acceptance result.

Archive branches and the release tag were published before removal of the old
remote branch names. Old remote heads are deleted with expected-SHA leases.
Local branch aliases in the contribution checkout can then be removed; other
checkouts and worktrees are not silently switched, reset, or deleted.

## Feature Stack Cleanup

All nine historical stage tips are preserved under
`archive/feature-stack-20260918/g1/<name>`. This includes snapshots of the two
active PR branches, so future PR edits cannot change historical verification.
The verifier checks exact commit pins before running any stage.

| Original branch | Frozen SHA | Disposition of original name |
|---|---|---|
| `g1/bugfixes` | `e272f3854e280628901c47cdaadf65ac86a5ca5e` | Retained for PR #4664 and its worktree |
| `g1/keyboard-arm-control` | `2343d572a18f05fd658d4cae19838c4c096d1194` | Retired; live submission is `submit/keyboard-arm-control` |
| `g1/embodiments` | `50e0b6913c722cc689f4b8d56b1b0a46557e4a6d` | Retained for PR #4651 |
| `g1/simulation` | `987ff6ec25b236590373c4da86aa63bd55d032fd` | Retired |
| `g1/cartesian-control` | `c1c8435a410b3bb3af177288f417f063c4e0b1e9` | Retired |
| `g1/xr` | `101a996472c0d696297837cbb7d0cf99c11dedc9` | Retired |
| `g1/xr-video` | `44a34f2528a9ae4d907169d926c170743712bfd6` | Retired |
| `g1/hand-support` | `abcc1d676651c25f21779fb41fbbd85daea4fbc1` | Retired |
| `g1/brainco-hands` | `476a8452df0aecd509aa4d5b5c93377f48e317fd` | Retired |

The seven retired names had no open upstream PRs when checked. Their remote
deletions use expected-SHA leases, after archive publication and tooling updates.
No code is deleted: cherry-picks can use archive refs or the original commit IDs.
The existing `~/lerobot-dev/lerobot` local `g1/simulation` checkout is deliberately
not changed; its old remote tracking ref will disappear on fetch/prune. Use
`lerobot-switch dev/g1-integration` for development or select the archived
simulation ref explicitly for historical review.

Feature-ref verification: all nine archive commits matched their pinned SHAs;
a fresh local clone replayed all nine merges without conflicts and produced a
source tree identical to `d89a1d0b5f628045cc73293bcdfa2dc6c0808549`.
The focused reference/installer/helper suite passed 38 tests. This ref-only
migration did not rerun the historical 407 physics/SDK tests or headset checks;
the unchanged source trees preserve that evidence at its original commits.

## Existing Development Checkouts

From a clean, idle checkout, or after saving your work on a separate branch:

```bash
git fetch --prune origin
git switch --track origin/dev/g1-integration
```

If the local development branch already exists, use
`git switch dev/g1-integration` instead. A fetch never deletes local historical
branches. Keep any unpublished work before retiring those local aliases.

For an installed branch-aware workspace, use its helper instead of raw checkout:

```bash
source ~/lerobot-dev/lerobot-dev.sh
lerobot-switch dev/g1-integration
```

Repository development-helper defaults now select that branch for new clones.
Existing deployed helper profiles and current checkouts are preserved; the
explicit command above selects the new branch without requiring a default
profile update. The helper still performs its normal dependency/provenance
checks. The reference-migration tests do not claim a full environment rebuild.

The historical stack verifier now creates `verification/g1-branch-stack` by
default in its fresh trial clone. It does not create or overwrite the working
development branch or recreate an obsolete release branch.

## Verification Scope

The migration checks the release tag via a fresh network clone and unchanged
commit, development-branch resolution, archive-tip equality, installer clone
behavior, and branch-helper bookkeeping. Runtime smoke checks reuse the
existing pinned Python environment and models. This is not a fresh conda
installation, dependency-version upgrade, or new headset/hardware acceptance.

Completed checks for this migration:

- 35 focused installer/operator and development-helper tests passed.
- A fresh network clone of the release tag resolved to the unchanged `d5e400bc`
  commit. Both headless XR examples completed 50 control frames and 25 camera
  frames; measured motion was 0.2581 rad for G1-29 and 0.1173 rad for G1-23.
- The fresh clone fetched and switched to `dev/g1-integration` at `d89a1d0b`.
  Because the smoke clone was single-ref, its fetch mapping was explicitly
  extended before setting up branch tracking; ordinary full clones do not need
  that extra step.
- Ruff, shell syntax, and Git whitespace checks passed for the changed tools.

The pre-existing, not-yet-published development-helper files were updated locally
without bundling that separate implementation into the installer migration commit.
The actively edited consolidated plan and its editor swap file were left untouched;
this note supersedes its earlier statement that the old integration names must remain.
