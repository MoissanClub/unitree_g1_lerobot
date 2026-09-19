# Integration Reference Migration

## Development Versus Installation

`dev/g1-integration` is the only ongoing combined development branch in
`MoissanClub/lerobot`. Its starting commit is
`d89a1d0b5f628045cc73293bcdfa2dc6c0808549`; later development must carry its own
verification evidence. `main` remains the upstream mirror. Existing PR heads
`g1/bugfixes` and `g1/embodiments` are preserved, as are frozen feature branches
used by the old branch-stack regression runner.

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
