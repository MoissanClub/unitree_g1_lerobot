# Branch-Aware LeRobot Development

Use this when editing and manually verifying contribution branches. It is separate
from the pinned coworker VR installation and never runs tests, launches CloudXR,
accepts a license, or connects to hardware for you.

## Daily Use

In Bash:

```bash
source ~/lerobot-dev/lerobot-dev.sh
lerobot-switch g1/bugfixes

# Run the exact verification commands you want.
python -m pytest -q tests/robots/test_unitree_g1.py

lerobot-switch archive/feature-stack-20260918/g1/simulation
python examples/unitree_g1/run_simulation.py \
  --assets "$G1_KINEMATICS_ASSETS" --embodiment g1_23 --headless

lerobot-switch dev/g1-integration
lerobot-check
```

The helper changes directory to `~/lerobot-dev/lerobot`, activates the dedicated
conda environment `lerobot-dev`, and prints the actual commit, interpreter, imported
source path, asset directory and lock directory. You do not need `PYTHONPATH` or a
separate manual `conda activate`. `lerobot-check` rechecks the setup without switching;
it allows ordinary source edits but rejects changed dependency files or a branch
switched outside the helper.

Stop applications using this checkout/environment before switching. One mutable
checkout is not suitable for running two different branches concurrently, even in
two terminals. An already-running Python process does not reload after a switch.

## Layout

```text
~/lerobot-dev/
  lerobot/                    # Fork checkout
  lerobot-dev.sh              # Sourceable Bash functions
  environment/
    workspace.py              # Setup and verification implementation
    profile.json              # G1 development dependency profile
    locks/<fingerprint>/       # Exact conda builds and resolved pip versions
    native/<revision>/        # Private pinned CycloneDDS library
    state.json                # Last successful setup and inventories
  assets/<script-hash>/        # Models prepared by the selected branch
```

The conda prefix is `<miniforge>/envs/lerobot-dev`. The pre-existing
`~/lerobot-dev/unitree_g1_lerobot` directory is left untouched. The helper does not
modify shell startup files, existing VR environments, Git remotes in other clones,
or the pinned operator release.

## Switching And Isolation

- Refuses tracked changes, untracked files, and unfinished merges/rebases. Commit
  or stash your work yourself; the helper never discards or stashes it.
- Uses a local branch when present. Otherwise fetches origin and creates a tracking
  branch. It does not silently pull, merge, rebase, commit or push.
- Installs only this checkout as editable LeRobot. Removes inherited Python import
  overrides and disables user-site packages. Verifies LeRobot, G1 and processor
  module origins using the selected interpreter.
- Refuses shell aliases/functions named `python`, `pip`, or `pytest` that could
  bypass the selected interpreter. Prefer `python -m pytest` and `python -m pip`.
- Reuses the environment when dependency files/profile/platform match. Changes to
  `pyproject.toml`, `uv.lock`, `setup.py`, `setup.cfg`, or the profile rebuild the
  owned environment from a clean conda base. No other environment is removed.
- Records exact conda artifacts and resolved pip versions per fingerprint. A new
  fingerprint resolves once; subsequent rebuilds replay those saved locks. This is
  a separate G1 profile, not an assertion that all upstream optional extras are
  installed or that upstream `uv.lock` was used unchanged.
- Detects changed installed Python/native package inventories. An unrecorded
  package installation is an error, not silently adopted as the baseline.
- Prepares the selected branch's pinned models into an asset directory keyed by
  its downloader. Verifies URDF hashes and records/checks all downloaded asset
  files, including meshes. Branches before Cartesian support unset the asset path.
- Defaults `MUJOCO_GL=egl`. Sets private Hugging Face/LeRobot cache paths and clears
  test opt-in flags; choose the test flags and viewer mode explicitly yourself.

If setup fails after Git switches, the shell reports **NOT READY** and does not
activate the old development environment as though it matched the new branch.
Fix the reported issue and rerun `lerobot-switch`.

To deliberately update your local branch:

```bash
git fetch origin
lerobot-switch dev/g1-integration
git merge --ff-only origin/dev/g1-integration
lerobot-switch dev/g1-integration
```

To restore a modified environment from its saved lock:

```bash
lerobot-switch --rebuild
```

To add a lasting G1 dependency, edit `environment/profile.json` and rerun
`lerobot-switch`. For project dependency changes, edit/commit the branch's dependency
files and rerun it. The old lock remains available for the old fingerprint.

## Installation

Prerequisites: Linux x86-64, Miniforge, Bash, Git/Git LFS, a C compiler, and `uv`.
Python 3.11+ at `/usr/bin/python3` runs the helper; the created environment uses
Python 3.12. NVIDIA drivers are needed for the GPU/XR features, not modified here.

From this repository:

```bash
/usr/bin/python3 tools/dev_workspace/install.py --root "$HOME/lerobot-dev"
```

Use `--conda /path/to/miniforge/bin/conda` for another installation; also set
`LEROBOT_DEV_CONDA` to that path in shells using the helper. Re-running the installer
updates helper files but preserves the local profile. `--update-profile` explicitly
replaces it with the repository's profile.

The profile includes G1 unit-test, simulation, visualization and XR dependencies,
conda-forge Pinocchio/CasADi, pinned Unitree SDK, private CycloneDDS and optional
BrainCo SDK. It is not a minimal dependency-declaration audit or an environment
for every LeRobot policy. Use a fresh minimal installation separately when reviewing
packaging or proving a feature's declared dependencies are sufficient.

Manual opt-ins and the exact per-branch suites remain in the
[branch verification guide](branch-stack-verification.md). No implicit pytest skip
policy is added to your commands: inspect skip summaries and enable the gates you
intend to exercise. The helper verifies setup, not test coverage or hardware safety.

## Verified On This Host

On September 18, 2026, the dedicated environment was created from scratch and
rebuilt after a dependency-profile change. Real shell switches exercised
the frozen bugfix/simulation snapshots under `archive/feature-stack-20260918/g1/`
and `dev/g1-integration`, including
deliberately incorrect inherited `PYTHONPATH` and `PYTHONHOME`.

- Bugfix branch: 96 tests passed.
- Simulation branch: 15 simulator/runtime tests passed.
- Integration branch at `da1c0fcd`: 320 targeted tests passed, plus one separate
  offscreen GPU test. No skips in these runs.
- Both G1-29 and G1-23 headless simulator and synthetic XR/camera examples passed.
- Helper unit tests cover dirty-checkout preservation, unmanaged-environment
  refusal, dependency fingerprints, Python/native package drift, shell path
  isolation, model checksums, and read-only asset reuse.

The explicit acceptance script is `tools/dev_workspace/verify.sh`; it is not run
by the installer or switch helper. Current JUnit reports are in
`~/lerobot-dev/results/`. The environment occupies approximately 9.6 GB on this
host, excluding caches. No X, headset, physical hardware, or license acceptance
was part of this verification.
