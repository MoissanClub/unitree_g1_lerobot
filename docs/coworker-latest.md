# Coworker Guide: Latest Development

Use this to test or build on **`MoissanClub/lerobot:dev/g1-integration`**.
It is a moving development branch, not a promoted release. For the supported
single-command installation/launcher workflow, use the
[pinned release guide](coworker-installation.md) instead.

The development setup helper is currently local work pending publication. Before
sharing these instructions, publish `tools/dev_workspace/` and its dependencies
with this guide. A fresh clone missing those files cannot perform this setup;
do not substitute the pinned installer and assume it installed development.

## Install Once

Prerequisites: Linux x86-64, Bash, Git/Git LFS, a C compiler, `uv`, Python 3.11+
at `/usr/bin/python3`, and Miniforge. GPU rendering and XR require working NVIDIA
drivers and Vulkan/EGL. The helper does not install host drivers or accept licenses.
See [workspace details](development-workspace.md) for dependency and isolation limits.

```bash
git clone https://github.com/MoissanClub/unitree_g1_lerobot.git "$HOME/unitree_g1_lerobot"
cd "$HOME/unitree_g1_lerobot"
test -f tools/dev_workspace/install.py
/usr/bin/python3 tools/dev_workspace/install.py --root "$HOME/lerobot-dev"
source "$HOME/lerobot-dev/lerobot-dev.sh"
lerobot-switch dev/g1-integration
lerobot-check
```

Run each command only after the preceding one succeeds. If already cloned, use
that checkout rather than cloning over it. For nondefault Miniforge locations,
follow the `--conda`/`LEROBOT_DEV_CONDA` instructions in the workspace guide.

The helper selects `~/lerobot-dev/lerobot`, activates `lerobot-dev`, installs that
checkout editable, and prepares branch-matched models. Check its reported Python,
source path, commit and `G1_KINEMATICS_ASSETS`. Do not set `PYTHONPATH` or reuse
another checkout's asset directory. No simulator or physical robot is started.

## Get the Latest Commit

Stop all processes using this checkout/environment first. The switch helper does
not automatically pull an existing branch. In Bash:

```bash
source "$HOME/lerobot-dev/lerobot-dev.sh"
lerobot-switch dev/g1-integration
git fetch origin
git merge --ff-only origin/dev/g1-integration
lerobot-switch dev/g1-integration
lerobot-check
git rev-parse HEAD
```

The second switch refreshes dependencies/assets for the updated source. If a dirty
checkout or divergent history blocks this sequence, preserve your work and resolve
it deliberately; do not reset it. Stop if setup reports NOT READY. One environment
and checkout must not serve different branches concurrently.

## Verify Both Embodiments

Run from the selected LeRobot checkout, using the activated interpreter:

```bash
python -m pytest -q -rs tests/robots/test_unitree_g1_simulation.py
python examples/unitree_g1/run_xr_simulation.py \
  --assets "$G1_KINEMATICS_ASSETS" --embodiment g1_29 --headless --steps 100
python examples/unitree_g1/run_xr_simulation.py \
  --assets "$G1_KINEMATICS_ASSETS" --embodiment g1_23 --headless --steps 100
```

Each replay should finish successfully and report control frames, camera frames
and nonzero motion. Here `--headless` means synthetic controller replay, not live
headset input. It does not test CloudXR or headset reception. Inspect test skips;
the [test inventory](branch-stack-commands.md) provides additional focused suites,
but its archived branch-switch commands are not part of this latest-branch workflow.

## Live Headset Review

The current development example consumes an existing CloudXR runtime; it does not
start one. Arrange the installed SDK's CloudXR service and explicitly accept its
EULA first. The example's shell must receive that runtime's OpenXR environment,
including the correct `XR_RUNTIME_JSON`; starting a service in another terminal
does not propagate environment variables. Do not start a second simulator/bridge
or the pinned operator launcher alongside it.

With that runtime ready, from the development checkout:

```bash
python examples/unitree_g1/run_xr_simulation.py \
  --assets "$G1_KINEMATICS_ASSETS" --embodiment g1_29 --video --steps 0
```

Connect through the Isaac Teleop headset client as described in the
[connection instructions](coworker-installation.md#three-services-one-launcher).
Review independent arm clutches, translation, wrist rotation, release/re-engagement,
and fresh camera images. Stop the example before repeating with `g1_23`; stop
CloudXR after the example disconnects. This example has no desktop spectator window;
SSH X forwarding is not required for headset video. X/headset review is manual.

There is currently no documented latest-branch equivalent of the pinned
`./run_g1_vr_sim.sh` installation contract. Do not edit `.vr-sim.json` or switch its
managed dependency checkout to bypass the pin. Use the pinned workflow when a
turnkey three-service launcher is the priority.

## Report Results

Record both repository SHAs, `lerobot-check` output, exact commands, test skips,
embodiment and headless versus headset mode. A moving branch name alone is not
reproducible evidence. This workflow is simulation-only, not physical acceptance.
See the [development model](development-model.md) for contribution policy.
