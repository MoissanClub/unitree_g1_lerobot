# unitree_g1_lerobot

VR teleoperation of Unitree G1-29 and G1-23 on the LeRobot stack, targeting simulation
and physical robots, with optional BrainCo hands. Simulation functionality exists;
physical acceptance and full cross-backend parity remain separate deliverables.

## Project Direction

Deliver a usable system comparable in explicitly selected capabilities to Unitree
xr_teleoperate. Optimize for upstream-owned implementation and low long-term fork
maintenance, while making each contribution easy to review and reproduce.
Do not wait for future SONIC support to deliver the decoupled arm-control path.

| Authoritative guide | Owns |
|---|---|
| [Goals and success criteria](docs/project-goals.md) | Product scope, optimization criteria, acceptance boundaries |
| [Development model](docs/development-model.md) | Repositories, branches, submissions, release and maintenance policy |
| [Architecture decisions](docs/architecture.md) | Runtime boundaries, control ownership, IK, XR, simulation and hands |
| [Execution plan](docs/project-plan.md) | Current evidence, delivery milestones, next actions and PR gates |

[Documentation index](docs/README.md) separates current guides, reference material,
verification evidence, and historical plans. Rung labels and old branch stacks are
history, not the current execution model.

## Run the Pinned Simulation Release

Read the [installation guide](docs/coworker-installation.md), then:
```bash
./install_g1_vr_sim.sh
./run_g1_vr_sim.sh --embodiment g1_29
# Or:
./run_g1_vr_sim.sh --embodiment g1_23
```

The installer uses `release/g1-vr-sim-d5e400bc` at the fixed LeRobot commit
`d5e400bcefeccc93ba956ce876530e5283512df1`, not the development branch.
CloudXR EULA acceptance is explicit. A local desktop or working `ssh -Y` is needed
for the spectator window. `--headless --live-xr` selects headset-only live operation;
`--headless` alone selects synthetic replay in this launcher.
Do not run the preserved DDS launchers alongside this operator session.

## Develop

Coworkers testing the latest code should start with the
[latest development guide](docs/coworker-latest.md), not the pinned installer.

The main working branch in `MoissanClub/lerobot` is **`dev/g1-integration`**.
`main` remains the upstream mirror. `g1/bugfixes` and `g1/embodiments` are retained
for existing PRs; `submit/keyboard-arm-control` is a focused submission branch.
Other old feature and integration branches are archived.

For an already-installed branch-aware workspace:
```bash
source ~/lerobot-dev/lerobot-dev.sh
lerobot-switch dev/g1-integration
```
See [workspace setup and provenance checks](docs/development-workspace.md).
The helper sources are local workspace work pending their own publication; use this
guide only where those files are present. A source checkout change alone does not
prove that an existing Python environment imports that checkout.

## Status and Boundaries

Both embodiments have existing arm simulation, mapping/IK, gravity compensation,
dual-arm XR, and historical user-reviewed headset video. Hub-path parity, systematic
recovery/performance coverage, physical control, and BrainCo hardware acceptance
remain open. Reviewed video was not attributed to every embodiment/version combination.

The development starting checkpoint `d89a1d0b` passed 407 targeted tests plus an
offscreen GPU test; this does not establish new headset or physical acceptance.
See [exact verification evidence](docs/branch-stack-verification.md).

## Operator and Diagnostic References

- [Pinned installer and three-process launcher](docs/coworker-installation.md)
- [Preserved experimental DDS workflow](docs/operator-guide.md), [camera/video](docs/camera-streaming.md)
- [Model and IK geometry](docs/independent-geometry.md), [motor comparisons](docs/motor-config-comparison.md)
- [Live simulation acceptance](docs/live-control-acceptance.md), [test organization](docs/verification-organization.md)
- [Read-only physical preflight](docs/physical-preflight.md); passing it never authorizes motion

This repository owns operator setup, acceptance tools and retained experiments.
Reusable production changes belong in LeRobot or the dependency that owns them.
Do not apply the experimental adjacent-checkout patch to the pinned release or
development fork. Its instructions apply only to the [experimental workflow](docs/operator-guide.md).
