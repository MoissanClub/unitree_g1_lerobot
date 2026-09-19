# Goals and Success Criteria

## Product Goal

Build controller-based VR teleoperation of **G1-29 and G1-23 on LeRobot**, for
simulation and physical robots, with and without BrainCo hands. The reference is
[Unitree xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate), not a
promise to reproduce every input device, end effector, or locomotion mode immediately.

A better architecture means shared LeRobot robot/processor interfaces, reusable
device/session infrastructure, explicit command ownership, standard recording,
and reproducible installation and verification. It does not mean inventing another
general-purpose robot framework.

## Optimization Criteria

1. **Upstream ownership and sustainable maintenance.** Maximize work accepted by
   LeRobot or the dependency that owns it. Minimize duplicated runtimes, fork-only
   public APIs, dependency churn, and independently evolving implementations.
2. **Low reviewer burden.** Prefer existing patterns and one bounded behavior change
   per PR, with compatible defaults, explicit dependencies, a reproducible example,
   focused tests, and clear limitations. Minimize reviewer context, not merely diff size.

Product delivery remains a constraint on both objectives: neither waiting indefinitely
for upstream nor dropping G1-23 meets the goal. A small maintained plugin/operator
adapter can be preferable to forcing unsuitable functionality into LeRobot core.
Release a tested integration while tracking each remaining downstream delta.

## Scope Contract

| Capability | Delivery intention | Evidence boundary |
|---|---|---|
| Independent controller-driven arms, G1-29 and G1-23 | Baseline | Each embodiment verified separately; G1-23 has five joints per arm and cannot generally satisfy arbitrary six-dimensional poses |
| MuJoCo simulation and physical deployment | Required product outcomes | Fixed-base simulation is not balance, collision, or hardware acceptance |
| Camera feedback in the headset | Baseline | Physical robot-to-workstation transport and frame age require separate checks |
| No hand / optional BrainCo | Required configurations | Hand transport, articulated hand physics, and hardware grasping are separate capabilities |
| Standard LeRobot recording | Product milestone | Joint actions first; Cartesian or SONIC-token datasets need their own schema and validation |
| Stationary or support-backed bimanual operation | Proposed first deployment scope | Support and control-authority mode must be selected explicitly; this does not establish free-standing safety |
| Mobile manipulation and skeletal finger input | Later scope decisions | Do not silently count these as completed reference parity |
| SONIC-based VR | Optional future control path | Not a prerequisite for G1-23 or current decoupled arm delivery |

Before claiming parity with the reference project, record a match/defer decision for
controller input, hand tracking, displays, recording, hands, and motion mode. BrainCo
articulated simulation is currently unimplemented; deciding its required fidelity is
an explicit product decision, not something fake-driver tests can close.

## SONIC Position

The inspected LeRobot code includes a 64-dimensional token decoder producing
29-joint commands. Isaac Teleop also describes SONIC tracking integration.
These are implementation signals, not a commitment to a complete LeRobot VR roadmap
or G1-23 SONIC support. Decoupled arm IK is a supported path, not disposable scaffolding.
Share device acquisition, cameras and hands where contracts fit; do not conflate
joint targets with latent tokens or run competing arm command writers.

Sources are moving references, not pinned acceptance evidence:
[LeRobot G1](https://github.com/huggingface/lerobot/blob/main/docs/source/unitree_g1.mdx),
[Isaac Teleop](https://github.com/NVIDIA/IsaacTeleop).
Recheck revisions before making implementation decisions.

## Definition of Done

For each supported embodiment/backend/hand combination, record source/model revisions,
reproducible setup, input/control/camera behavior, fault and recovery results, and
remaining limitations. Separate automated, manually observed, and physical evidence.
A PR merge, startup window, mocked SDK test, or aggregate pass count is insufficient
to mark the entire product complete.

The [execution plan](project-plan.md) owns status and milestones.
The [development model](development-model.md) owns delivery and review policy.
