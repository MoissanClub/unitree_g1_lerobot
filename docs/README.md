# Documentation Index

## Authoritative Project Guides

Read these in order. Each owns a different part of the plan; other documents must
not redefine project scope, branch policy, architecture, or priorities.

1. [Goals and success criteria](project-goals.md): G1-29/G1-23, sim/real, optional
   BrainCo, upstream ownership and low reviewer burden.
2. [Development model](development-model.md): repositories, five active branches,
   short-lived work/submission branches, release pins and fork-delta management.
3. [Architecture decisions](architecture.md): one configurable robot, processor IK,
   execution authority, device/session reuse, simulator boundaries, hands and video.
4. [Execution plan](project-plan.md): evidence matrix, product milestones, immediate
   actions, proposed submissions and explicit decision/verification gates.

## Run and Develop

| Guide | Scope |
|---|---|
| [Coworker installation](coworker-installation.md) | Default pinned native VR-simulation release; installation and one operator launcher |
| [Coworker latest development](coworker-latest.md) | Latest integration checkout, environment refresh and verification; helper publication prerequisite |
| [Development workspace](development-workspace.md) | Local branch-aware helper, dependency/provenance checks; helper publication remains separate |
| [Experimental operator guide](operator-guide.md) | Preserved external DDS workflow and adjacent-checkout patch, not the default release |
| [Camera streaming](camera-streaming.md) | Experimental capture/frame/session contract and headset commands |
| [G1-23 live simulator](g1-23-live-simulator.md) | Experimental supported-arm DDS backend, not whole-body physics |
| [Simulation-to-physical gates](sim-to-real-plan.md) | Staged physical acceptance; no implicit actuation authorization |
| [Read-only physical preflight](physical-preflight.md) | Implemented passive tooling and source-specific lifecycle audit |

Never mix the release's isolated environment, the development checkout, and the
experimental patch procedure without explicitly choosing a workflow.

## Diagnostics and Evidence

- [Non-physical backlog](future-non-physical-work.md): broader motion, fault recovery,
  timing/endurance, native lifecycle, and model matching.
- [Live control acceptance](live-control-acceptance.md): existing small-signal baseline.
- [Independent geometry](independent-geometry.md), [model-source audit](g1-model-source-audit.md).
- [Motor comparisons](motor-config-comparison.md): distinct gain/compensation experiments.
- [Verification organization](verification-organization.md): tests versus diagnostics,
  shared cases/metrics, backend isolation and reuse.
- [Frozen branch-stack results](branch-stack-verification.md) and
  [reproduction commands](branch-stack-commands.md): historical nine-stage evidence,
  not the new upstream submission ancestry.
- [Reference migration](branch-reference-migration-20260918.md): exact archived refs,
  release tag and ref-only checks.
- [G1 asset provenance](../assets/g1/README.md) and
  [pinned motor sources](../assets/g1/motor_sources/README.md).

Reports under `verification/` are immutable evidence at their recorded revisions.
Missing machine-local logs are not equivalent to failed committed evidence.

## Historical Documents

[Archive index](archive/README.md) preserves old rung plans, installation notes,
handoffs and branching strategies. Old entry paths are short navigation notices.
Do not execute historical branch deletion, rebuild, patch or install instructions as
current guidance. Active safety limitations in technical guides still apply.

## Maintenance Rules

Update the owning current guide, not a second roadmap. Add results to the appropriate
evidence record with exact revisions; do not overwrite old reports or infer acceptance
from a branch name. Archive superseded reasoning when it has provenance value; remove
duplicate current status prose. Keep release, development, and experimental evidence
separate. Documentation cleanup itself does not establish a new runtime test result.
