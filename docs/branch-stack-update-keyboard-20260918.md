# Keyboard-First Stack Revision: 2026-09-18

Historical branch names below describe the publication at this checkpoint.
They now resolve under `archive/integration/*`; use `dev/g1-integration` for
ongoing work. See the [reference migration](branch-reference-migration-20260918.md).

## Changes

The first feature PR now adds `unitree_g1_keyboard` to the standard
`lerobot-teleoperate` framework. It requires no exoskeleton and demonstrates arm
control using the existing upstream G1-29 Hub/DDS simulator, before our native
simulation or embodiment changes. The small `g1/bugfixes` branch stays first.

The keyboard uses LeRobot's existing discrete-key listener, including its SSH/TTY
fallback. It sends bounded joint targets, not Cartesian poses. Enter enables from
measured positions; Space holds/disables; Escape exits. Keys select left/right
arm, joint number, and signed increments. Repeated key events are rate-limited,
not queued. It starts disabled and rejects missing/invalid feedback. A stopped
listener or absent feedback for 0.5 seconds terminates control. The feedback timer
measures delivery to the teleoperator, not a hardware sensor timestamp.

The standard CLI rejects physical use and body controllers for this initial
keyboard acceptance. The Hub simulator's default torso support stays active.
Joint limits and a target-lead bound are not collision avoidance or an emergency
stop. Existing upstream exoskeleton control remains available and unchanged.

`g1/embodiments` now follows keyboard control and configures its joint layout from
the robot selection. G1-23 has five joints per arm. The native simulator follows,
with all-joint motion/mapping tests and real PTY CLI tests for both embodiments.
Cartesian/XR/video and the independent hand/BrainCo chain retain their functionality.
The previous Tk joint-control example remains available as a diagnostic artifact.

## Evidence And Environment

The [verification guide](branch-stack-verification.md) owns final tested tips and
results. [Acceptance commands](branch-stack-commands.md) cover each stage, including
the exact pytest paths. The headless PTY test exercises actual terminal input and
CLI parsing; the Hub test exercises real DDS/physics through the standard loop.
Native tests jog each available arm joint separately and compare feedback/limits.
X/headset and all physical acceptance remain manual and unclaimed.

The older test environment aborted in native CycloneDDS before control began.
Selecting the compatible native library built for `lerobot-dev` via
`CYCLONEDDS_HOME` allowed the Hub test to pass. No SDK/library workaround was
embedded into the keyboard class. Run Hub/DDS tests serially, with a compatible
native library and cached model dependencies. The acceptance Hub snapshot is
`68459ed68f6f68e1f661091dfcb6ebce44681aec`.

An additional check in the existing `lerobot-dev` environment found that the
Unitree SDK wheel omitted `utils/lib/crc_amd64.so`. Importing IDL messages alone
had not detected that packaging issue. The missing `utils/lib` package data was
restored from a clean checkout of the exact installed SDK commit
`65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5`; no dependency version or active LeRobot
checkout was changed. Readiness checks should instantiate `CRC`, not only import
the SDK. See the command guide's dependency preflight. Rebuilding this environment
from the same wheel may require restoring the same package data again.

The existing XR interactive example is introduced on the video branch. The
earlier proposed command on the XR-only branch was incorrect; its independent
acceptance command is its pytest controller-to-physics integration suite.

## Existing Checkouts

Old published feature tips are preserved on origin under
`archive/20260918-keyboard/g1/<name>`. The old integration branches and coworker
installer pin are unchanged. The new combined branch is
`integration/g1-acceptance-keyboard-20260918`.

The current `~/lerobot-dev` checkout is not switched or reset automatically.
Before reviewing, stop applications using it and ensure the working tree is clean.
For the new keyboard branch:

```bash
git fetch origin
lerobot-switch g1/keyboard-arm-control
```

For each old local feature name you already have, preserve it before switching
to the rewritten remote branch. Example, while on the new keyboard branch:

```bash
git branch -m g1/embodiments archive/local-embodiments-before-keyboard
lerobot-switch g1/embodiments
```

Repeat for simulation/Cartesian/XR/video/hands/BrainCo branches that exist locally,
using a unique archive name. Do not rename a branch with uncommitted work without
first preserving that work deliberately. Do not merge the old feature history
back into the rebuilt stack. Unpublished local commits need deliberate transplant.

For combined review without migrating each old local branch:

```bash
git fetch origin
lerobot-switch dev/g1-integration
```

Publishing rewritten features uses explicit expected-tip leases, so another
collaborator's changes are not silently overwritten. The active development
environment and the installer pin are not modified by branch publication.
