# Future Non-Physical Work

## Checkpoint: 2026-09-10

The simulation workflow is implemented: configurable G1-29/G1-23 support, embodiment-specific
mapping/IK/motor settings, gravity compensation, MuJoCo/DDS feedback, and bilateral XR control.
Dual-arm headset control was visually reviewed. Camera capture and OpenXR submission passed
headless checks on both embodiments; the user also verified video in the VR headset.
The video review did not specify an embodiment or establish exhaustive lifecycle acceptance.

Implementation commit: `4612a5c`. The project suite passed 45 of 47 tests (two opt-in tests
skipped); the GPU display/mailbox tests passed separately. Headless video evidence is in
`/tmp/g1-xr-headless-3hc8owod` (temporary, not a durable archive).

The following is future acceptance and reliability work, not a request to begin more
implementation today. Keep physical robot actuation out of these experiments.

## Recommended Sequence

1. **Per-joint DDS verification, both embodiments.** Use an independent DDS sender to
   exercise each active arm joint. Verify names, sparse slots, directions, limits, unused
   transport slots, and measured response. Preserve the existing mapping/contract tests;
   this adds systematic live-path evidence rather than replacing them.
2. **Numerical end-to-end tracking.** Send scripted Cartesian and orientation sweeps through
   IK -> DDS -> actuators -> measured joint feedback -> measured hand FK. Report target,
   commanded, and measured poses, separating IK residuals from actuator tracking error.
   Cover single-arm and bilateral motion, holds, reversals, and workspace boundaries.
   Respect G1-23's five-joint position/orientation compromise. Establish explicit tolerances
   before declaring acceptance; do not infer hardware limits from simulation results.
3. **Failure and recovery.** Exercise independent clutch engagement/release, invalid or lost
   tracking, headset disconnect/reconnect, stale commands, and simulator/camera/CloudXR
   restarts. Record the intended hold/release behavior, recovery time, and absence of stale
   input replay or unexpected motion. Camera loss should show a placeholder without blocking
   control. Preserve the single command-publisher rule and simulation-only safeguards.
4. **Performance and sustained operation.** Compare video off/on using identical trajectories
   and settings. Measure physics real-time factor, control-loop timing/jitter, tracking error,
   frame rate/drops/age, and resource usage. Distinguish local capture age from headset latency;
   use a separate measurement for end-to-end headset latency. Run longer sessions to detect
   resource growth, stalls, and stability problems. Choose duration and thresholds explicitly.
5. **Native cleanup investigations.** Isolate the DDS callback teardown crash with a minimal
   SDK-only reproducer before attributing it to CycloneDDS or LeRobot. Fresh-process tests
   remain containment, not a fix. Investigate the SDK `XR_ERROR_SESSION_NOT_STOPPING`
   graphics-shutdown warning separately; successful process exit is not proof of correct
   OpenXR lifecycle handling.

Start by combining items 1 and 2 into a repeatable acceptance suite for G1-29 and G1-23.
Then run recovery and sustained-load experiments. Archive configuration, software revisions,
numerical reports, and logs in durable verification artifacts, not only temporary directories.
No additional keyboard-control stage is required: it would reuse IK while adding another
input source, rather than isolating the remaining transport/physics questions.

## Acceptance Boundaries

Rung 4's core simulation capability and visual control checkpoints work; systematic numerical
and lifecycle acceptance remains open. Rung 4V has basic headset-video verification, while
reconnect and performance acceptance remains open. Do not mark either fully accepted based
solely on visual review. No physical safety, maximum payload, or full-body balance claim follows.

After appropriate simulation acceptance and separate hardware safety preparation, the planned
physical sequence is G1-29 baseline first (5a), then G1-23 (5b). Physical work is not part of
this backlog. Upstream contribution preparation remains separate from experimental acceptance.
