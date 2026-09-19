# Architecture Decisions

## Boundaries and Ownership

Use one top-level LeRobot `UnitreeG1`, parameterized by embodiment and supported
control mode. Avoid parallel G1-29/G1-23 robot classes or duplicate control loops.
The same public method boundary can serve sim and real; it does not imply equal
dynamics, identical feature sets, or validated hardware support.

```text
XR device/session
    -> frame mapping + independent clutch/rebasing
    -> Cartesian end-effector targets + measured state
    -> LeRobot action processor -> reusable IK
    -> named arm joint targets -> UnitreeG1 execution
    -> selected simulator OR explicitly validated physical transport

Camera source -> camera adapter / transport -> shared XR display session
Hand input -> closure mapping / retargeting -> optional per-side hand driver
```

| Boundary | Decision |
|---|---|
| Device input | Reuse/extend Isaac Teleop acquisition and lifecycle; no DDS slot knowledge, robot-specific sim routing, or IK inside the device reader |
| Cartesian adaptation | Stateful processor may consume measured joints and solver state; it does not own the servo loop |
| Kinematics | Reuse existing IK/helpers where possible; preserve joint naming, frames, units, quaternion conventions and tool transforms before publishing APIs |
| Execution | Own gains, bounds, control authority and gravity feedforward downstream of XR; never apply compensation twice |
| Simulation | Prefer existing Hub/DDS path for upstream-facing acceptance; keep the working native arm path until replacement parity |
| Hands | Optional composition beneath the robot; separate from XR, IK and body action schema |
| Video | Reuse session/graphics infrastructure; a small camera adapter in LeRobot, platform work in its owning dependency |

## Embodiments and IK

G1-29 has fourteen arm joints; G1-23 has ten. Preserve the sparse DDS layout for
G1-23 rather than compacting transport indices. Named joint actions are the public
adaptation boundary; model/solver order and DDS order are explicit conversions.

Different embodiments have different attainable poses. Separate reachable-target
acceptance from workspace characterization; do not weaken all orientation thresholds
to hide G1-23's five-joint limitation. Current supported-arm solvers lock non-arm joints;
moving-base/waist reaching needs matched models and separate verification.

Direct joint teleoperation bypasses IK. Intermediate pose matrices do not establish
a complete training/recording schema. Keep initial robot actions compatible with
existing named joint commands.

## Three Runtime Paths

| Path | Current purpose | Limits |
|---|---|---|
| Upstream Hub/DDS | Existing LeRobot G1 Robot-facing simulator; Hub distributes code/assets, execution is local | G1-29 keyboard verified; full Cartesian/XR parity and G1-23 environment migration still need evidence |
| Fork native arm simulator | Existing pinned release and development integration; in-process supported-arm physics | Fixed non-arm joints, disabled collisions, no walking/balance/articulated-hand parity |
| Experimental external DDS tools | Preserved operator/diagnostic workflow in this repository | Adjacent-checkout patch and environment are distinct; not the default coworker installation |

Do not add native simulation as a prerequisite of upstream Cartesian or hand changes.
Coordinate environment/model contributions with the Hub owner. Retire a runtime only
after control, compensation, camera, headless and manual workflows are covered elsewhere.
Keep diagnostically useful differences explicit, rather than claiming full physics equivalence.

The experimental package retains `robots/`, `xr/`, `simulation/`, and
`diagnostics/{shared,backends,simulation,xr,physical}/`.
Root launchers remain stable. [Verification organization](verification-organization.md)
describes reusable cases/metrics and isolated backend execution.
The [experimental operator guide](operator-guide.md) owns patch/setup instructions;
never apply that patch to the pinned release or development fork.

## Physical Control and SONIC

Choose a named command-authority mode before hardware enablement. Explicitly identify
the owner of arms, waist, legs and hands, watchdog placement, handover and stop behavior.
Unitree stock locomotion plus a compatible arm-only path is a candidate, not an automatic
consequence of `is_simulation=False`. Existing learned controllers are another candidate;
neither is assumed valid for both embodiments.

SONIC is a distinct whole-body token-driven mode. Do not let independent arm IK and
SONIC write the same joints. Reuse device/camera/hand layers where compatible, but do
not invent a shared joint/latent target schema. G1-23 delivery does not wait for SONIC.
See [goals](project-goals.md#sonic-position) and [physical gates](sim-to-real-plan.md).

## Hands and Camera

Compose optional left/right hand devices beneath `UnitreeG1`, merge their features,
and dispatch commands without forcing one body-controller action schema.
A simulation end-effector selector is not a complete physical I/O protocol.
Finalize the concrete lifecycle/configuration/fault contract with maintainers before
adding a generic public API. Avoid carrying the compatibility-only `G1WithHands`
wrapper into an upstream proposal.

BrainCo transport tests, hardware mapping, and articulated simulation are different
acceptance classes. Tool transforms and compensation must reflect attached hand mass
and hardware identity, not just an I/O selector.

Use one shared XR input/display lifecycle, not independently competing sessions.
The existing same-host RGB mailbox is not a network camera stream. Physical delivery
must cover robot-to-workstation transport, frame freshness and disconnect behavior.
The existing headset image is a mono virtual monitor, not stereo or head-controlled
camera calibration. Preserve it until any replacement is independently validated.

## Compatibility and Open Decisions

Settled: one configurable robot, named joint actions, processor IK, separate device
lifecycle, downstream execution/feedforward, explicit sim/physical evidence, and
independent hand composition.

Still open: final upstream solver sharing, moving-base/tool-frame contract, G1-23
Hub placement, physical authority mode, hand public API, and XR graphics ownership.
Maintainer discussions should be narrow and timed to the relevant submission.
Neither the archived fork APIs nor this design document imply upstream approval.

The historical experimental runtime notes are retained in the
[architecture snapshot](archive/planning-checkpoint-20260918/architecture.md).
