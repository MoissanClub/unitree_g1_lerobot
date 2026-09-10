# Physical Stage 0 preflight

Stage 0 tooling is implemented. Physical acceptance and the command/stop contract remain
pending operator review. This tool never enables motion, even when feedback checks pass.

Run from a host on the robot's DDS network using Python with `unitree-sdk2py` installed.
LeRobot, IK, MuJoCo, XR, and the LeRobot ZMQ server are not required for this passive probe.
Use a fresh process for each observation; the native DDS teardown finding remains open.

```bash
# Replace ROBOT_NIC with the verified robot-facing interface.
G1_PREFLIGHT_PYTHON=/home/dwei/miniforge3/envs/g1brainco/bin/python \
  ./run_g1_physical_preflight.sh \
  --embodiment g1_29 --network-interface ROBOT_NIC \
  --duration-s 5 --max-feedback-age-s 0.1 \
  --output artifacts/physical/g1_29/preflight-001.json
```

The launcher uses `python3` unless `G1_PREFLIGHT_PYTHON` is set. Existing SDK/CycloneDDS
library setup must be active. A missing dependency produces a failed JSON report.
The interface is mandatory; loopback and nonexistent interfaces are rejected.
The default domain is 0; change it explicitly with `--domain-id` when appropriate.
The tool does not auto-detect a robot or infer its embodiment from packet length.

Copy `configs/physical_preflight_contract.example.json` to a run-specific file and fill
in externally verified observations. Include it with `--contract PATH`. Include
`--lerobot-root PATH` to hash-check the patched LeRobot source against the recorded
lifecycle audit. Neither option imports LeRobot or starts the server. A missing or
changed checkout is recorded as an unverified audit, not silently treated as equivalent.

The JSON report records expected joint names/slots/model ranges, latest measured active
joint positions/velocities/estimated torques and raw motor status, raw mode fields,
valid sample count, receive timing, tick progression, command-topic sample counts,
source hashes, operator notes, cleanup failures, and unresolved verification boundaries.
Output filenames must be new: existing reports are never overwritten. A killed process
may leave an `incomplete` report, which is not a successful observation.

Feedback checks reject missing/insufficient samples, nonfinite active-joint feedback,
positions outside model ranges with a 0.02 rad sanity tolerance, receive gaps or final
age above the supplied bound, stalled/regressing robot ticks, and changing mode fields.
The freshness bound is a diagnostic threshold, not a selected hardware watchdog setting.
Tick progression and receive timestamps cannot establish absolute source latency.
Raw motor status fields are recorded without claiming fault-code interpretation.
Model bounds and the selected embodiment do not verify physical mapping or joint signs.

Exit 0 means **feedback checks passed only**. Exit 1 means collection/feedback/cleanup
failure; 130 means interruption; argument errors exit 2. Invalid invocation or an
unwritable output can fail before a report is reserved. Every completed report leaves
`stage0_gate` as `pending_external_review` and `motion_enabled` as `false`.

## Preflight's own command/stop contract

- Subscribe to `rt/lowstate`, `rt/lowcmd`, and `rt/arm_sdk` using HG message types.
  Create no application writer, RPC client, robot instance, or command message.
  DDS discovery traffic is expected; there are no motor commands or mode changes.
- Acquire no arm, leg, waist, or balance authority. Existing controllers remain responsible
  for the robot. No command targets, gains, or gravity feedforward are installed.
- On stale/invalid feedback, report failure. On normal exit, interruption, or initialization
  failure, close every created subscriber and record cleanup errors. This performs no
  hold, release, damping, or handover because the observer never owns control.
- Receiving command samples proves activity only. It does not identify/count publishers;
  observing none does not establish that a silent publisher is absent. `mode_machine`
  and `mode_pr` do not identify the MotionSwitcher mode or the owner of each joint.

## Lifecycle audit: pinned LeRobot plus project patch

Audited full checkout: LeRobot `3f2c29ef7e44b1ddccbcda3b6a63939e53639e9e`, with
`patches/lerobot-g1-embodiments.patch` successfully applied. Exact post-patch hashes are
in `configs/physical_preflight_audit.json`. The temporary audit checkout was
`/tmp/g1-stage0-lerobot`; it is not a deployed runtime or a durable dependency install.
All source paths below are under `src/lerobot/robots/unitree_g1/` in that checkout.

| Path | Finding | Consequence for physical work |
|---|---|---|
| `unitree_g1.py`: constructor and `connect()` | Physical mode selects `unitree_sdk2_socket`, opens command transport, seeds active joints from measured state, and may call `reset()` and start a controller thread. | `connect()` is not a read-only probe. Controller configuration changes startup behavior. |
| `unitree_sdk2_socket.py`: `ChannelFactoryInitialize()` | Creates ZMQ PUSH command connection on 6000 and SUB feedback connection on 6001. | Physical LeRobot does not use the direct DDS connection used in simulation. |
| `run_g1_server.py`: `main()` | Calls MotionSwitcher `ReleaseMode()` while an active named mode is reported, then creates the `rt/lowcmd` writer. | Starting the existing server changes control ownership; do not start it for Stage 0. |
| `unitree_g1.py`: `reset()` | Interpolates every active joint toward configured/controller home over three seconds. | Reset is whole-active-layout motion, not an arm-only initialization procedure. |
| `unitree_g1.py`: `publish_lowcmd()` | Sends the complete retained command message; partial action updates leave other slots in it. | An arm-only action dictionary alone does not establish arm-only authority. |
| `unitree_g1.py`: `disconnect()` / `_send_zero_torque()` | Physical exit publishes zero gains/feedforward for all active joints after a bounded controller-thread join; a still-running thread only causes a warning. | Exit makes joints passive, not held; no verified ownership handover and a possible late writer remain. |
| `unitree_g1.py`: state subscription / `get_observation()` | Retains latest state without a physical freshness watchdog; subscription may block. | Receiving a dictionary does not establish fresh state or interruption cleanup. |
| `run_g1_server.py`: forwarding/exit | Forwards incoming commands; no command-age watchdog or explicit hold/handover is implemented in the inspected loop. | Server exit or client loss cannot be used as a verified stopping mechanism. |
| `g1_runtime.py` / local G1-23 registration | G1-23 hardware capability remains false. | Passive G1-23 observation does not enable or validate actuation. |

Local `diagnostics/g1_startup_diagnostic.py` publishes IK startup sequences; the XR bridge
also raises/reorients arms during startup. These simulation launchers are not used by
preflight. No changes to their control behavior or the LeRobot patch are made here.

## External review before Stage 1

Review the report together with the filled contract. Confirm physical identity and mapping,
actual control mode, arm and body ownership/support, selected command interface, and the
single-publisher arrangement. Select gains/feedforward and the measured-state initialization
and authority transition. Define concrete behavior for stale feedback, lost commands,
operator stop/interruption, and normal exit, including who holds or supports the arms.
Record evidence and reviewer identity; prose in a contract file is not automatic acceptance.

Stage 1 must implement the reviewed contract and its motion limits. The observations and
source audit here do not resolve these hardware-specific decisions or pass Stage 1.

## Hardware observation: 2026-09-11

The five-second passive run on `enP8p1s0`, domain 0, with the expected G1-29 mapping
passed feedback checks: 2,384 valid samples at 474.6 Hz, maximum receive gap 7.02 ms,
and final receive/tick-progress age 1.11 ms against the 100 ms bound. All 29 expected
active slots had finite, in-model-range feedback. Raw mode fields stayed at
`mode_machine=5`, `mode_pr=0`; their ownership meaning was not inferred.

The observer received 2,237 `rt/lowcmd` samples and no `rt/arm_sdk` samples. This
establishes existing command activity, not publisher identity or exclusive ownership.
No application commands or mode changes were sent by preflight. The SDK printed two
`[Reader] take sample error` warnings; the report contained no feedback/cleanup errors
and the process exited 0. These warnings are not claimed resolved by that result.

The first attempt failed before DDS connection because the installed SDK's optional
tracing targeted an inaccessible `/tmp/cdds.LOG`. Preflight now removes that tracing
configuration within its own process while preserving interface/transport settings.
Local reports are retained under ignored `artifacts/physical/g1_29/` as
`preflight-hardware-001.json` (initial failure) and `preflight-hardware-002.json` (pass).
The patched LeRobot source matched the lifecycle audit manifest. Physical identity,
control ownership, contract review, and the LeRobot hardware connection remain unverified.

## Offline verification

```bash
./verify_g1_physical_preflight.sh
```

This checks subscriber-only SDK access and cleanup on initialization failure, tick wrap/
regression/stall, stale and invalid feedback, mode changes, sparse G1-23 mapping, and report
semantics. It does not connect to DDS or operate a robot.
