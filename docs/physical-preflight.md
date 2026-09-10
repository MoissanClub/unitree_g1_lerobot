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

## LeRobot physical object initialization

`verify_g1_lerobot_initialization.sh` runs the real patched LeRobot constructor with
`is_simulation=False`, `controller=None`, empty cameras, and gravity compensation off.
It checks G1-29 and G1-23 physical backend selection, exact joint/arm mappings against
this project's motor profiles, feature schemas, disconnected state, sparse gains, and
G1-23's retained hardware capability guard. A third test serializes/deserializes the
physical configuration and constructs the object through LeRobot's robot factory.

```bash
LEROBOT_ROOT=/path/to/patched/lerobot G1_INIT_PYTHON=/path/to/python3.12 \
  ./verify_g1_lerobot_initialization.sh
```

Use the pinned checkout and patch recorded in `configs/physical_preflight_audit.json`.
The runner verifies audited file hashes and imported source location; a missing or
incompatible checkout or dependency fails the dedicated command. Generic test discovery
skips this integration suite unless `LEROBOT_ROOT` is set. Python 3.12 or newer is needed
for the pinned source; install the LeRobot robot-stack dependencies and Unitree SDK in
that interpreter. The existing Python 3.10 `g1brainco` environment can run passive DDS
preflight but cannot import this LeRobot revision.

The constructor/configuration/feature code and package checks run unmodified. Test guards
fail on transport initialization, publisher/subscriber construction, socket creation via
ZMQ, DDS domain creation, thread starts, or calls to connect/reset/send/disconnect.
Calibration files are isolated in a temporary directory. Object destruction occurs while
guards remain installed, so an unexpected destructor disconnect is also detected.
This verifies construction with disabled controllers/cameras/gravity IK; it does not test
those optional subsystems or the physical connection/ownership path. It sends no robot
commands and requires no robot connection.

## Verified LeRobot read-only connection

The LeRobot patch now adds an explicit `UnitreeG1Config(read_only=True)` path. This is
physical **DDS observation through the real LeRobot class**, not the existing ZMQ command
backend. `read_only=False` retains the existing behavior described in the lifecycle audit.
Read-only mode requires `is_simulation=False`, an explicit non-loopback
`network_interface`, no controller, no cameras, and gravity compensation off.

`connect()` creates only an HG lowstate subscriber and waits up to `state_timeout_s`
(default 5 s). It rejects invalid active-joint feedback and changes in mode fields,
regressing/stalled ticks, and excessive feedback gaps. `get_observation()` exposes the
normal LeRobot observation dictionary and rejects stale state using `max_state_age_s`
(default 0.1 s). `send_action()`, `publish_lowcmd()`, and `reset()` raise in this mode.
`disconnect()` closes the subscriber without publishing zero torque or changing robot
control mode. Reconnecting the same object is rejected; use a fresh process for each
session because SDK channel initialization and native teardown remain process-level concerns.
G1-23 may be observed passively, but its command-capable hardware guard stays false.

The direct API is:

```python
from unitree_g1_lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config

robot = UnitreeG1(UnitreeG1Config(
    embodiment="g1_29", is_simulation=False, read_only=True,
    network_interface="enP8p1s0", controller=None,
    cameras={}, gravity_compensation=False,
))
try:
    robot.connect()
    observation = robot.get_observation()
finally:
    robot.disconnect()
```

For a bounded verification with a durable report and guards against creating command
publishers or ZMQ sockets:

```bash
G1_INIT_PYTHON=/tmp/g1-lerobot-init-env/bin/python \
LD_LIBRARY_PATH=/home/dwei/demos/.deps/cyclonedds-install/lib \
  ./verify_g1_lerobot_connection.sh \
  --lerobot-root /tmp/g1-stage0-lerobot --network-interface enP8p1s0 \
  --embodiment g1_29 --duration-s 5 \
  --output artifacts/physical/g1_29/lerobot-connect-002.json
```

The `/tmp` paths are this session's isolated test environment/checkout, not permanent
installation locations. Python 3.12.14, torch 2.11.0+cpu, draccus 0.11.6,
unitree_sdk2py 1.0.1, and cyclonedds 0.10.2 were used. No production robot environment
was replaced. Full environment versions are saved locally beside the report.

**Hardware result, 2026-09-11:** the actual constructor, `connect()`,
`get_observation()`, and `disconnect()` passed on `enP8p1s0` for expected G1-29.
There were 839 observation calls and 838 distinct tick snapshots during five seconds;
all 29 expected joint positions were present. Maximum sampled receive gap was 13.16 ms
and final sampled age 11.00 ms against the 100 ms bound. There were zero command-transport
attempts and no report errors. One SDK `[Reader] take sample error` warning was printed;
its cause remains unresolved. The process exited 0. Report:
`artifacts/physical/g1_29/lerobot-connect-001.json` (local, ignored).
These approximately 168 Hz snapshots measure diagnostic polling, not the DDS receive rate.
Command topics were not monitored by this LeRobot verification.

The current audit hashes include this read-only extension. The earlier passive hardware
report used the prior patch at project commit `59cde76`; its recorded hashes are preserved.
This run verifies the passive LeRobot lifecycle, not command acquisition, ZMQ server
operation, balance ownership, physical joint identity, or the Stage 1 stop contract.

Offline lifecycle tests additionally inject receiver startup failure, missing/stale/invalid
feedback, mode changes, forbidden commands, and repeated disconnect/reconnect:

```bash
LEROBOT_ROOT=/tmp/g1-stage0-lerobot \
LD_LIBRARY_PATH=/home/dwei/demos/.deps/cyclonedds-install/lib \
  /tmp/g1-lerobot-init-env/bin/python -m unittest discover \
  -s tests -p test_lerobot_readonly_connection.py -v
```

When updating a checkout with the older embodiment patch already applied, reverse that
exact older patch before applying the current patch. Preserve unrelated changes and use
`git apply --reverse --check` first; the helper deliberately refuses incompatible trees.
A fresh checkout at the pinned revision can use `apply_lerobot_embodiment_patch.sh` directly.
