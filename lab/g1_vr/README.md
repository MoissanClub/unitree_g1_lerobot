# G1-29 XR simulation-lab scripts

These scripts provide a repeatable, simulation-only path from workstation
precheck through XRoboToolkit capture, trace validation, native MuJoCo replay,
and a live Quest video/control test. They never select physical DDS or authorize
physical G1 motion.

Run all commands from the integration repository. Keep the headset off until a
script explicitly accepts the handoff phrase.

## 1. Precheck

```bash
./lab/g1_vr/01_precheck.sh
```

The precheck creates `outputs/g1-vr-lab/<timestamp>/`, records revisions and
the URDF hash, checks the pinned imports, PC Service listeners, GPU/NVENC,
speaker, desktop display, disk space, and absence of a G1 DDS process. It writes
the selected path to `outputs/g1-vr-lab/current-session`; scripts 2–5 refuse to
run unless that session has a passing precheck.

The headset/app network link is intentionally not required during precheck,
because the headset is expected to be off.

## 2. Record a trace

Example right-only capture:

```bash
./lab/g1_vr/02_record_trace.sh \
  --label right-motion \
  --instruction "Hold only the right grip. Move and rotate the right controller."
```

Recommended labels/instructions are:

| Label | Instruction |
| --- | --- |
| `idle-released` | Keep both grips released and controllers still. |
| `right-motion` | Hold only the right grip; move and rotate the right controller. |
| `left-motion` | Hold only the left grip; move and rotate the left controller. |
| `bimanual-motion` | Hold both grips; move both controllers slowly. |
| `buttons-head` | Move the head and press each requested trigger/button. |
| `tracking-reconnect` | Follow the spoken tracking-loss/reconnect procedure. |

The backend connects while the headset is off, then waits. At the one terminal
prompt, type exactly:

```text
wearing the headset now
```

Press Enter and put on the headset. Ten seconds later, voice announces the
recording start and no terminal interaction is needed. The recording stops at
the configured duration. Voice then asks for both grips to be released, the
controllers to be put down, and the headset to be removed. After returning to
the terminal, type `headset off` to close the run boundary.

The trace is never overwritten if the label already exists.

## 3. Validate recorded data

```bash
./lab/g1_vr/03_check_trace.sh --label right-motion --expect right
```

Validation checks all recorded checksums, schema/order, duration/rate, ready
source state, tracking ratios, grip/trigger/button ranges, and controller motion.
Behavior expectations are `any`, `idle`, `right`, `left`, `bimanual`,
`buttons-head`, and `reconnect`. A JSON report is saved under `checks/`.

## 4. Replay through native MuJoCo

```bash
./lab/g1_vr/04_replay_trace.sh --label right-motion --expect right
```

The trace is validated first, then replayed at 60 Hz through the shared guard,
G1-29 retargeter, upstream LeRobot native MuJoCo robot, and a viewer. Telemetry
and logs are saved under `telemetry/`. Use `--headless` for automated regression.

For a deterministic headset-free test of the same validation/replay path:

```bash
python tools/synthesize_g1_vr_trace.py /tmp/g1-vr-synthetic \
  --scenario bimanual --duration-s 4
./lab/g1_vr/03_check_trace.sh \
  --trace /tmp/g1-vr-synthetic --expect bimanual
./lab/g1_vr/04_replay_trace.sh \
  --trace /tmp/g1-vr-synthetic --expect bimanual --headless
```

Synthetic traces are development fixtures, not headset or operator evidence.

## 5. Live end-to-end control and Quest video

```bash
./lab/g1_vr/05_live_e2e.sh --duration-s 60
```

Before prompting, the script starts the exact native MuJoCo simulation and
viewer, connects the XR source, and makes the Quest H.264 desktop-video listener
ready. It then uses the same `wearing the headset now` / ten-second handoff as
trace capture. Spoken stages cover right-only, left-only, bimanual, release/hold,
and video observation. The run ends automatically; voice announces when to put
down the controllers and remove the headset. The final terminal confirmation is
`headset off`.

Quest video captures the desktop containing the viewer for the same simulation
being controlled; it is not a second or synthetic robot display. Video, backend
logs, and control telemetry share one run directory under `live/`.

## Configuration overrides

Defaults use the pinned clean-install workspace. Override only when deliberately
testing another audited checkout:

- `G1_VR_PLUGIN_ROOT`
- `G1_VR_LEROBOT_ROOT`
- `G1_VR_PYTHON`
- `G1_VR_ASSETS_ROOT`
- `XRBT_SDK_SITE_PACKAGES`
- `G1_VR_LAB_OUTPUT_ROOT`
- `G1_VR_VOICE_RATE` (default `-35`)
- `G1_VR_DONNING_SECONDS` (default `10`; shorten only in automated tests)

If a backend fails while the headset is on, voice still instructs the operator
to release grips and remove the headset. Artifacts are retained; rerunning never
reinterprets a failed or partial run as a pass.
