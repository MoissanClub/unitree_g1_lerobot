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
the URDF hash, checks the pinned imports, starts the XRoboToolkit PC Service when
its two listeners are absent, checks the resulting listeners, GPU/NVENC,
speaker, desktop display, disk space, and absence of a G1 DDS process. It writes
the selected path to `outputs/g1-vr-lab/current-session`; scripts 2–4 refuse to
run unless that session has a passing precheck.

The headset/app network link is intentionally not required during precheck,
because the headset is expected to be off.

The service launcher is discovered below `XRBT_WORKSPACE` or at
`/opt/apps/roboticsservice`. Set `G1_VR_XRBT_PC_SERVICE_DIR` to an exact service
directory when using another installation. Precheck starts the binary as the
transient user unit `xrobotoolkit-pc-service.service`, allowing it to remain
active across scripts while keeping its output in the evidence session. Precheck
refuses to start a duplicate when a service process already exists but one of the
required listeners is missing; inspect that failed process instead.
`--skip-service-start` retains a check-only mode.

## 2. Record the calibration suite

Before wearing the headset, rehearse the exact phase order and voice prompts:

```bash
./lab/g1_vr/02_record_trace.sh --dry-run
```

The walkthrough is accelerated: it speaks every real instruction and countdown
but skips each 5–10 second capture window. It never requires precheck, prompts
for headset handoff, connects to XRoboToolkit, writes data, or changes suite
resume state. To practice one instruction in isolation:

```bash
./lab/g1_vr/02_record_trace.sh --dry-run-only calibration-forward-inward --dry-run
```

Collect the full suite in one headset run:

```bash
./lab/g1_vr/02_record_trace.sh --max-attempts 3
```

The fixed, versioned suite first captures calibration anchors and only then
captures motion-validation traces. It uses explicit spoken instructions:

This is **human–VR calibration**, not robot or IK calibration. It estimates the
operator/body frame, human workspace scale and offset, and controller-to-palm
orientation. Retargeting consumes that result to construct embodiment-specific
end-effector targets; the G1 URDF and IK policy remain downstream inputs. The
anchor set records this boundary as `retargeting_before_robot_ik`.

| Label | Requested pose or motion |
| --- | --- |
| `calibration-down-inward` | Arms naturally down; palms inward. |
| `calibration-forward-up` | Arms fully forward; palms up. |
| `calibration-forward-inward` | Same forward position; palms inward. |
| `calibration-forward-down` | Same forward position; palms down. |
| `calibration-forward-outward` | Same forward position; palms outward. |
| `calibration-horizontal-up` | Horizontal T-pose; palms up. |
| `calibration-horizontal-forward` | Same T-pose; palms forward. |
| `calibration-horizontal-down` | Same T-pose; palms down. |
| `calibration-horizontal-backward` | Same T-pose; palms backward. |
| `calibration-up-inward` | Arms fully upward; palms inward. |
| `right-forward` | From the 90-degree pose, hold right grip; fully extend forward and return. |
| `right-up` | From the 90-degree pose, hold right grip; move upward and return. |
| `right-outward` | From the 90-degree pose, hold right grip; move directly right and return. |
| `left-forward` | Mirrored left-arm forward motion. |
| `left-up` | Mirrored left-arm upward motion. |
| `left-outward` | Mirrored left-arm outward motion. |

One backend connection remains alive for the entire suite. This avoids requiring
XRoboToolkit to rediscover the Quest between traces. The backend connects while
the headset is off, then waits. At the one terminal prompt, type exactly:

```text
wearing the headset now
```

Press Enter and put on the headset. Ten seconds later, voice walks through every
needed segment with a preparation prompt and a precise recording prompt.
Calibration captures are checked for source readiness, tracking, stillness,
position separation, wrist-orientation change, and rate/integrity. Motion traces
add grip and translation checks.

The ten calibration captures form four independently checked position units.
Changing arm position gets the normal 7-second preparation window. Palm
subphases in the forward and horizontal units keep the same arm position and
get only a 2-second settling interval. Quality is decided only after an entire
position unit: if one horizontal subphase fails, all four horizontal subphases
are repeated together. The retry count applies separately to each position
unit. The richer orientation sets are intentionally placed in the forward and
horizontal poses, where Quest Pro controller camera recognition is expected to
be strongest. Opposed palm directions provide robust sign and change checks.

Passing static-anchor reports include representative median controller/head
positions and sign-aligned mean quaternions. These summaries are inputs to the
later calibration solve; the normalized and raw JSONL samples remain the
authoritative evidence and allow the transform to be recomputed.

Calibration and trace recording are distinct artifacts despite sharing one
headset handoff and one persistent XR connection:

- `calibration/units/` and
  `calibration/anchor-set.json` contain static observations used to solve a
  transform. The anchor set explicitly records that the transform is not yet
  solved.
- `traces/` and `trace-reports/` contain dynamic experiment trajectories. Each
  motion attempt records the SHA-256 identity of the immutable anchor set.

Motion recording never starts if any calibration position unit remains failed.
Rerun script 2 to retry only that unit; once all four pass, the same script
writes the anchor set and proceeds into the separate trace-recording phase.

After all needed segments, voice asks for both grips to be released, the
controllers to be put down, and the headset to be removed. After returning to
the terminal, type `headset off` to close the run boundary.

The suite and every attempt are immutable under
`calibration-suites/<suite-id>/`. Its `manifest.json` identifies the selected
passing trace for each segment. If some segments remain failed, run script 2
again: it resumes the current suite, skips passing position units and motion
traces, and collects only unresolved work with up to three new attempts each.
Use `--new-suite` only to deliberately collect the entire plan again.

## 3. Validate and replay recorded data

Script 2 already performs the quality checks needed to accept or retry each
capture while the headset is on. Script 3 independently rechecks checksums,
schema/order, duration/rate, source readiness, tracking, grip, and motion, saves
a validation report, and only then replays the trace through native MuJoCo. A
validation failure blocks replay.

```bash
suite="$(cat outputs/g1-vr-lab/current-session)/calibration-suites/current-suite"
suite="$(cat "${suite}")"
trace="$(jq -r '.segments["right-forward"].selected_trace' "${suite}/manifest.json")"
./lab/g1_vr/03_replay_trace.sh --trace "${trace}" --expect right
```

The trace is replayed at 60 Hz through the shared guard,
G1-29 retargeter, upstream LeRobot native MuJoCo robot, and a viewer. Telemetry
and logs are saved under `telemetry/`. Use `--headless` for automated regression,
or `--validate-only` for forensic revalidation without starting MuJoCo. Behavior
expectations are `any`, `idle`, `right`, `left`, `bimanual`, `buttons-head`, and
`reconnect`.

For a deterministic headset-free test of the same validation/replay path:

```bash
python tools/synthesize_g1_vr_trace.py /tmp/g1-vr-synthetic \
  --scenario bimanual --duration-s 4
./lab/g1_vr/03_replay_trace.sh \
  --trace /tmp/g1-vr-synthetic --expect bimanual --headless
```

Synthetic traces are development fixtures, not headset or operator evidence.

## 4. Live end-to-end control and Quest video

```bash
./lab/g1_vr/04_live_e2e.sh --duration-s 60
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
- `G1_VR_XRBT_PC_SERVICE_DIR`
- `G1_VR_LAB_OUTPUT_ROOT`
- `G1_VR_VOICE_RATE` (default `-35`)
- `G1_VR_DONNING_SECONDS` (default `10`; shorten only in automated tests)

If a backend fails while the headset is on, voice still instructs the operator
to release grips and remove the headset. Artifacts are retained; rerunning never
reinterprets a failed or partial run as a pass.
