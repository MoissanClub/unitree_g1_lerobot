# G1-29 VR baseline before XRoboToolkit integration — 2026-11-16

This is the unchanged simulation baseline for the
`integration/g1-29-xrobotoolkit` development series. It establishes the existing
G1-29 operator behavior before a second XR input source is introduced. It is not
XRoboToolkit, live CloudXR, headset, or physical-robot evidence.

## Candidate

- Operator repository revision:
  `e9891610f0779b7567e2e2637d53c8fb3a0f4d25`
- Pinned LeRobot revision:
  `d5e400bcefeccc93ba956ce876530e5283512df1`
- Compatibility contract:
  [`configs/g1_29_vr_compatibility.json`](../../configs/g1_29_vr_compatibility.json)
- G1-29 URDF SHA-256:
  `8bbf006633fc50b616f665c7a970780cc296577a0adfd7d28b049e751c238735`

The local `.vr-sim.json` selected the clean-install verification interpreter,
LeRobot checkout and assets. The compatibility manifest records the observed
Python and key package versions.

## Command

```bash
./run_g1_vr_sim.sh --embodiment g1_29 --headless
```

## Observation

The bounded synthetic replay completed successfully:

```text
PASS g1_29: 100 frames, motion=0.2631rad, 54 distinct camera frames
Session stopped.
```

Per-service logs were written locally to:

```text
/home/dwei/lerobot-sim/unitree_g1_lerobot/outputs/vr-sim-20261116-210823-helckq9z
```

The simulator reported readiness with gravity and measured-pose gravity
compensation enabled. CloudXR explicitly ran as a replay-only no-op. The bridge
reported the pass above, and the launcher completed ordered shutdown without a
reported forced or nonzero child exit.

## Regression gate

The opt-in process test now checks the manifest against the installed LeRobot
revision, interpreter contract and G1-29 URDF. It verifies that the installed
kinematics exposes 14 named arm actions, then requires the 100-frame replay to
produce bounded motion, multiple distinct nonblank camera frames, a pass summary
and clean shutdown without the listed fatal log fragments.

This gate deliberately does not require X, a headset, live OpenXR/CloudXR,
XRoboToolkit, network access, DDS or physical hardware.
