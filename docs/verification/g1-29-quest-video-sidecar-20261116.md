# G1-29 Quest video sidecar automated gate — 2026-11-16

This is automated, headset-free evidence for Commit 11. It does not claim that
the Quest decoded a recognizable MuJoCo image.

The tracked `tools/quest_video_sender.py` reconciles the previously exercised
local snapshot at
`/home/dwei/meta_quest_xrobotoolkit/mujoco-quest-video-sender/quest_video_sender.py`.
It implements XRoboToolkit Quest `OPEN_CAMERA` parsing and four-byte
big-endian-length-prefixed H.264 access units. The process is optional and
outside the XR command loop.

Automated command:

```bash
PYTHONPATH=/home/dwei/lerobot-sim/lerobot_teleoperator_g1_vr/src:\
/home/dwei/lerobot-sim/unitree_g1_lerobot/.vr-sim/clean-install-verification/lerobot/src:. \
/home/dwei/lerobot-sim/unitree_g1_lerobot/.vr-sim/clean-install-verification/env/bin/python \
  -m pytest -q tests/xr/test_quest_video.py tests/xr/test_supervised_g1_29.py
```

Observed result: `10 passed`. This included a real bounded sender process that
bound a loopback control port, reached readiness, and stopped normally. Protocol
tests used mock sockets/data and did not start FFmpeg or require X11, NVENC, a
Quest, or a decoder.

The live simulation-lab gate remains pending: start the sidecar on the simulator
desktop, request video from the Quest, verify a recognizable updating image,
exercise both arms, interrupt video, and confirm control hold/shutdown remains
available. Preserve `sender.log` and `metadata.json` from the reported evidence
directory.
