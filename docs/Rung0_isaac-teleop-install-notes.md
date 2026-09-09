# Isaac Teleop — Install & First-Session Notes

Field notes from a working install, September 2026. Written for developers setting up
NVIDIA Isaac Teleop for XR teleoperation, with or without LeRobot.

**Verified on:** Ubuntu 24.04 · x86_64 · NVIDIA GPU · miniforge · `isaacteleop` 1.5.144rc1 ·
CloudXR Runtime 6.3.0

---

## 1. System requirements

| | Requirement |
|---|---|
| OS | Ubuntu **22.04 or 24.04** |
| Arch | **x86_64** (aarch64 only supports `retargeters-lite`) |
| GPU | NVIDIA, **CUDA 12.8+** |
| Driver | **580.95.05 or newer** |
| Python | 3.11 / 3.12 / 3.13 |

`isaacteleop` publishes **Linux wheels only**. There is no macOS or Windows path.

```bash
nvidia-smi                      # driver >= 580.95.05, CUDA >= 12.8
lsb_release -a && python3 -V
```

---

## 2. Install

### `uv` is not required

NVIDIA and LeRobot both document `uv`, but plain pip works. One flag differs:
`--prerelease=allow` (uv) is `--pre` (pip).

```bash
python3 -m venv ~/.venvs/isaacteleop && source ~/.venvs/isaacteleop/bin/activate

pip install 'isaacteleop[cloudxr,retargeters-lite]~=1.3.131' \
  --extra-index-url https://pypi.nvidia.com --pre
```

Use a venv. Ubuntu 24.04 enforces PEP 668, so installing into the system interpreter is
refused.

### ⚠️ Use `retargeters-lite`, not `retargeters`

The full `retargeters` extra pulls **`dex-retargeting`**, which has no distribution for
many environments and fails with:

```
Additionally, some packages in these conflicts have no matching distributions
available for your environment:
    dex-retargeting
ERROR: ResolutionImpossible
```

`retargeters-lite` is the scipy-based path, resolves on x86_64 **and** aarch64, and is
sufficient for XR-controller pose input. `dex-retargeting` is **hand/finger** retargeting —
you only need it for dexterous-hand mapping.

If it still won't resolve, check your interpreter (`platform.machine()`, `sys.version`):
`arm64`/`Darwin` means you're on the wrong machine entirely; Python 3.13 is the most common
Linux culprit — rebuild the venv on 3.12.

### Version pinning

Two doc sets disagree. Pick one and stay there:

| Source | Pin |
|---|---|
| LeRobot integration docs | `~=1.3.131` with `retargeters-lite` |
| Isaac Teleop quick start | `~=1.5.0` (or `~=1.0`) with `retargeters` |

**If you are heading for LeRobot, use LeRobot's pin** — that's the tested combination.

---

## 3. EULA

```bash
python -m isaacteleop.cloudxr.service run --accept-eula
```

### What it actually does

`isaacteleop/cloudxr/runtime.py::check_eula()` writes a marker file:

```
~/.cloudxr/run/eula_accepted          # contains the text "accepted"
```

That is the entire persistent effect — no download, no license server, no activation.
Every later launch short-circuits on `os.path.isfile(marker)`.
To reset: `rm ~/.cloudxr/run/eula_accepted`.
The licence itself is `deps/cloudxr/CLOUDXR_LICENSE` in the IsaacTeleop repo.

### Two gotchas

**`python -m isaacteleop.cloudxr` is deprecated** as of 1.5 (removed in 1.7). It warns and
forwards to `isaacteleop.cloudxr.service run`. LeRobot's README still uses the old form.

**It does not "accept and exit."** It accepts *and starts the service in the foreground*.
The command will not return. Subcommands: `run`, `start` (detached), `stop`, `status`, `logs`.

To write the marker without running anything:

```bash
mkdir -p ~/.cloudxr/run && echo accepted > ~/.cloudxr/run/eula_accepted
```

---

## 4. Network

```bash
sudo ufw allow 47998/udp
sudo ufw allow 49100,48322/tcp     # Quest / PICO
hostname -I                         # note the workstation IP
```

Apple Vision Pro uses a different set (`48010,48322/tcp`, `47998:48000,48005,48008,48012/udp`).

Headset and workstation must be IP-reachable. A dedicated WiFi 6 router is recommended.

---

## 5. Start the service

```bash
python -m isaacteleop.cloudxr.service run --accept-eula
```

Expected output:

```
Running Isaac Teleop 1.5.144rc1, CloudXR Runtime 6.3.0
CloudXR runtime:   running, log file: ~/.cloudxr/logs/cxr_server.<ts>.log
CloudXR WSS proxy: running, log file: ~/.cloudxr/logs/wss.<ts>.log
device profile:    Quest3  (NV_DEVICE_PROFILE)
web client:        https://nvidia.github.io/IsaacTeleop/client/release-1.5.x/
Activate CloudXR environment in another terminal: source ~/.cloudxr/run/cloudxr.env
Keep this terminal open, Ctrl+C to terminate.
```

Leave it running. Everything else goes in other terminals, after
`source ~/.cloudxr/run/cloudxr.env`.

### ⚠️ `device profile: Quest3` is a default, not a detection

```python
DEFAULT_DEVICE_PROFILE = "Quest3"
"""NV_DEVICE_PROFILE used when no env file, process env, or caller sets one."""
```

It never inspects your headset. Documented alternatives:

```
Quest3, auto-webrtc, auto-native, AppleVisionPro
```

For the **browser** client (WebRTC transport), `auto-webrtc` negotiates your headset's real
FOV, resolution and controller layout instead of assuming Quest 3's:

```bash
python -m isaacteleop.cloudxr.service run --cloudxr-device-profile auto-webrtc
```

### ⚠️ Use the printed web-client URL

The service prints a **version-matched** client (`.../client/release-1.5.x/`). The
`/client/main/` URL in the docs is the dev client and can drift from your runtime's protocol.

---

## 6. Headset

No app install — it's a browser client.

1. On the headset, browse to `https://<workstation-ip>:48322` and **accept the self-signed
   certificate**
2. Open the client URL the service printed
3. Set video codec to **H.264**
4. Enter XR and connect

---

## 7. Verify without a robot

```bash
source ~/.cloudxr/run/cloudxr.env
python examples/teleop/python/gripper_retargeting_example_simple.py
```

Reads XR controller input over CloudXR and prints grip values. No robot required.

### Acceptance criteria

- [ ] Web client loads, certificate accepted
- [ ] Session stays up **5+ minutes** without dropping
- [ ] **Both** controllers tracked simultaneously
- [ ] Squeeze and trigger sweep cleanly 0 → 1
- [ ] Pose values continuous — no jumps, no NaN
- [ ] Tracking survives turning your torso and reaching to full extension

The last one matters most. Inside-out tracking drops controllers held low, behind you, or
at full extension — exactly the reach envelope teleoperation uses.

---

## 8. Headset compatibility

LeRobot's example README names: **Quest 3, Pico 4, Apple Vision Pro**.
Isaac Teleop's quick start names: **Meta Quest, PICO, Apple Vision Pro**.

Neither enumerates models. **PICO is the best-supported option** — PICO and NVIDIA
co-launched Isaac Teleop, and NVIDIA's own G1 walkthrough demos a PICO.

**Older Quest hardware is a risk.** There is a developer-forum report of a Quest 3S failing
with `XR_ERROR_FORM_FACTOR_UNSUPPORTED` and no tracked devices. If you see that:

1. Try `--cloudxr-device-profile auto-webrtc` first
2. If that fails, it is a runtime/form-factor mismatch, not a config problem — swap headsets

> **Not yet verified in these notes:** Meta Quest Pro. It is not named in either doc set.
> Treat it as unproven and have a Quest 3 or PICO 4 as fallback.

---

## Quick reference

```bash
# install
pip install 'isaacteleop[cloudxr,retargeters-lite]~=1.3.131' \
  --extra-index-url https://pypi.nvidia.com --pre

# firewall
sudo ufw allow 47998/udp && sudo ufw allow 49100,48322/tcp

# service (terminal 1, leave open)
python -m isaacteleop.cloudxr.service run --accept-eula

# work (terminal 2)
source ~/.cloudxr/run/cloudxr.env
python examples/teleop/python/gripper_retargeting_example_simple.py

# headset: accept cert at https://<ip>:48322, open printed client URL, codec H.264
```

## Known issues

| Symptom | Cause | Fix |
|---|---|---|
| `ResolutionImpossible: dex-retargeting` | full `retargeters` extra | use `retargeters-lite` |
| `uv: command not found` | uv not installed | use pip; `--pre` replaces `--prerelease=allow` |
| Deprecation warning on `isaacteleop.cloudxr` | moved in 1.5 | `isaacteleop.cloudxr.service run` |
| Command never returns | it runs the service, not just the EULA | expected; use another terminal |
| Hangs on a headless box | EULA prompts on stdin | write the marker manually |
| Client won't connect | cert not accepted / firewall | `https://<ip>:48322`, check ufw |
| `XR_ERROR_FORM_FACTOR_UNSUPPORTED` | headset not supported by this runtime | `auto-webrtc`, else swap headsets |
