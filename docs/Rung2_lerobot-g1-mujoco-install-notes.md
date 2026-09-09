# LeRobot × Unitree G1 in MuJoCo — Install Notes

Field notes from getting LeRobot's Unitree G1 **simulation** path running end to end,
September 2026. No robot hardware required — but as of this writing the documented
install does not work, and this records every workaround needed.

**Verified on:** Ubuntu 24.04 (glibc 2.39) · x86_64 · miniforge · Python 3.12 ·
LeRobot 0.6.2 · pinocchio 3.9.0 (conda-forge) · casadi 3.7.2 · mujoco 3.6.0

> **Summary:** seven separate blockers between `pip install` and a stepping simulator.
> Six are packaging or documentation defects; one is a hard crash in Unitree's SDK on
> Ubuntu 24.04.

---

## 0. TL;DR — the working sequence

```bash
# --- environment ---
conda create -n lerobot python=3.12 -y && conda activate lerobot
conda install -c conda-forge "pinocchio>=3.0.0,<4.0.0"      # MUST be conda-forge

# --- LeRobot (NOTE: no [kinematics] — see §2) ---
git clone https://github.com/huggingface/lerobot.git && cd lerobot
pip install -e ".[dataset]" "huggingface_hub>=1.5"
pip install casadi mujoco loguru

# --- CycloneDDS C library from source (see §4) ---
git clone -b 0.10.2 --depth 1 https://github.com/eclipse-cyclonedds/cyclonedds.git ~/cyclonedds
cmake -S ~/cyclonedds -B ~/cyclonedds/build \
      -DCMAKE_INSTALL_PREFIX=$HOME/.local/cyclonedds \
      -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF
cmake --build ~/cyclonedds/build --target install -j$(nproc)
conda env config vars set CYCLONEDDS_HOME=$HOME/.local/cyclonedds
conda deactivate && conda activate lerobot

# --- Unitree SDK (see §3) ---
pip install "unitree_sdk2py @ git+https://github.com/unitreerobotics/unitree_sdk2_python.git"

# --- verify ---
python -c "
import pinocchio; print('pin', pinocchio.__version__)          # expect 3.9.x
from pinocchio import casadi as cpin; print('cpin OK')
import casadi, mujoco, loguru, lerobot, unitree_sdk2py; print('all OK')"
```

Then apply the **`<Tracing>` patch (§5)** in your script, and run from a **real X session
or under `xvfb-run` (§7)**.

---

## 1. Pinocchio must come from conda-forge

`G1_29_ArmIK` does `from pinocchio import casadi as cpin`. **PyPI's `pin` package has no
CasADi bindings** — empirically verified:

```
pinocchio 4.1.0
ImportError: cannot import name 'casadi' from 'pinocchio'
```

conda-forge's `pinocchio` 3.x does. LeRobot's docs say this; it is not optional.

---

## 2. ⚠️ Do NOT install `lerobot[kinematics]`

`kinematics = ["lerobot[placo-dep]"]` → `placo>=0.9.6,<0.9.16` + **PyPI `pin`**. pip then
tries to replace your conda Pinocchio and dies:

```
Attempting uninstall: pin
  Found existing installation: pin 3.9.0
error: uninstall-no-record-file
× Cannot uninstall pin 3.9.0
╰─> The package's contents are unknown: no RECORD file was found for pin.
```

If it *succeeds* instead of erroring, that is worse — you now have PyPI `pin` 4.x, no
CasADi bindings, and a violated `<4.0.0` constraint.

**You do not need it.** `kinematics` is the **Placo** solver used by the SO-101 Cartesian
path. The G1's IK uses conda Pinocchio + CasADi directly and never touches Placo.

> **Upstream defect:** `lerobot[kinematics]` is incompatible with the conda Pinocchio that
> LeRobot's own G1 documentation requires. Following both doc pages is impossible.
> Consequence: the G1 path and the SO-101 XR path cannot share one environment.

---

## 3. `unitree_sdk2_python` is not on PyPI

LeRobot's docs say `pip install unitree_sdk2_python==1.0.1`. That distribution does not
exist on any index. The package is named **`unitree_sdk2py`** (version 1.0.1 in the repo's
`setup.py`) and is published only on GitHub:

```bash
pip install "unitree_sdk2py @ git+https://github.com/unitreerobotics/unitree_sdk2_python.git"
```

---

## 4. CycloneDDS must be built from source

`unitree_sdk2py` pins `cyclonedds==0.10.2` exactly. That Python binding compiles against a
native CycloneDDS C library at build time:

```
Could not locate cyclonedds. Try to set CYCLONEDDS_HOME or CMAKE_PREFIX_PATH
ERROR: Failed to build 'cyclonedds' when getting requirements to build wheel
```

**conda-forge does not have it** (`PackagesNotFoundInChannelsError`). Build it:

```bash
git clone -b 0.10.2 --depth 1 https://github.com/eclipse-cyclonedds/cyclonedds.git ~/cyclonedds
cmake -S ~/cyclonedds -B ~/cyclonedds/build \
      -DCMAKE_INSTALL_PREFIX=$HOME/.local/cyclonedds \
      -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF
cmake --build ~/cyclonedds/build --target install -j$(nproc)
```

- `cmake_minimum_required(VERSION 3.16)` — CMake 4.x is fine
- **`CYCLONEDDS_HOME` points at the install prefix** (`~/.local/cyclonedds`), *never* the
  source tree. This is the most common mistake.
- Match the C library tag to the binding version. A skew builds cleanly and then segfaults.
- Keep the install prefix **outside** the conda env so `conda env remove` doesn't take it.
- If `import unitree_sdk2py` can't find `libddsc.so`, add
  `LD_LIBRARY_PATH=$HOME/.local/cyclonedds/lib`.
- Check nothing else wins the loader race (`ldconfig -p | grep ddsc`) — ROS 2 ships its own.

---

## 5. 🔴 Unitree SDK crashes on Ubuntu 24.04

```
*** buffer overflow detected ***: terminated
Aborted (core dumped)
```

Triggered by `ChannelFactoryInitialize(0, "lo")`, which LeRobot calls from
`UnitreeG1.connect()` **even in simulation mode**.

### Root cause

Not cyclonedds — plain `Domain(0)` and `Domain(0, <minimal XML>)` both work. The crash is
in Unitree's config template (`unitree_sdk2py/core/channel_config.py`), which enables
tracing by default:

```xml
<Tracing>
    <Verbosity>config</Verbosity>
    <OutputFile>/tmp/cdds.LOG</OutputFile>
</Tracing>
```

At `config` verbosity CycloneDDS dumps its entire resolved configuration through the trace
formatter — a path glibc 2.39's `_FORTIFY_SOURCE=3` flags as an overflow.

### Workaround

`channel.py` does `from .channel_config import ChannelConfigHasInterface`, so the name is
bound in **`channel`'s** namespace. Patch it there, before anything constructs the robot:

```python
import unitree_sdk2py.core.channel as _ch
_ch.ChannelConfigHasInterface = '''<?xml version="1.0" encoding="UTF-8" ?>
    <CycloneDDS>
        <Domain Id="any">
            <General>
                <Interfaces>
                    <NetworkInterface name="$__IF_NAME__$" priority="default" multicast="default"/>
                </Interfaces>
            </General>
        </Domain>
    </CycloneDDS>'''
```

Verify:

```bash
python -c "
import unitree_sdk2py.core.channel as ch
ch.ChannelConfigHasInterface = ch.ChannelConfigHasInterface.split('<Tracing>')[0] + '</Domain></CycloneDDS>'
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
ChannelFactoryInitialize(0, 'lo'); print('DDS OK')"
```

> **Upstream defect (Unitree, not LeRobot):** the shipped DDS config makes the G1 Python SDK
> abort on the current Ubuntu LTS. The `<Tracing>` block also writes an unrequested
> `/tmp/cdds.LOG` on every run. Removing it fixes the crash.

---

## 6. Hub env has undeclared dependencies

`make_env("lerobot/unitree-g1-mujoco", trust_remote_code=True)` downloads code that imports
packages nothing declares:

```
ModuleNotFoundError: Hub env 'lerobot/unitree-g1-mujoco:env.py' failed to import
because the dependency 'loguru' is not installed locally.
```

```bash
pip install loguru
```

Find the rest before hitting them one at a time:

```bash
grep -rhoE "^[[:space:]]*(from|import) [a-zA-Z_][a-zA-Z0-9_]*" \
  ~/.cache/huggingface/hub/models--lerobot--unitree-g1-mujoco/snapshots/*/ --include=*.py \
  | awk '{print $2}' | cut -d. -f1 | sort -u \
  | while read m; do python -c "import $m" 2>/dev/null || echo "MISSING: $m"; done
```

`rclpy` is expected to be missing and degrades gracefully
(*"Camera image publishing will be disabled"*). Do not install ROS 2 for it.

---

## 7. The sim needs a real display

The env initializes `head_camera` and opens a **GLFW** window unconditionally. Over SSH:

```
GLFWError: (65550) b'X11: The DISPLAY environment variable is missing'
ERROR: could not initialize GLFW
```

- **On the machine's own X session:** works as-is.
- **Over SSH with a local session present:** `export DISPLAY=:0`
- **Genuinely headless / unattended / CI:** `xvfb-run -a python your_script.py`

`MUJOCO_GL=egl` may not help — GLFW needs an X server specifically, independent of MuJoCo's
render backend.

**Plan for this**: any scheduled or automated run needs `xvfb-run`.

---

## 8. API notes for anyone writing against this

### `G1_29_ArmIK` ships stale Pinocchio data

`__init__` calls `buildReducedRobot()` (creating `reduced_robot.data`) and only *then*
`addFrame("L_ee")` / `addFrame("R_ee")`. The shipped `data.oMf` is shorter than
`model.nframes`, so forward kinematics on the EE frames raises `IndexError` and then
segfaults on teardown.

```python
m = ik.reduced_robot.model
d = m.createData()              # NOT ik.reduced_robot.data
ik.reduced_robot.data = d
```

`exo_ik.py` silently does the same thing. One-line upstream fix: call `createData()` after
the two `addFrame()` calls.

### Joint ordering

`G1_29_ArmIK` exposes `_arm_reorder_g1_to_pin` / `_arm_reorder_pin_to_g1`. **Measured: the
two orders are identical** for this URDF, so the mappings are currently no-ops. Apply them
anyway — they document the assumption and protect against a URDF change.

### Action / observation shape

With `controller=None`, the action space is **all 29 joints**, keyed `f"{JointName}.q"`:

```python
action = {f"{j.name}.q": 0.0 for j in G1_29_JointIndex}     # legs + waist
action.update({f"{j.name}.q": float(q[k]) for k, j in enumerate(G1_29_JointArmIndex)})
robot.send_action(action)
```

`get_observation()` returns ~87 keys (29 joints × q/dq/tau).

### The smoothing filter lags 2 steps, not 1

`WeightedMovingFilter` computes `np.array(self._data_queue).T @ self._weights`. The deque
is oldest-first, so with weights `[0.4, 0.3, 0.2, 0.1]` the **oldest** sample gets 0.4 and
the **newest** gets 0.1. Measured group delay: **2.0 steps** (1.0 if reversed).

At a 30 Hz teleop loop that is 67 ms of latency where 33 ms would do. Budget for it, or
reverse the weights in your own fork.

### Residuals are soft, not exact

The IK is a weighted least-squares problem — the only hard constraint is joint limits; pose
lives entirely in the objective:

```python
opti.minimize(50*translational_cost + 1*rotation_cost
              + 0.02*regularization_cost + 0.1*smooth_cost)
```

So `solve_ik` **always returns a solution**, never reports unreachability, and a static
target can settle at a non-zero residual (~2 mm / 1° observed) where translation and
rotation objectives trade off. If you need a reachability signal, threshold it yourself:

```python
e_t = np.linalg.norm(np.asarray(ik.translational_error(q, L, R)).ravel())
e_r = np.linalg.norm(np.asarray(ik.rotational_error(q, L, R)).ravel())
```

### IK failure path contaminates the next solve

On convergence failure `solve_ik` returns `(current_q, zeros)` — safe — but assigns the
unconverged iterate to `self.init_data` **first**, seeding the next warm start. It also
returns zero feed-forward torque, dropping gravity compensation for that cycle. Intermittent
failures produce droop-and-recover chatter. Present in both LeRobot and `xr_teleoperate`.

---

## Issue checklist for maintainers

| # | Where | Issue |
|---|---|---|
| 1 | LeRobot | `lerobot[kinematics]` conflicts with the conda Pinocchio the G1 docs require |
| 2 | LeRobot | docs reference `unitree_sdk2_python` on PyPI; no such distribution |
| 3 | LeRobot | cyclonedds C-library prerequisite undocumented |
| 4 | LeRobot | `G1_29_ArmIK` ships `reduced_robot.data` stale w.r.t. its own added frames |
| 5 | LeRobot | `WeightedMovingFilter` applies weights oldest-first — 2× intended group delay |
| 6 | LeRobot | IK failure path poisons the warm start and drops gravity compensation |
| 7 | LeRobot Hub | `lerobot/unitree-g1-mujoco` has undeclared deps (`loguru`) |
| 8 | **Unitree** | `<Tracing>` in the default DDS config aborts on glibc 2.39 / Ubuntu 24.04 |

Net effect of 1–3, 7 and 8: **LeRobot's G1 simulation path cannot be run by following the
documentation** — which blocks every would-be contributor who does not own the hardware.
