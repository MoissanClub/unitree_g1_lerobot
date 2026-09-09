# Pinned Motor Sources

These files are unmodified reference data, not executable dependencies of the comparison.
`motor_configs.py` parses the controller as Python AST; it never imports or executes it.

| Files | Source revision | License |
|---|---|---|
| `g1_29dof.xml`, `g1_23dof.xml` | `unitreerobotics/unitree_mujoco` at `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d` | BSD-3-Clause, see `LICENSE.unitree_mujoco` |
| `robot_arm.py.txt` | `unitreerobotics/xr_teleoperate` at `817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b`, `teleop/robot_control/robot_arm.py` | Apache-2.0, see `LICENSE.xr_teleoperate` and source attribution |

Original locations:

- https://github.com/unitreerobotics/unitree_mujoco/tree/1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d/unitree_robots/g1
- https://github.com/unitreerobotics/xr_teleoperate/blob/817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b/teleop/robot_control/robot_arm.py

Generated profiles record source SHA-256 digests. Meshes are reused from
`lerobot/unitree-g1-mujoco` revision `a38dc8617f0fca51b38e9354dc58ee35ad850fb5`.
The benchmark builds temporary in-memory supported-arm scenes from these XML files;
the source models, existing URDF/IK viewers, and live DDS simulator remain unchanged.

The six detached dummy joints in the source G1-23 XML are transport placeholders and
are excluded from its real motor profile. The profile retains the sparse DDS indices
of the 23 physical joints; the arm benchmark uses only the ten real arm actuators.
