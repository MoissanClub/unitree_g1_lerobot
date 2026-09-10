"""Native supported-arm G1-23 simulator on loopback DDS, without XR dependencies."""
from __future__ import annotations

import fcntl
import os
from pathlib import Path
import threading
import time

import mujoco
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.utils.crc import CRC

from .dds import patch_unitree_dds_config
from .motor_bench import DT, MotorPlant, mesh_directory
from ..robots.motor_configs import load_profile

IDENTITY_TOPIC = "rt/lerobot/g1_sim_identity"


def acquire_simulator_lock():
    path = Path(f"/tmp/lerobot-g1-sim-{os.getuid()}.lock")
    handle = path.open("a+")
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        handle.close()
        raise RuntimeError("Another local G1 simulator is running. Stop it before starting a different embodiment.") from exc
    return handle


class NativeG1Simulation:
    """250 Hz DDS/state updates, two 500 Hz physics substeps per step().

    Only arms are dynamic. Other real joint states are fixed at zero; unused
    transport slots remain zero. Incoming commands own PD and feedforward torque.
    When commands become stale, measured arm positions are latched and held with
    source-derived PD gains and exact-model gravity compensation (simulation only).
    """
    embodiment = "g1_23"

    def __init__(self, command_timeout_s=0.5):
        if not np.isfinite(command_timeout_s) or command_timeout_s <= 0:
            raise ValueError("command_timeout_s must be positive and finite")
        self._process_lock = acquire_simulator_lock()
        self.closed = False
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.thread = None
        self.failure = None
        self.subscriber = self.publisher = self.identity_publisher = None
        try:
            self.plant = MotorPlant(load_profile(self.embodiment), mesh_directory())
            self.indices = np.array([m["dds_index"] for m in self.plant.motors])
            self.command_timeout_s = command_timeout_s
            self.command = None
            self.command_time = 0.0
            self.hold = self.plant.data.qpos[self.plant.qadr].copy()
            self.holding = True
            self.steps = 0
            self.rejected_commands = 0
            self.received_commands = 0
            self.last_torque = np.zeros(len(self.indices))
            self.crc = CRC()
            patch_unitree_dds_config()
            ChannelFactoryInitialize(0, "lo")
            self.publisher = ChannelPublisher("rt/lowstate", LowState_)
            self.publisher.Init()
            self.identity_publisher = ChannelPublisher(IDENTITY_TOPIC, String_)
            self.identity_publisher.Init()
            self.subscriber = ChannelSubscriber("rt/lowcmd", LowCmd_)
            self.subscriber.Init(self.receive, 1)
        except BaseException:
            self.close()
            raise

    def receive(self, msg):
        fields = np.array([[msg.motor_cmd[i].q, msg.motor_cmd[i].dq, msg.motor_cmd[i].kp,
                            msg.motor_cmd[i].kd, msg.motor_cmd[i].tau, msg.motor_cmd[i].mode]
                           for i in self.indices], dtype=float)
        unused = (13, 14, 20, 21, 27, 28)
        invalid = (msg.crc != self.crc.Crc(msg) or not np.isfinite(fields).all()
                   or np.any(fields[:, 2:4] < 0)
                   or any(msg.motor_cmd[i].mode or msg.motor_cmd[i].kp or msg.motor_cmd[i].kd
                          or msg.motor_cmd[i].tau for i in unused))
        with self.lock:
            if invalid:
                self.rejected_commands += 1
                return
            self.command = fields
            self.command_time = time.monotonic()
            self.received_commands += 1

    def step(self, action=None):
        with self.lock:
            if self.closed:
                return
            p = self.plant
            stale = self.command is None or time.monotonic() - self.command_time > self.command_timeout_s
            if stale and not self.holding:
                self.hold = p.data.qpos[p.qadr].copy()
            self.holding = stale
            for _ in range(2):
                q, dq = p.data.qpos[p.qadr], p.data.qvel[p.vadr]
                if stale:
                    raw = p.kp * (self.hold - q) - p.kd * dq + p.gravity_torque()
                else:
                    command = self.command
                    target = np.clip(command[:, 0], p.ranges[:, 0], p.ranges[:, 1])
                    raw = command[:, 2] * (target - q) + command[:, 3] * (command[:, 1] - dq) + command[:, 4]
                    raw = np.where(command[:, 5] == 1, raw, 0.0)
                self.last_torque = np.clip(raw, -p.limits, p.limits)
                p.data.ctrl[p.aids] = self.last_torque
                mujoco.mj_step(p.model, p.data)
                if not np.isfinite(p.data.qpos).all() or np.any(p.data.warning.number):
                    raise RuntimeError(f"Native G1-23 physics warning: {p.data.warning.number}")
            self.steps += 1
            state = unitree_hg_msg_dds__LowState_()
            state.tick = self.steps & 0xffffffff
            state.imu_state.quaternion = [1.0, 0.0, 0.0, 0.0]
            for j, i in enumerate(self.indices):
                state.motor_state[i].mode = 1
                state.motor_state[i].q = float(p.data.qpos[p.qadr[j]])
                state.motor_state[i].dq = float(p.data.qvel[p.vadr[j]])
                state.motor_state[i].tau_est = float(self.last_torque[j])
            state.crc = self.crc.Crc(state)
            self.publisher.Write(state)
            if self.steps % 25 == 1:
                self.identity_publisher.Write(String_(data=self.embodiment))
            return {}, 0.0, False, False, {}

    def start(self):
        if self.thread is not None or self.closed:
            raise RuntimeError("Simulator can only be started once before close")
        def run():
            try:
                while not self.stop_event.is_set():
                    start = time.monotonic()
                    self.step()
                    self.stop_event.wait(max(0, 2 * DT - (time.monotonic() - start)))
            except BaseException as exc:
                self.failure = exc
                self.stop_event.set()
        self.thread = threading.Thread(target=run, name="g1-23-physics")
        self.thread.start()

    def reset(self):
        with self.lock:
            self.plant.reset()
            self.command = None
            self.hold = self.plant.data.qpos[self.plant.qadr].copy()
            self.holding = True

    def snapshot(self):
        with self.lock:
            return self.plant.data.qpos.copy()

    def close(self):
        if self.closed:
            return
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=3)
        for channel in (self.subscriber, self.publisher, self.identity_publisher):
            if channel is not None:
                channel.Close()
        self.closed = True
        self._process_lock.close()


def make_g1_23_simulation():
    env = NativeG1Simulation()
    return None, env
