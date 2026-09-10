"""Loopback DDS support for the existing LeRobot G1 simulation path."""
from __future__ import annotations

import threading
import time

import numpy as np
import unitree_sdk2py.core.channel as unitree_channel


def patch_unitree_dds_config() -> None:
    unitree_channel.ChannelConfigHasInterface = """<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
    <Domain Id="any">
        <General>
            <Interfaces>
                <NetworkInterface name="$__IF_NAME__$" priority="default" multicast="default"/>
            </Interfaces>
        </General>
    </Domain>
</CycloneDDS>"""


def connect_unitree_g1_external_dds(robot, g1_module, joint_index: type, timeout_s: float) -> None:
    robot._ChannelFactoryInitialize(0, "lo")
    verify_simulator_identity(getattr(robot.config, "embodiment", "g1_29"), timeout_s)
    robot.lowcmd_publisher = robot._ChannelPublisher(g1_module.kTopicLowCommand_Debug, g1_module.hg_LowCmd)
    robot.lowcmd_publisher.Init()
    robot.lowstate_subscriber = robot._ChannelSubscriber(g1_module.kTopicLowState, g1_module.hg_LowState)
    robot.lowstate_subscriber.Init()

    robot.subscribe_thread = threading.Thread(target=robot._subscribe_lowstate, daemon=True)
    robot.subscribe_thread.start()

    for cam in robot._cameras.values():
        if not cam.is_connected:
            cam.connect()

    robot.crc = g1_module.CRC()
    robot.msg = g1_module.unitree_hg_msg_dds__LowCmd_()
    robot.msg.mode_pr = 0

    lowstate = None
    deadline = time.time() + timeout_s
    while lowstate is None:
        with robot._lowstate_lock:
            lowstate = robot._lowstate
        if lowstate is None:
            if time.time() > deadline:
                robot._shutdown_event.set()
                raise TimeoutError(
                    f"Timed out waiting for external G1 MuJoCo DDS lowstate ({timeout_s:.1f}s). "
                    "Start process A first with ./run_g1_mujoco_dds_sim.sh."
                )
            time.sleep(0.01)

    robot.msg.mode_machine = lowstate.mode_machine
    robot.kp = np.array(robot.config.kp, dtype=np.float32)
    robot.kd = np.array(robot.config.kd, dtype=np.float32)
    for joint in joint_index:
        robot.msg.motor_cmd[joint].mode = 1
        robot.msg.motor_cmd[joint].kp = robot.kp[joint.value]
        robot.msg.motor_cmd[joint].kd = robot.kd[joint.value]
        robot.msg.motor_cmd[joint].q = lowstate.motor_state[joint.value].q


def verify_simulator_identity(embodiment: str, timeout_s: float):
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
    observed = []
    ready = threading.Event()

    def receive(msg):
        observed.append(msg.data)
        ready.set()

    subscriber = unitree_channel.ChannelSubscriber("rt/lerobot/g1_sim_identity", String_)
    subscriber.Init(receive, 1)
    try:
        # Older G1-29 simulators do not announce identity. G1-23 must do so.
        ready.wait(timeout_s if embodiment == "g1_23" else min(timeout_s, 0.5))
        if observed and observed[-1] != embodiment:
            raise ValueError(f"Simulator embodiment mismatch: requested {embodiment}, running {observed[-1]}")
        if not observed and embodiment == "g1_23":
            raise TimeoutError("No native G1-23 simulator identity received; start --embodiment g1_23 first")
    finally:
        subscriber.Close()


def start_simulator_identity(embodiment: str):
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
    publisher = unitree_channel.ChannelPublisher("rt/lerobot/g1_sim_identity", String_)
    publisher.Init()
    stopped = threading.Event()

    def publish():
        while not stopped.is_set():
            publisher.Write(String_(data=embodiment))
            stopped.wait(0.1)

    thread = threading.Thread(target=publish, name="g1-simulator-identity")
    thread.start()

    def close():
        stopped.set()
        thread.join(timeout=2)
        publisher.Close()
    return close
