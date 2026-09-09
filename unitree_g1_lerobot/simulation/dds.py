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
