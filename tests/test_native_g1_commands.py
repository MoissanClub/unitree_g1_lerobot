"""Validate DDS command envelopes without opening a DDS participant."""
import threading
import unittest

import numpy as np
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

from unitree_g1_lerobot.simulation.native_g1 import NativeG1Simulation


class NativeCommandTests(unittest.TestCase):
    def test_invalid_crc_nonfinite_and_wrong_embodiment_commands_are_rejected(self):
        env = NativeG1Simulation.__new__(NativeG1Simulation)
        env.indices = np.array([15,16,17,18,19,22,23,24,25,26])
        env.lock, env.crc = threading.RLock(), CRC()
        env.rejected_commands = env.received_commands = 0
        msg = unitree_hg_msg_dds__LowCmd_()
        for i in env.indices:
            msg.motor_cmd[i].mode = 1
            msg.motor_cmd[i].kp = 80
            msg.motor_cmd[i].kd = 3
        msg.crc = env.crc.Crc(msg)
        env.receive(msg)
        self.assertEqual(env.received_commands, 1)
        original = env.command.copy()
        msg.crc ^= 1
        env.receive(msg)
        msg.motor_cmd[20].mode = 1
        msg.crc = env.crc.Crc(msg)
        env.receive(msg)
        msg.motor_cmd[20].mode = 0
        msg.motor_cmd[15].q = float("nan")
        msg.crc = env.crc.Crc(msg)
        env.receive(msg)
        self.assertEqual(env.rejected_commands, 3)
        self.assertEqual(env.received_commands, 1)
        np.testing.assert_array_equal(env.command, original)


if __name__ == "__main__":
    unittest.main()
