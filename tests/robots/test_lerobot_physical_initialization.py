"""Real LeRobot constructor acceptance, without connecting to any transport.

Opt in with LEROBOT_ROOT pointing to the audited, patched checkout. The dedicated
launcher treats absent dependencies or an incompatible checkout as failures.
"""
from contextlib import ExitStack
import gc
import importlib
import json
import os
from pathlib import Path
import socket
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

from unitree_g1_lerobot.diagnostics.physical.physical_preflight import ROOT, audit_checkout


class PhysicalInitializationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        checkout = os.environ.get('LEROBOT_ROOT')
        if not checkout:
            raise unittest.SkipTest('Run verify_g1_lerobot_initialization.sh with LEROBOT_ROOT')
        checkout = Path(checkout).expanduser().resolve()
        audit = audit_checkout(checkout)
        if not audit['matches_audited_files'] or not audit['patch_matches_audit']:
            raise RuntimeError('Checkout must match configs/physical_preflight_audit.json; apply the project patch')
        sys.path.insert(0, str(checkout / 'src'))
        from unitree_g1_lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config
        cls.robot_class, cls.config_class = UnitreeG1, UnitreeG1Config
        cls.module = importlib.import_module(UnitreeG1.__module__)
        if Path(cls.module.__file__).resolve() != checkout / 'src/lerobot/robots/unitree_g1/unitree_g1.py':
            raise RuntimeError('Imported LeRobot does not come from the requested checkout')
        cls.transport = importlib.import_module('lerobot.robots.unitree_g1.unitree_sdk2_socket')

    def construct_and_check(self, embodiment, round_trip=False):
        import cyclonedds.domain
        import draccus
        import unitree_sdk2py.core.channel as sdk
        import zmq

        forbidden = []
        with tempfile.TemporaryDirectory(prefix='g1-init-calibration-') as calibration, ExitStack() as stack:
            def deny(owner, name):
                mock = stack.enter_context(patch.object(owner, name,
                    side_effect=AssertionError(f'Initialization must not call {name}')))
                forbidden.append(mock)
                return mock

            # Patch side-effect boundaries, never the constructor, config, feature logic,
            # package availability checks, or backend-selection implementation under test.
            for name in ('connect', 'disconnect', 'reset', 'send_action', 'publish_lowcmd'):
                deny(self.robot_class, name)
            for owner in (sdk, self.transport):
                for name in ('ChannelFactoryInitialize', 'ChannelPublisher', 'ChannelSubscriber'):
                    deny(owner, name)
            for name in ('_SDKChannelFactoryInitialize', '_SDKChannelPublisher', '_SDKChannelSubscriber'):
                deny(self.module, name)
            deny(threading.Thread, 'start')
            deny(socket.socket, 'connect')
            deny(socket.socket, 'connect_ex')
            deny(socket.socket, 'bind')
            deny(socket.socket, 'sendto')
            deny(zmq.Context, 'socket')
            deny(cyclonedds.domain.Domain, '__init__')
            deny(cyclonedds.domain.DomainParticipant, '__init__')

            config = self.config_class(embodiment=embodiment, is_simulation=False,
                controller=None, gravity_compensation=False, cameras={},
                id='physical-initialization-test', calibration_dir=Path(calibration))
            if round_trip:
                config = draccus.decode(self.config_class, draccus.encode(config))
                from lerobot.robots.utils import make_robot_from_config
                robot = make_robot_from_config(config)
            else:
                robot = self.robot_class(config)
            self.assertIs(type(robot), self.robot_class)
            self.assertFalse(robot.config.is_simulation)
            self.assertEqual(robot.embodiment.name, embodiment)
            self.assertIs(robot._ChannelFactoryInitialize, self.transport.ChannelFactoryInitialize)
            self.assertIs(robot._ChannelPublisher, self.transport.ChannelPublisher)
            self.assertIs(robot._ChannelSubscriber, self.transport.ChannelSubscriber)
            self.assertFalse(robot.is_connected)
            self.assertIsNone(robot.controller)
            self.assertIsNone(robot.arm_ik)
            self.assertIsNone(robot.subscribe_thread)
            self.assertIsNone(robot._controller_thread)
            self.assertIsNone(robot.sim_env)
            self.assertEqual(robot.cameras, {})
            self.assertEqual(robot.get_observation(), {})
            for name in ('_ctx', '_lowcmd_sock', '_lowstate_sock'):
                self.assertIsNone(getattr(self.transport, name))

            profile = json.loads((ROOT / 'configs/motors' / f'{embodiment}.json').read_text())
            expected = {'k' + ''.join(part.title() for part in motor['joint'].removesuffix('_joint').split('_')):
                        motor['dds_index'] for motor in profile['motors']}
            self.assertEqual({joint.name: joint.value for joint in robot.joint_index}, expected)
            self.assertEqual(set(robot.action_features), {f'{name}.q' for name in expected})
            self.assertEqual({j.value for j in robot.arm_index},
                             {m['dds_index'] for m in profile['motors'] if m['arm']})
            self.assertEqual(robot.observation_features, robot.action_features)
            self.assertEqual(robot.embodiment.hardware_supported, embodiment == 'g1_29')
            self.assertEqual(len(config.kp), 29)
            self.assertEqual(len(config.kd), 29)
            for slot in set(range(29)) - set(expected.values()):
                self.assertEqual((config.kp[slot], config.kd[slot], config.default_positions[slot]), (0, 0, 0))
            # The base Robot destructor may call disconnect if it thinks it is connected.
            # Destroy the object while all guards remain active; don't call disconnect.
            del robot
            gc.collect()
            for mock in forbidden:
                mock.assert_not_called()

    def test_g1_29_physical_constructor(self):
        self.construct_and_check('g1_29')

    def test_g1_23_physical_constructor_preserves_hardware_guard(self):
        self.construct_and_check('g1_23')

    def test_physical_config_round_trip_and_robot_factory(self):
        self.construct_and_check('g1_29', round_trip=True)


if __name__ == '__main__':
    unittest.main()
