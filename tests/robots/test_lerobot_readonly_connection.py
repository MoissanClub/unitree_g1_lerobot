"""Exercise real read-only connect/disconnect with a fake DDS receiver only."""
from contextlib import ExitStack
import tempfile
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

from . import test_lerobot_physical_initialization as initialization


class ReadOnlyConnectionTests(initialization.PhysicalInitializationTests):
    # Reuse audited source loading; inherited construction checks also cover regressions.
    def message(self, tick=1):
        return NS(tick=tick, mode_machine=5, mode_pr=0,
                  motor_state=[NS(q=i/100, dq=0., tau_est=0.) for i in range(35)],
                  imu_state=NS(gyroscope=[], accelerometer=[], quaternion=[], rpy=[]),
                  wireless_remote=[])

    def fixture(self, stack, message=True, fail_init=False):
        tmp = stack.enter_context(tempfile.TemporaryDirectory())
        config = self.config_class(embodiment='g1_29', is_simulation=False, read_only=True,
                    network_interface='test0', state_timeout_s=.01, max_state_age_s=.1,
                    calibration_dir=Path(tmp))
        robot = self.robot_class(config)
        stack.callback(robot.disconnect)
        stack.enter_context(patch('socket.if_nametoindex', return_value=1))
        factory = stack.enter_context(patch.object(robot, '_ChannelFactoryInitialize'))
        writer = stack.enter_context(patch.object(robot, '_ChannelPublisher',
                                   side_effect=AssertionError('Forbidden writer')))
        subscriber = stack.enter_context(patch.object(robot, '_ChannelSubscriber')).return_value
        def init(callback):
            self.callback = callback
            if fail_init:
                raise RuntimeError('injected receiver failure')
            if message:
                callback(self.message())
        subscriber.Init.side_effect = init
        return robot, subscriber, writer, factory

    def test_connect_observe_disconnect_without_writer(self):
        with ExitStack() as stack:
            robot, subscriber, writer, factory = self.fixture(stack)
            robot.connect()
            self.assertTrue(robot.is_connected)
            self.assertEqual(robot.get_observation()['kRightShoulderPitch.q'], .22)
            factory.assert_called_once_with(0, 'test0')
            for call in (lambda: robot.send_action({}), lambda: robot.publish_lowcmd({}), robot.reset):
                with self.assertRaises(RuntimeError):
                    call()
            robot.disconnect()
            robot.disconnect()
            subscriber.Close.assert_called_once()
            self.assertFalse(robot.is_connected)
            writer.assert_not_called()
            with self.assertRaises(RuntimeError):
                robot.connect()

    def test_timeout_and_partial_init_close_subscriber(self):
        for message, fail_init, error in [(False, False, TimeoutError), (False, True, RuntimeError)]:
            with self.subTest(fail_init=fail_init), ExitStack() as stack:
                robot, subscriber, writer, _ = self.fixture(stack, message, fail_init)
                with self.assertRaises(error):
                    robot.connect()
                subscriber.Close.assert_called_once()
                self.assertFalse(robot.is_connected)
                writer.assert_not_called()

    def test_stale_invalid_and_mode_changed_feedback(self):
        for failure in ('stale', 'nan', 'mode'):
            with self.subTest(failure=failure), ExitStack() as stack:
                robot, _, writer, _ = self.fixture(stack)
                robot.connect()
                if failure == 'stale':
                    robot._read_only_received_at -= 1
                    error = TimeoutError
                else:
                    msg = self.message(2)
                    if failure == 'nan':
                        msg.motor_state[15].q = float('nan')
                    else:
                        msg.mode_machine = 6
                    self.callback(msg)
                    error = ValueError
                with self.assertRaises(error):
                    robot.get_observation()
                writer.assert_not_called()

    def test_invalid_readonly_config_rejected(self):
        for options in ({'is_simulation': True}, {'controller': 'SonicWholeBodyController'},
                        {'gravity_compensation': True}, {'network_interface': 'lo'},
                        {'max_state_age_s': float('nan')}):
            kwargs = dict(is_simulation=False, read_only=True, network_interface='test0')
            kwargs.update(options)
            with self.subTest(options=options), self.assertRaises(ValueError):
                self.config_class(**kwargs)
