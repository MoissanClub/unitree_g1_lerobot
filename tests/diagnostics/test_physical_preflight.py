"""Offline safety and feedback tests; no SDK, network, or robot required."""
import json
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from unitree_g1_lerobot.diagnostics.physical import physical_preflight as p


class PreflightTests(unittest.TestCase):
    def setUp(self):
        self.profile = json.loads((p.ROOT / 'configs/motors/g1_29.json').read_text())
        self.obs = p.Observation(self.profile, .1)

    def msg(self, tick=1):
        states = [NS(q=0., dq=0., tau_est=0., mode=0, motorstate=0) for _ in range(35)]
        for m in self.profile['motors']:
            states[m['dds_index']].q = sum(m['range_rad']) / 2
        return NS(tick=tick, motor_state=states, mode_machine=1, mode_pr=0)

    def test_fresh_feedback_and_tick_wrap(self):
        self.obs.receive(self.msg(2**32-1), 1)
        self.obs.receive(self.msg(0), 1.02)
        self.assertTrue(self.obs.report(1.03)['feedback_checks_passed'])

    def test_missing_stale_frozen_and_regressed(self):
        self.assertFalse(self.obs.report(1)['feedback_checks_passed'])
        self.obs.receive(self.msg(), 1)
        self.obs.receive(self.msg(), 1.02)
        self.assertFalse(self.obs.report(1.2)['feedback_checks_passed'])
        self.obs.receive(self.msg(0), 1.21)
        self.assertIn('Robot tick regressed or sources are mixed', self.obs.report(1.22)['errors'])

    def test_transient_gap_and_mode_change_persist(self):
        self.obs.receive(self.msg(), 1)
        msg = self.msg(2)
        msg.mode_machine = 2
        self.obs.receive(msg, 1.2)
        self.assertFalse(self.obs.report(1.21)['feedback_checks_passed'])

    def test_invalid_feedback_is_latched(self):
        msg = self.msg()
        msg.motor_state[15].q = float('nan')
        self.obs.receive(msg, 1)
        self.obs.receive(self.msg(2), 1.01)
        self.obs.receive(self.msg(3), 1.02)
        self.assertFalse(self.obs.report(1.03)['feedback_checks_passed'])

    def test_g1_23_sparse_mapping(self):
        profile = json.loads((p.ROOT / 'configs/motors/g1_23.json').read_text())
        obs = p.Observation(profile, .1)
        obs.receive(self.msg(), 1)
        slots = {j['dds_index'] for j in obs.latest['joints']}
        self.assertEqual(len(slots), 23)
        self.assertFalse(slots & {13, 14, 20, 21, 27, 28})

    def test_recovered_tick_stall_remains_failure(self):
        self.obs.receive(self.msg(1), 1)
        self.obs.receive(self.msg(1), 1.09)
        self.obs.receive(self.msg(2), 1.15)
        self.assertIn('Robot tick stalled during observation', self.obs.report(1.16)['errors'])

    def test_short_transport_and_range_violation(self):
        msg = self.msg()
        msg.motor_state = msg.motor_state[:29]
        self.obs.receive(msg, 1)
        self.assertTrue(self.obs.errors)
        msg = self.msg(2)
        msg.motor_state[15].q = 100
        self.obs.receive(msg, 1.01)
        self.assertTrue(any('Outside model' in error for error in self.obs.errors))

    def test_unavailable_checkout_is_not_audited(self):
        self.assertFalse(p.audit_checkout(None)['matches_audited_files'])
        self.assertFalse(p.audit_checkout(Path('/nonexistent/lerobot'))['matches_audited_files'])

    def test_subscriber_only_and_cleanup_on_failure(self):
        for fail in (False, True):
            calls, closed = [], []
            class Subscriber:
                def __init__(self, topic, dtype):
                    self.topic = topic
                    calls.append(topic)
                def Init(self, callback):
                    if fail and self.topic == 'rt/arm_sdk':
                        raise RuntimeError('injected failure')
                def Close(self):
                    closed.append(self.topic)
            class Channel:
                ChannelSubscriber = Subscriber
                @staticmethod
                def ChannelFactoryInitialize(*args):
                    pass
                def __getattr__(self, name):
                    raise AssertionError(f'Forbidden SDK access: {name}')
            args = NS(domain_id=0, network_interface='test0', duration_s=.001)
            if fail:
                with self.assertRaises(RuntimeError):
                    p.observe(args, self.obs, Channel(), object, object)
            else:
                p.observe(args, self.obs, Channel(), object, object)
            self.assertEqual(calls, ['rt/lowstate', 'rt/lowcmd', 'rt/arm_sdk'])
            self.assertEqual(closed, list(reversed(calls)))

    def test_cli_report_never_enables_motion(self):
        import tempfile
        def observe(args, observation):
            now = p.time.monotonic()
            observation.receive(self.msg(1), now-.02)
            observation.receive(self.msg(2), now-.01)
        with tempfile.TemporaryDirectory() as tmp, \
             patch.object(p.socket, 'if_nameindex', return_value=[(1, 'test0')]), \
             patch.object(p, 'observe', side_effect=observe):
            output = Path(tmp) / 'report.json'
            args = ['--embodiment', 'g1_29', '--network-interface', 'test0', '--output', str(output)]
            self.assertEqual(p.main(args), 0)
            report = json.loads(output.read_text())
            self.assertFalse(report['motion_enabled'])
            self.assertEqual(report['stage0_gate'], 'pending_external_review')
            self.assertTrue(report['missing_contract_fields'])
            with self.assertRaises(FileExistsError):
                p.main(args)

    def test_cli_failure_and_interruption_leave_reports(self):
        import tempfile
        for exc, code in [(RuntimeError('SDK unavailable'), 1), (KeyboardInterrupt(), 130)]:
            with tempfile.TemporaryDirectory() as tmp, \
                 patch.object(p.socket, 'if_nameindex', return_value=[(1, 'test0')]), \
                 patch.object(p, 'observe', side_effect=exc):
                output = Path(tmp) / 'report.json'
                self.assertEqual(p.main(['--embodiment', 'g1_29', '--network-interface',
                                         'test0', '--output', str(output)]), code)
                report = json.loads(output.read_text())
                self.assertEqual(report['status'], 'failed')
                self.assertFalse(report['motion_enabled'])


if __name__ == '__main__':
    unittest.main()
