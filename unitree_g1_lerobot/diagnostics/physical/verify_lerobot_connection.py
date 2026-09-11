"""Verify real LeRobot init/connect/get_observation/disconnect in passive DDS mode."""
import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import importlib
import json
from pathlib import Path
import sys
import tempfile
import time
from unittest.mock import patch

from unitree_g1_lerobot.diagnostics.physical.physical_preflight import ROOT, Observation, audit_checkout, positive, file_hash


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lerobot-root', type=Path, required=True)
    parser.add_argument('--network-interface', required=True)
    parser.add_argument('--embodiment', choices=('g1_29', 'g1_23'), default='g1_29')
    parser.add_argument('--duration-s', type=positive, default=5.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    args.lerobot_root = args.lerobot_root.expanduser().resolve()
    audit = audit_checkout(args.lerobot_root)
    if not audit['matches_audited_files'] or not audit['patch_matches_audit']:
        parser.error('LeRobot checkout must match the current lifecycle audit manifest')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as f:
        json.dump({'status': 'incomplete', 'motion_enabled': False}, f)
    report = dict(started_utc=datetime.now(timezone.utc).isoformat(), lifecycle_audit=audit,
                  embodiment=args.embodiment, network_interface=args.network_interface,
                  script_sha256=file_hash(Path(__file__)), motion_enabled=False,
                  mode='LeRobot physical read_only DDS', initialized=False, connected=False,
                  disconnected=False, observation_calls=0, errors=[],
                  stage0_gate='pending_external_review')
    observation = Observation(json.loads((ROOT / 'configs/motors' / f'{args.embodiment}.json').read_text()), .1)
    exit_code = 1
    try:
        sys.path.insert(0, str(args.lerobot_root / 'src'))
        from unitree_g1_lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config
        import unitree_sdk2py.core.channel as channel
        import zmq
        module = importlib.import_module(UnitreeG1.__module__)
        if Path(module.__file__).resolve() != args.lerobot_root / 'src/lerobot/robots/unitree_g1/unitree_g1.py':
            raise RuntimeError('LeRobot import does not match audited checkout')
        with tempfile.TemporaryDirectory(prefix='g1-connect-calibration-') as tmp, ExitStack() as stack:
            guards = []
            for owner, name in [(channel, 'ChannelPublisher'), (module, '_SDKChannelPublisher'),
                                (zmq.Context, 'socket')]:
                guards.append(stack.enter_context(patch.object(owner, name,
                    side_effect=AssertionError(f'Read-only verification forbids {name}'))))
            robot = UnitreeG1(UnitreeG1Config(embodiment=args.embodiment, is_simulation=False,
                        read_only=True, network_interface=args.network_interface,
                        controller=None, gravity_compensation=False, cameras={},
                        calibration_dir=Path(tmp)))
            report['initialized'] = True
            if robot.is_connected:
                raise RuntimeError('Constructor unexpectedly connected')
            try:
                robot.connect()
                report['connected'] = robot.is_connected
                if not robot.is_connected:
                    raise RuntimeError('connect() did not establish feedback')
                deadline = time.monotonic() + args.duration_s
                last_tick = None
                while time.monotonic() < deadline:
                    values = robot.get_observation()
                    expected = {f'{j.name}.q' for j in robot.joint_index}
                    if {k for k in values if k.endswith('.q')} != expected:
                        raise ValueError('LeRobot observation joint schema mismatch')
                    report['observation_calls'] += 1
                    with robot._lowstate_lock:
                        state = robot._lowstate
                        received_at = robot._read_only_received_at
                    if state.tick != last_tick:
                        observation.receive(state, received_at)
                        last_tick = state.tick
                    report['last_lerobot_joint_positions'] = {k: values[k] for k in sorted(expected)}
                    time.sleep(.005)
                # Snapshot feedback statistics before teardown, which is measured separately.
                report['feedback'] = observation.report()
            finally:
                robot.disconnect()
                report['disconnected'] = not robot.is_connected
            for guard in guards:
                guard.assert_not_called()
            report['command_transport_attempts'] = sum(g.call_count for g in guards)
        if not report['feedback']['feedback_checks_passed']:
            raise ValueError('; '.join(report['feedback']['errors']))
        if not report['disconnected']:
            raise RuntimeError('Robot remained connected after disconnect()')
        report['status'] = 'pass'
        exit_code = 0
    except KeyboardInterrupt:
        report['status'] = 'interrupted'
        exit_code = 130
    except Exception as exc:
        report['status'] = 'failed'
        report['errors'].append(f'{type(exc).__name__}: {exc}')
    finally:
        report['finished_utc'] = datetime.now(timezone.utc).isoformat()
        report['sampling_note'] = 'Feedback snapshots sampled through LeRobot at up to 200 Hz; not DDS receive rate. Command topics were not monitored.'
        temp = args.output.with_suffix('.tmp')
        temp.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        temp.replace(args.output)
    print(f"{report['status']}: {args.output}")
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
