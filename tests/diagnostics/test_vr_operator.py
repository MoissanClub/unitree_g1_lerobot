"""Operator contracts independent of SDK, GPU, physical devices, and X."""

import importlib.util
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


def module(name):
    path = Path(__file__).resolve().parents[2] / "tools" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


service = module("g1_vr_service")
operator = module("g1_vr_operator")


def test_installation_completion_guidance(capsys):
    installer = module("install_g1_vr_sim")
    installer.print_next_steps(Path("/tmp/checkout with spaces"))
    output = capsys.readouterr().out
    for expected in (
        "INSTALLATION SUCCESSFUL",
        "ssh -Y YOUR_USER@WORKSTATION_IP",
        "cd '/tmp/checkout with spaces'",
        "--accept-cloudxr-eula",
        "--embodiment g1_29",
        "--embodiment g1_23",
        "--headless --live-xr",
        "Installation did not accept it",
        "Ctrl+C",
        "visual review",
    ):
        assert expected in output


@pytest.mark.parametrize("size,embodiment", [(10, "g1_23"), (14, "g1_29")])
def test_command_validation(size, embodiment):
    msg = dict(embodiment=embodiment, q=[0.1] * size, sent_at=time.monotonic())
    q = service.validate_command(msg, embodiment, size, -np.ones(size), np.ones(size))
    np.testing.assert_allclose(q, 0.1)


@pytest.mark.parametrize(
    "override",
    [
        {"embodiment": "g1_29"},
        {"q": [0]},
        {"q": [float("nan")] * 10},
        {"q": [2] * 10},
        {"sent_at": float("nan")},
        {"sent_at": 0},
        {"sent_at": time.monotonic() + 3600},
    ],
)
def test_bad_commands_rejected(override):
    msg = dict(embodiment="g1_23", q=[0] * 10, sent_at=time.monotonic())
    msg.update(override)
    with pytest.raises(ValueError):
        service.validate_command(msg, "g1_23", 10, -np.ones(10), np.ones(10))


def test_expired_command_is_recoverable_but_future_or_invalid_is_not():
    msg = dict(embodiment="g1_23", q=[0] * 10, sent_at=time.monotonic() - 1)
    with pytest.raises(service.ExpiredCommand):
        service.validate_command(msg, "g1_23", 10, -np.ones(10), np.ones(10))
    for update in ({"q": [2] * 10}, {"sent_at": time.monotonic() + 1}):
        with pytest.raises(ValueError) as error:
            service.validate_command(
                msg | update, "g1_23", 10, -np.ones(10), np.ones(10)
            )
        assert not isinstance(error.value, service.ExpiredCommand)


def test_readiness_rejects_dead_child(tmp_path):
    class Child:
        returncode = 7

        def poll(self):
            return self.returncode

    with pytest.raises(RuntimeError, match="simulator exited"):
        operator.wait_ready(tmp_path / "ready", [("simulator", Child())], timeout=1)


def test_readiness_timeout(tmp_path):
    with pytest.raises(TimeoutError):
        operator.wait_ready(tmp_path / "ready", [], timeout=0)


def test_readiness_success(tmp_path):
    (tmp_path / "ready").touch()
    operator.wait_ready(tmp_path / "ready", [])


def test_shutdown_waits_for_consumers_before_producers(tmp_path):
    events = []

    class Child:
        returncode = 0

        def __init__(self, role):
            self.role = role

        def wait(self, timeout):
            assert (tmp_path / f"stop.{self.role}").exists()
            if self.role == "bridge":
                assert not (tmp_path / "stop.simulator").exists()
                assert not (tmp_path / "stop.cloudxr").exists()
            if self.role == "simulator":
                assert events == ["bridge"]
                assert not (tmp_path / "stop.cloudxr").exists()
            events.append(self.role)

    children = [(role, Child(role)) for role in ("simulator", "cloudxr", "bridge")]
    assert operator.shutdown_services(tmp_path, children) == []
    assert events == ["bridge", "simulator", "cloudxr"]


def test_viewer_close_is_not_a_shared_producer_stop(tmp_path):
    (tmp_path / "managed").touch()
    (tmp_path / "stop").touch()
    for role in ("simulator", "cloudxr"):
        args = SimpleNamespace(role=role, run_dir=tmp_path)
        assert not service.stopped(args)
        (tmp_path / f"stop.{role}").touch()
        assert service.stopped(args)
    assert service.stopped(SimpleNamespace(role="bridge", run_dir=tmp_path))


def camera_frame(sequence, uniform=True, source="g1_29"):
    pixels = np.zeros((6, 8, 3), dtype=np.uint8)
    if not uniform:
        pixels[0, 0] = 100
    return {
        "embodiment": source, "session_id": "test", "sequence": sequence
    }, pixels


def test_uniform_camera_is_nonfatal_and_logs_only_transitions(capsys):
    monitor = service.CameraMonitor("g1_29")
    monitor.observe(camera_frame(1))
    monitor.observe(camera_frame(2))
    monitor.observe(None)
    monitor.observe(camera_frame(3, uniform=False))
    monitor.observe(camera_frame(4, uniform=False))
    output = capsys.readouterr().out
    assert output.count("Camera image is uniform") == 1
    assert output.count("visible detail again") == 1
    assert len(monitor.sequences) == 4
    monitor.verify_replay(0.1)


@pytest.mark.parametrize("source", ["g1_23", None])
def test_camera_source_mismatch_is_explicit(source):
    monitor = service.CameraMonitor("g1_29")
    with pytest.raises(RuntimeError, match=f"expected 'g1_29', got {source!r}"):
        monitor.observe(camera_frame(1, source=source))


@pytest.mark.parametrize("frames,motion", [
    ([camera_frame(1), camera_frame(2)], 0.1),
    ([camera_frame(1, uniform=False)] * 2, 0.1),
    ([camera_frame(1), camera_frame(2, uniform=False)], 0.0),
    ([], 0.1),
])
def test_replay_requires_motion_distinct_frames_and_visible_content(frames, motion):
    monitor = service.CameraMonitor("g1_29")
    for frame in frames:
        monitor.observe(frame)
    with pytest.raises(RuntimeError, match="Replay failed"):
        monitor.verify_replay(motion)


@pytest.mark.parametrize("force_cleanup", [False, True])
def test_primary_bridge_error_survives_shutdown(tmp_path, monkeypatch, capsys, force_cleanup):
    import json
    import sys

    source = tmp_path / "lerobot"
    (source / "src/lerobot").mkdir(parents=True)
    assets = tmp_path / "assets"
    assets.mkdir()
    (assets / "g1_29.urdf").touch()
    config = tmp_path / "config.json"
    config.write_text(json.dumps({"python": sys.executable, "lerobot": str(source), "assets": str(assets)}))
    monkeypatch.setattr(operator, "ROOT", tmp_path)
    monkeypatch.setattr(sys, "argv", ["operator", "--headless", "--config", str(config)])
    monkeypatch.setattr(operator.signal, "signal", lambda *args: operator.signal.SIG_DFL)
    monkeypatch.setattr(operator.os, "killpg", lambda *args: None)
    monkeypatch.setattr(operator, "wait_ready", lambda *args: None)

    class Child:
        pid = 12345

        def __init__(self, command, **kwargs):
            self.role = command[2]
            self.returncode = 1 if self.role == "bridge" else None
            self.waits = 0

        def poll(self):
            return self.returncode

        def wait(self, timeout):
            self.waits += 1
            if force_cleanup and self.role == "simulator" and self.waits == 1:
                raise operator.subprocess.TimeoutExpired("simulator", timeout)
            if self.returncode is None:
                self.returncode = -15 if force_cleanup and self.role == "simulator" else 0

    monkeypatch.setattr(operator.subprocess, "Popen", Child)
    with pytest.raises(RuntimeError, match=r"Bridge failed \(1\)") as error:
        operator.main()
    output = capsys.readouterr().out
    assert "bridge exited with code 1" not in output
    if force_cleanup:
        assert "simulator required forced termination" in output
        assert "simulator required forced termination" in error.value.__notes__[0]
    else:
        assert "Shutdown problems" not in output


def test_installer_pin_is_available_in_shipped_bundle():
    import subprocess

    installer = module("install_g1_vr_sim")
    bundle = installer.ROOT / "patches/lerobot-g1-vr-fixes.bundle"
    heads = subprocess.check_output(["git", "bundle", "list-heads", str(bundle)], text=True)
    assert heads.splitlines() == [f"{installer.COMMIT} refs/heads/fix/g1-vr-recovery"]


@pytest.mark.parametrize("mode", ["expired", "timeout"])
def test_simulator_hold_paths_project_without_rewriting_feedback(tmp_path, monkeypatch, mode):
    import sys
    import types

    measured = np.r_[-1.025, 1.025, np.zeros(8)]
    commands, replies = [], []
    ik = SimpleNamespace(
        size=10, lower=-np.ones(10), upper=np.ones(10),
        arm_action=lambda q: np.asarray(q).copy(),
        project_arm_positions=lambda q: np.clip(q, -1, 1),
    )
    sim = SimpleNamespace(ik=ik, data=SimpleNamespace(qpos=measured), qadr=np.arange(10))

    class Robot:
        _native = sim

        def connect(self):
            pass

        def disconnect(self):
            pass

        def send_action(self, q):
            assert q.shape == (10,)
            assert np.all(q >= -1) and np.all(q <= 1)
            commands.append(q.copy())

        def step_simulation(self):
            pass

        def render_simulation(self, *args):
            return np.ones((240, 320, 3), dtype=np.uint8)

    class Writer:
        def __init__(self, *args):
            pass

        def publish(self, *args):
            pass

        def close(self):
            pass

    socket = SimpleNamespace(
        setsockopt=lambda *args: None, bind=lambda *args: None,
        poll=lambda *args: True, recv_json=lambda: {"q": [0] * 10},
        send_json=lambda reply: replies.append(reply), close=lambda: None,
    )
    context = SimpleNamespace(socket=lambda *args: socket, term=lambda: None)
    monkeypatch.setitem(sys.modules, "zmq", SimpleNamespace(Context=lambda: context, REP=1, LINGER=2))
    monkeypatch.setitem(sys.modules, "lerobot.cameras.frame_channel", types.SimpleNamespace(FrameWriter=Writer))
    monkeypatch.setitem(sys.modules, "lerobot.robots.unitree_g1", types.SimpleNamespace(
        UnitreeG1=lambda config: Robot(), UnitreeG1Config=lambda **kwargs: None,
    ))
    # Use the real loop, with a clock that reaches timeout immediately after the command.
    ticks = iter(range(20))
    monkeypatch.setattr(service.time, "monotonic", lambda: float(next(ticks)))
    monkeypatch.setattr(service.time, "sleep", lambda _: None)
    stops = iter((False, True))
    monkeypatch.setattr(service, "stopped", lambda _: next(stops))

    def validate(*args):
        if mode == "expired":
            raise service.ExpiredCommand()
        return np.zeros(10)

    monkeypatch.setattr(service, "validate_command", validate)
    args = SimpleNamespace(embodiment="g1_23", assets=tmp_path, run_dir=tmp_path, headless=True)
    service.simulator(args)
    np.testing.assert_array_equal(commands[-1], np.clip(measured, -1, 1))
    np.testing.assert_array_equal(replies[0]["q"], measured)
    np.testing.assert_array_equal(sim.data.qpos, measured)
    assert replies[0].get("status") == ("expired_command" if mode == "expired" else None)
