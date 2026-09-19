"""Operator contracts independent of SDK, GPU, physical devices, and X."""

import importlib.util
import subprocess
import sys
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


@pytest.mark.parametrize("legacy", [False, True])
def test_simulation_joint_metadata_compatibility(legacy):
    joints = SimpleNamespace(size=10, lower=-np.ones(10), upper=np.ones(10))
    sim = SimpleNamespace(ik=joints) if legacy else joints
    assert service.simulation_joints(sim) is joints


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


def test_installer_release_pin_is_unchanged():
    installer = module("install_g1_vr_sim")
    assert installer.RELEASE_TAG == "release/g1-vr-sim-d5e400bc"
    assert installer.COMMIT == "d5e400bcefeccc93ba956ce876530e5283512df1"


def test_installer_clones_release_tag_before_installing(tmp_path, monkeypatch):
    installer = module("install_g1_vr_sim")
    repo = tmp_path / "origin"
    subprocess.run(["git", "init", "-b", "main", str(repo)], check=True)
    subprocess.run(
        [
            "git",
            "-C",
            str(repo),
            "-c",
            "user.name=Test",
            "-c",
            "user.email=test@example.com",
            "commit",
            "--allow-empty",
            "-m",
            "release",
        ],
        check=True,
    )
    commit = subprocess.check_output(
        ["git", "-C", str(repo), "rev-parse", "HEAD"], text=True
    ).strip()
    subprocess.run(["git", "-C", str(repo), "tag", installer.RELEASE_TAG], check=True)
    monkeypatch.setattr(installer, "REPOSITORY", str(repo))
    monkeypatch.setattr(installer, "COMMIT", commit)
    monkeypatch.setattr(installer.platform, "system", lambda: "Linux")
    monkeypatch.setattr(installer.platform, "machine", lambda: "x86_64")
    monkeypatch.setattr(installer.shutil, "which", lambda name: name)
    prefix = tmp_path / "installation"
    monkeypatch.setattr(sys, "argv", ["installer", "--prefix", str(prefix)])
    actual_run = installer.run

    class DependenciesReached(Exception):
        pass

    def git_only(*args, **kwargs):
        if args[0] != "git":
            raise DependenciesReached
        actual_run(*args, **kwargs)

    monkeypatch.setattr(installer, "run", git_only)
    with pytest.raises(DependenciesReached):
        installer.main()
    checkout = prefix / "lerobot"
    assert (
        subprocess.check_output(
            ["git", "-C", str(checkout), "rev-parse", "HEAD"], text=True
        ).strip()
        == commit
    )
    assert (
        subprocess.check_output(
            ["git", "-C", str(checkout), "branch", "--show-current"], text=True
        ).strip()
        == ""
    )


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
