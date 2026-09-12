"""Operator contracts independent of SDK, GPU, physical devices, and X."""

import importlib.util
import time
from pathlib import Path

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
