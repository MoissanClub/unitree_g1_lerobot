"""Frozen branch selection, independent of simulator and hardware dependencies."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


PATH = Path(__file__).resolve().parents[2] / "tools/verify_lerobot_branch_stack.py"
SPEC = importlib.util.spec_from_file_location("branch_stack_verifier", PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.fixture
def verifier(tmp_path):
    args = SimpleNamespace(
        checkout=tmp_path / "checkout",
        output=tmp_path / "reports",
        remote="unused",
        base=MODULE.BASE,
        gpu=False,
        python=Path("python"),
        assets=tmp_path / "assets",
        sdk_site_packages=None,
        integration_branch="verification/test",
    )
    return MODULE.Verification(args)


def pinned_refs():
    return {
        f"origin/{MODULE.ARCHIVE_PREFIX}/{name}": sha
        for name, sha in MODULE.STACK_COMMITS.items()
    }


def test_all_nine_stages_use_frozen_refs(verifier, monkeypatch):
    refs = pinned_refs()
    monkeypatch.setattr(verifier, "run", lambda command: refs[command[-1]])
    resolved = verifier.resolve_branches()
    assert list(resolved) == [MODULE.BUGFIX_BRANCH, *MODULE.BRANCHES]
    assert len(resolved) == 9
    assert all(name.startswith("archive/") for name in resolved)


def test_changed_archive_is_rejected(verifier, monkeypatch):
    monkeypatch.setattr(verifier, "run", lambda command: "0" * 40)
    with pytest.raises(RuntimeError, match="Archived branch .* changed"):
        verifier.resolve_branches()


def test_execute_never_checks_out_active_pr_branches(verifier, monkeypatch):
    refs, commands, stages = pinned_refs(), [], []

    def run(command, **kwargs):
        commands.append(command)
        if command[:2] == ["git", "rev-parse"]:
            return refs.get(command[-1], "final-test-commit")
        return ""

    monkeypatch.setattr(verifier, "run", run)
    monkeypatch.setattr(verifier, "examples", lambda: None)
    monkeypatch.setattr(
        verifier, "verify", lambda *args, **kwargs: stages.append((args, kwargs))
    )
    verifier.execute()
    selections = [
        command[-1]
        for command in commands
        if command[:3] == ["git", "switch", "--detach"]
        or command[:2] == ["git", "merge"]
    ]
    assert selections == list(refs) * 2
    assert len(stages) == 18
    assert verifier.report["status"] == "passed_without_gpu_gate"
