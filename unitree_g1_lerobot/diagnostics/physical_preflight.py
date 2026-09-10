"""Passive physical G1 preflight. Never creates a command writer or robot instance.

Exit 0 means feedback checks passed, NOT permission or readiness to actuate.
Run in a fresh process; DDS discovery traffic is sent, but no application commands.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import socket
import subprocess
import threading
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
CONTRACT_FIELDS = (
    "physical_identity", "mapping_verification", "control_mode_evidence",
    "arm_owner", "legs_waist_balance_owner", "physical_support", "command_interface",
    "single_publisher_verification", "measured_state_initialization", "ownership_transition",
    "gains_and_feedforward", "stale_feedback_response", "lost_commands_response",
    "operator_stop_procedure", "interruption_response", "normal_exit_handover",
    "lifecycle_audit_revision", "reviewer",
)


def revision(path):
    try:
        return subprocess.check_output(
            ["git", "-C", str(path), "rev-parse", "HEAD"], stderr=subprocess.DEVNULL,
            text=True, timeout=3).strip()
    except (OSError, subprocess.SubprocessError):
        return None


def file_hash(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit_checkout(path):
    manifest = json.loads((ROOT / "configs/physical_preflight_audit.json").read_text())
    hashes = {}
    if path is not None:
        hashes = {name: file_hash(path / name) if (path / name).is_file() else None
                  for name in manifest["files"]}
    return dict(path=str(path) if path else None, revision=revision(path) if path else None,
                files=hashes, matches_audited_files=hashes == manifest["files"],
                audited_revision=manifest["lerobot_revision"],
                patch_matches_audit=file_hash(ROOT / "patches/lerobot-g1-embodiments.patch") == manifest["patch_sha256"])


class Observation:
    def __init__(self, profile, max_age):
        self.profile = profile
        self.max_age = max_age
        self.lock = threading.Lock()
        self.count = 0
        self.last_received = self.last_progress = self.first_received = None
        self.tick = None
        self.max_gap = 0.0
        self.errors = set()
        self.modes = set()
        self.latest = None
        self.transport_details = {}
        self.commands = {"rt/lowcmd": 0, "rt/arm_sdk": 0}

    def receive(self, msg, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            try:
                if len(msg.motor_state) != 35:
                    raise ValueError("Expected HG 35-slot motor_state transport")
                joints = []
                for motor in self.profile["motors"]:
                    state = msg.motor_state[motor["dds_index"]]
                    values = [float(state.q), float(state.dq), float(state.tau_est)]
                    if not all(math.isfinite(v) for v in values):
                        raise ValueError(f"Nonfinite feedback: {motor['joint']}")
                    lo, hi = motor["range_rad"]
                    if not lo - 0.02 <= values[0] <= hi + 0.02:
                        self.errors.add(f"Outside model position range: {motor['joint']}")
                    joints.append(dict(joint=motor["joint"], dds_index=motor["dds_index"],
                                       q=values[0], dq=values[1], tau_est=values[2],
                                       mode=int(state.mode), motorstate=int(state.motorstate)))
                tick = int(msg.tick)
                if self.last_received is not None:
                    self.max_gap = max(self.max_gap, now - self.last_received)
                else:
                    self.first_received = now
                if self.tick is None:
                    self.last_progress = now
                elif tick != self.tick:
                    delta = (tick - self.tick) % (2**32)
                    if delta >= 2**31:
                        self.errors.add("Robot tick regressed or sources are mixed")
                    if now - self.last_progress > self.max_age:
                        self.errors.add("Robot tick stalled during observation")
                    self.last_progress = now
                self.tick = tick
                self.last_received = now
                self.count += 1
                self.modes.add((int(msg.mode_machine), int(msg.mode_pr)))
                self.latest = dict(tick=tick, mode_machine=int(msg.mode_machine),
                                   mode_pr=int(msg.mode_pr), joints=joints)
            except (AttributeError, TypeError, ValueError, IndexError, OverflowError) as exc:
                self.errors.add(str(exc))

    def command(self, topic):
        def receive(_msg):
            with self.lock:
                self.commands[topic] += 1
        return receive

    def report(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            errors = set(self.errors)
            age = None if self.last_received is None else now - self.last_received
            progress_age = None if self.last_progress is None else now - self.last_progress
            if self.count < 2:
                errors.add("Fewer than two valid feedback samples")
            if age is None or age > self.max_age:
                errors.add("Missing or stale feedback at end of observation")
            if progress_age is None or progress_age > self.max_age:
                errors.add("Robot tick is not advancing freshly")
            if self.max_gap > self.max_age:
                errors.add("Feedback gap exceeded freshness bound")
            if len(self.modes) > 1:
                errors.add("Raw control mode fields changed during observation")
            span = 0 if self.count < 2 else self.last_received - self.first_received
            return dict(feedback_checks_passed=not errors, errors=sorted(errors),
                        valid_samples=self.count, receive_rate_hz=(self.count - 1) / span if span else None,
                        last_receive_age_s=age, last_tick_progress_age_s=progress_age,
                        max_receive_gap_s=self.max_gap, latest=self.latest,
                        transport_details=self.transport_details,
                        observed_command_samples=dict(self.commands))


def observe(args, observation, channel=None, state_type=None, command_type=None):
    if channel is None:
        import unitree_sdk2py.core.channel as channel
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
        state_type, command_type = LowState_, LowCmd_
        # Some SDK installs hard-code a shared /tmp/cdds.LOG owned by another user.
        # Keep interface/transport settings but remove optional tracing for this process.
        config = ET.fromstring(channel.ChannelConfigHasInterface)
        for domain in config.findall("Domain"):
            for tracing in domain.findall("Tracing"):
                domain.remove(tracing)
        channel.ChannelConfigHasInterface = ET.tostring(config, encoding="unicode")
        source = Path(channel.__file__).resolve()
        observation.transport_details = dict(
            backend="unitree_sdk2py DDS subscribers", channel_source=str(source),
            channel_sha256=file_hash(source), sdk_checkout_revision=revision(source.parent),
            dds_tracing_disabled=True,
        )
    subscribers = []
    try:
        channel.ChannelFactoryInitialize(args.domain_id, args.network_interface)
        for topic, dtype, callback in [
            ("rt/lowstate", state_type, observation.receive),
            *[(topic, command_type, observation.command(topic)) for topic in observation.commands],
        ]:
            sub = channel.ChannelSubscriber(topic, dtype)
            subscribers.append(sub)
            sub.Init(callback)  # No worker queue: timestamp receipt directly.
        deadline = time.monotonic() + args.duration_s
        while time.monotonic() < deadline:
            time.sleep(min(0.02, max(0, deadline - time.monotonic())))
    finally:
        errors = []
        for sub in reversed(subscribers):
            try:
                sub.Close()
            except Exception as exc:
                errors.append(str(exc))
        if errors:
            observation.errors.add("Subscriber cleanup failed: " + "; ".join(errors))


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("Requires a finite positive value")
    return number


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--embodiment", choices=("g1_29", "g1_23"), required=True)
    parser.add_argument("--network-interface", required=True)
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument("--duration-s", type=positive, default=5.0)
    parser.add_argument("--max-feedback-age-s", type=positive, default=0.1)
    parser.add_argument("--contract", type=Path, help="Operator observations; never automatically authorize motion")
    parser.add_argument("--lerobot-root", type=Path, help="Hash-check a patched checkout against the lifecycle audit; no imports")
    parser.add_argument("--output", type=Path, required=True, help="New durable JSON report; refuses overwrite")
    args = parser.parse_args(argv)
    if args.network_interface == "lo" or args.network_interface not in dict((n, i) for i, n in socket.if_nameindex()):
        parser.error("Select an existing physical network interface (loopback is rejected)")
    if not 0 <= args.domain_id <= 232:
        parser.error("domain-id must be between 0 and 232")
    if args.duration_s < 2 * args.max_feedback_age_s:
        parser.error("duration-s must be at least twice max-feedback-age-s")
    contract = {}
    if args.contract:
        contract = json.loads(args.contract.read_text())
        if not isinstance(contract, dict):
            parser.error("contract must be a JSON object")
    profile_path = ROOT / "configs" / "motors" / f"{args.embodiment}.json"
    profile = json.loads(profile_path.read_text())
    observation = Observation(profile, args.max_feedback_age_s)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Reserve the report before connecting; incomplete runs cannot resemble a passed run.
    with args.output.open("x") as stream:
        json.dump({"status": "incomplete", "motion_enabled": False}, stream)
    report = dict(schema_version=1, started_utc=datetime.now(timezone.utc).isoformat(),
                  embodiment=args.embodiment, network_interface=args.network_interface,
                  domain_id=args.domain_id, duration_s=args.duration_s,
                  max_feedback_age_s=args.max_feedback_age_s,
                  project_revision=revision(ROOT), profile_sha256=file_hash(profile_path),
                  script_sha256=file_hash(Path(__file__)),
                  lifecycle_audit=audit_checkout(args.lerobot_root),
                  patch_sha256=file_hash(ROOT / "patches/lerobot-g1-embodiments.patch"),
                  expected_mapping=[{k: m[k] for k in ("joint", "dds_index", "arm", "range_rad")}
                                    for m in profile["motors"]],
                  operator_contract=contract,
                  missing_contract_fields=[k for k in CONTRACT_FIELDS
                                           if not isinstance(contract.get(k), str) or not contract[k].strip()],
                  motion_enabled=False, stage0_gate="pending_external_review",
                  limitations=["Physical identity, joint signs, and controller ownership need external verification.",
                               "Command sample counts do not count publishers or prove absence of a silent writer.",
                               "mode_machine/mode_pr are raw fields, not MotionSwitcher mode or ownership confirmation.",
                               "Local receive time and tick progression do not prove synchronized source age.",
                               "Model ranges are sanity checks, not approved hardware motion limits.",
                               "G1-23 hardware actuation remains blocked."])
    status = 1
    try:
        observe(args, observation)
        status = 0
    except KeyboardInterrupt:
        observation.errors.add("Operator interrupted observation")
        status = 130
    except Exception as exc:
        observation.errors.add(f"Preflight failed: {type(exc).__name__}: {exc}")
    finally:
        report.update(observation.report())
        report["status"] = "feedback_pass" if report["feedback_checks_passed"] else "failed"
        report["finished_utc"] = datetime.now(timezone.utc).isoformat()
        temp = args.output.with_suffix(args.output.suffix + ".tmp")
        temp.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        temp.replace(args.output)
    if not report["feedback_checks_passed"] and status == 0:
        status = 1
    print(f"{report['status']}: {args.output}; Stage 0 requires external contract review; no motion enabled.")
    return status


if __name__ == "__main__":
    raise SystemExit(main())
