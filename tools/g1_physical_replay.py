"""Prepare calibrated G1-29 replay plans, or execute one reviewed segment on the robot host."""
import argparse
import hashlib
import json
import math
import os
import queue
import runpy
import select
import signal
import subprocess
import sys
import tempfile
import termios
import threading
import time
import tty
from pathlib import Path

import numpy as np


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class PauseState:
    """Freeze plan time while a latched operator pause is active."""

    def __init__(self):
        self.paused = False
        self.started = None
        self.total = 0.0

    def update(self, action, now):
        if action == "pause" and not self.paused:
            self.paused = True
            self.started = now
            return "paused"
        if action == "resume" and self.paused:
            self.total += now - self.started
            self.paused = False
            self.started = None
            return "resumed"
        return None

    def elapsed(self, begin, now):
        active_pause = now - self.started if self.paused else 0.0
        return now - begin - self.total - active_pause


class KeyboardPause:
    """Read Q/R immediately from an interactive terminal without blocking control."""

    def __init__(self, stream=None):
        self.stream = stream or sys.stdin
        self.fd = None
        self.settings = None

    def __enter__(self):
        if not self.stream.isatty():
            raise RuntimeError("Physical execution requires an interactive terminal for Q pause")
        self.fd = self.stream.fileno()
        self.settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        print("Keyboard safety control active: Q pauses; R resumes; Ctrl+C stops and releases.", flush=True)
        return self

    def poll(self):
        action = None
        while select.select([self.fd], [], [], 0)[0]:
            key = os.read(self.fd, 1).decode(errors="ignore").lower()
            if key == "q":
                action = "pause"
            elif key == "r":
                action = "resume"
        return action

    def __exit__(self, *_):
        if self.settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.settings)


class VoiceAlarm:
    """English speech with a background worker that cannot stall the control loop."""

    def __init__(self, tts_bin, language, rate, runner=subprocess.run):
        self.command = [tts_bin, "-w", "-l", language, "-r", str(rate)]
        self.runner = runner
        self.messages = queue.Queue(maxsize=1)
        self.thread = None

    def _speak(self, message, check):
        print(f"VOICE: {message}", flush=True)
        return self.runner([*self.command, message], check=check)

    def speak(self, message):
        self._speak(message, True)

    def __enter__(self):
        def worker():
            while True:
                message = self.messages.get()
                if message is None:
                    return
                try:
                    self._speak(message, False)
                except OSError as exc:
                    print(f"Voice alarm failed during motion: {exc}", file=sys.stderr, flush=True)

        self.thread = threading.Thread(target=worker, name="g1-replay-voice", daemon=True)
        self.thread.start()
        return self

    def announce(self, message):
        try:
            self.messages.put_nowait(message)
        except queue.Full:
            pass

    def __exit__(self, *_):
        if self.thread is not None:
            self.messages.put(None)
            self.thread.join(timeout=10)


def approach_duration(distance, cfg, requested):
    safety_minimum = max(
        cfg.period_s,
        2 * distance / cfg.max_velocity,
        math.sqrt(6 * distance / cfg.max_acceleration),
    )
    return max(requested, safety_minimum)


def validate_staging_contract(contract, args, distance, cfg):
    """Require an explicit, bounded review before a posture starts outside the normal gate."""
    if not args.stage_from_current:
        return args.approach_duration_s
    staging = contract.get("physical_replay_staging")
    if not isinstance(staging, dict):
        raise TypeError("--stage-from-current requires a reviewed physical_replay_staging contract")
    for key in ("reviewed", "path_and_clearance_reviewed", "live_waist_gravity_reviewed"):
        if staging.get(key) is not True:
            raise ValueError(f"Physical replay staging gate not approved: {key}")
    if not isinstance(staging.get("reviewer"), str) or not staging["reviewer"].strip():
        raise ValueError("Physical replay staging requires a named reviewer")
    if staging.get("label") != args.label:
        raise ValueError("Physical replay staging approval does not match the selected label")
    if staging.get("lower_body_mode") != "stock_balance":
        raise ValueError("Physical replay staging requires lower_body_mode=stock_balance")
    if cfg.gravity_urdf is None:
        raise ValueError("Stock-balance staging requires the reviewed live-waist gravity model")
    maximum = staging.get("max_start_delta_rad")
    minimum_duration = staging.get("minimum_duration_s")
    if not isinstance(maximum, (int, float)) or not math.isfinite(maximum) or maximum <= 0:
        raise ValueError("Physical replay staging needs a positive finite max_start_delta_rad")
    if (
        not isinstance(minimum_duration, (int, float))
        or not math.isfinite(minimum_duration)
        or minimum_duration < 5
    ):
        raise ValueError("Physical replay staging minimum_duration_s must be finite and at least 5")
    if distance > maximum:
        raise ValueError("Measured start exceeds the reviewed staging envelope")
    if distance > cfg.max_displacement:
        raise ValueError("Arm SDK max_displacement does not cover the reviewed staging path")
    if args.staging_duration_s < minimum_duration:
        raise ValueError("Requested staging duration is shorter than the reviewed minimum")
    return args.staging_duration_s


def wait_for_warning(keyboard, voice, seconds):
    pause = PauseState()
    begin = time.monotonic()
    while pause.elapsed(begin, time.monotonic()) < seconds:
        now = time.monotonic()
        change = pause.update(keyboard.poll(), now)
        if change == "paused":
            voice.announce("Motion start is paused. Press R to continue, or Control C to stop.")
        elif change == "resumed":
            voice.announce("Motion start resumed. Keep clear of the robot arms.")
        time.sleep(0.02)


def prepare(args):
    from lerobot.robots.unitree_g1.g1_cartesian_control import (
        G1ArmKinematics,
        G1CartesianConfig,
    )
    from lerobot_teleoperator_g1_vr.calibration import (
        load_anchor_grid,
        load_profile,
        solve_anchor,
        verify_trace_anchor,
    )

    suite = args.suite
    anchors = suite / "calibration/anchor-set.json"
    ik = G1ArmKinematics(G1CartesianConfig("g1_29", args.urdf))
    profile = load_profile(anchors, ik)
    segments = {}
    if args.kind == "postures":
        for key, entry in load_anchor_grid(anchors).items():
            if args.label and args.label != entry["label"]:
                continue
            targets, result, _ = solve_anchor(ik, profile, key, entry)
            segments[entry["label"]] = [{"action": ik.arm_action(result.q), "time_s": 0.,
                                         "targets": [t.tolist() for t in targets], "ik_status": result.status}]
    else:
        manifest = suite / "manifest.json"
        for label, segment in json.loads(manifest.read_text())["segments"].items():
            if label.startswith("calibration-") or (args.label and label != args.label):
                continue
            if segment["status"] != "passed":
                raise ValueError(f"Unaccepted segment {label}")
            verify_trace_anchor(manifest, label, anchors)
            trace = Path(segment["selected_trace"])
            subprocess.run([sys.executable, str(Path(__file__).with_name("check_g1_vr_trace.py")),
                            str(trace), "--expect", label.split("-")[0]], check=True)
            with tempfile.TemporaryDirectory() as directory:
                telemetry = Path(directory) / "trajectory.json"
                subprocess.run([sys.executable, str(Path(__file__).with_name("run_supervised_g1_29_vr.py")),
                                "--source", "trace", "--trace", str(trace), "--anchor-set", str(anchors),
                                "--urdf", str(args.urdf), "--telemetry", str(telemetry)], check=True)
                rows = json.loads(telemetry.read_text())
            for r in rows:
                r["time_s"] = r["captured_at_s"] - rows[0]["captured_at_s"]
            segments[label] = rows
    if not segments:
        raise ValueError("No matching accepted segment")
    output = args.output or suite.parents[1] / "physical-plans" / f"{args.kind}-{time.time_ns()}.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    plan = {"version": 1, "kind": args.kind, "profile": profile.metadata(), "segments": segments}
    encoded = json.dumps(plan, allow_nan=False)
    with output.open("x") as stream:
        stream.write(encoded)
    print(f"Offline replay plan {output}\nSHA256 {digest(output)}")


def execute(args):
    # The robot host uses its reviewed Arm SDK checkout and Python environment.
    from lerobot.robots.unitree_g1 import g1_arm_sdk as sdk

    checkout = Path(sdk.__file__).resolve().parents[4]
    validation = runpy.run_path(str(checkout / "examples/unitree_g1/validate_arm_sdk.py"))
    contract = json.loads(args.contract.read_text())
    contract_phase = "hold" if args.authority_only else None
    cfg = validation["validate_contract"](contract, args.urdf, True, phase=contract_phase)
    raw = args.execute.read_bytes()
    if contract.get("replay_plan_sha256") != hashlib.sha256(raw).hexdigest():
        raise ValueError("Contract must identify the reviewed replay plan by SHA256")
    plan = json.loads(raw)
    if (plan["version"] != 1 or plan["kind"] != args.kind or plan["profile"]["embodiment"] != "g1_29"
            or plan["profile"]["urdf_sha256"] != digest(args.urdf)):
        raise ValueError("Replay plan model, version or task mismatch")
    rows = plan["segments"][args.label]
    keys = sdk.ARM_KEYS
    points = np.array([[row["action"][key] for key in keys] for row in rows])
    times = np.array([r["time_s"] for r in rows]) * args.stretch
    if (not np.isfinite(points).all() or not np.isfinite(times).all() or times[0] != 0
            or np.any(np.diff(times) <= 0) or np.any(points < cfg.lower) or np.any(points > cfg.upper)):
        raise ValueError("Invalid plan timestamps or joint envelope")
    if len(points) > 1:
        dt = np.diff(times)
        velocity = np.diff(points, axis=0) / dt[:, None]
        if np.max(np.abs(velocity)) > cfg.max_velocity:
            raise ValueError("Replay timing exceeds reviewed velocity")
        padded = np.vstack((np.zeros(14), velocity, np.zeros(14)))
        intervals = np.r_[dt[0], (dt[:-1] + dt[1:]) / 2, dt[-1]]
        if np.any(np.abs(np.diff(padded, axis=0)) > cfg.max_acceleration * intervals[:, None]):
            raise ValueError("Replay timing exceeds reviewed acceleration")
    robot = sdk.G1ArmSDK(cfg)
    voice = VoiceAlarm(args.tts_bin, args.tts_language, args.voice_rate)
    phase_name = (
        "measured-pose authority hold"
        if args.authority_only
        else "initial positioning"
        if args.stage_from_current
        else "robot arm"
    )
    with args.output.open("x", buffering=1) as log, KeyboardPause() as keyboard, voice:
        completed = False
        try:
            log.write(json.dumps({"plan_sha256": contract["replay_plan_sha256"],
                                  "contract_sha256": digest(args.contract), "label": args.label,
                                  "stretch": args.stretch,
                                  "approach_duration_s": args.approach_duration_s,
                                  "stage_from_current": args.stage_from_current,
                                  "staging_duration_s": args.staging_duration_s,
                                  "authority_only": args.authority_only,
                                  "authority_hold_s": args.authority_hold_s,
                                  "tts_language": args.tts_language}) + "\n")
            robot.connect()
            measured = robot.observation()
            start = np.array([measured[k] for k in keys])
            distance = float(np.max(abs(points - start)))
            requested_approach = (
                args.approach_duration_s
                if args.authority_only
                else validate_staging_contract(contract, args, distance, cfg)
            )
            if not args.authority_only and distance > cfg.max_displacement:
                raise ValueError("Segment exceeds reviewed displacement from measured start")
            if args.kind == "trajectories" and not args.authority_only:
                expected = np.asarray(rows[0]["measured"])
                if np.max(abs(start - expected)) > cfg.max_tracking_error:
                    raise ValueError("Measured starting pose differs from simulation replay")
            voice.speak(
                f"Warning. {phase_name.capitalize()} motion is planned. Keep clear. Motion will begin after a "
                f"{args.warning_seconds:g} second warning. Press Q to pause, or Control C to stop."
            )
            if args.stage_from_current:
                voice.announce(
                    "The stock lower body balance controller remains active. "
                    "Arm gravity feedforward follows the measured pelvis and waist."
                )
                print(
                    f"Reviewed initial positioning phase: {distance:.6f} rad maximum joint travel "
                    f"over at least {args.staging_duration_s:.1f} seconds.",
                    flush=True,
                )
                log.write(
                    json.dumps(
                        {
                            "event": "initial_positioning_ready",
                            "maximum_joint_travel_rad": distance,
                            "requested_duration_s": args.staging_duration_s,
                        }
                    )
                    + "\n"
                )
            wait_for_warning(keyboard, voice, args.warning_seconds)
            robot.activate()
            begin = time.monotonic()
            pause = PauseState()
            hold = start.copy()
            last_measured = start.copy()
            next_voice = begin
            if args.authority_only:
                voice.announce(
                    "Full arm authority is active at the measured pose. No arm displacement is planned."
                )
            else:
                first_target_distance = float(np.max(abs(points[0] - start)))
                approach = approach_duration(first_target_distance, cfg, requested_approach)
            while True:
                now = time.monotonic()
                change = pause.update(keyboard.poll(), now)
                if change == "paused":
                    hold = last_measured.copy()
                    voice.announce(
                        "Emergency pause active. Planned motion is paused and arm torque remains active. "
                        "Press R to resume, or Control C to stop and release."
                    )
                    next_voice = now + args.voice_interval_s
                elif change == "resumed":
                    voice.announce("Robot arm motion is resuming. Keep clear.")
                    next_voice = now + args.voice_interval_s
                elapsed = pause.elapsed(begin, now)
                if change is not None:
                    log.write(
                        json.dumps(
                            {
                                "event": change,
                                "wall_elapsed_s": now - begin,
                                "plan_elapsed_s": elapsed,
                            }
                        )
                        + "\n"
                    )
                if args.authority_only or pause.paused:
                    target = hold
                else:
                    u = np.clip(elapsed / approach, 0., 1.)
                    t = elapsed - approach
                    if t <= 0:
                        target = start + (points[0] - start) * (u*u*(3-2*u))
                    else:
                        target = [np.interp(t, times, col) for col in points.T]
                command = dict(zip(keys, map(float, target), strict=True))
                robot.send(command)
                observation = robot.observation()
                last_measured = np.array([observation[k] for k in keys])
                log.write(json.dumps({"elapsed_s": elapsed, "target": command,
                                      "measured": last_measured.tolist(),
                                      "paused": pause.paused}) + "\n")
                if now >= next_voice:
                    if pause.paused:
                        voice.announce(
                            "Emergency pause remains active. Arm torque remains active. "
                            "Press R to resume, or Control C to stop and release."
                        )
                    else:
                        message = (
                            "Full arm authority is holding the measured pose. No displacement is planned."
                            if args.authority_only
                            else "Warning. Initial positioning is moving the robot arms. Keep clear. Press Q to pause."
                            if args.stage_from_current
                            else "Warning. Robot arms are moving. Keep clear. Press Q to pause."
                        )
                        voice.announce(message)
                    next_voice = now + args.voice_interval_s
                authority_done = args.authority_only and elapsed >= args.authority_hold_s
                motion_done = (
                    not args.authority_only and not pause.paused and t >= times[-1] + 1.0
                )
                if authority_done or motion_done:
                    if max(abs(observation[k] - command[k]) for k in keys) > cfg.max_tracking_error:
                        raise RuntimeError("Final posture outside reviewed tracking tolerance")
                    completed = True
                    break
                time.sleep(cfg.period_s)
            voice.announce(
                "Returning the robot arms to the acquisition pose before releasing to stock control."
            )
            log.write(json.dumps({"event": "normal_release_started"}) + "\n")
            robot.release_to_initial()
            log.write(
                json.dumps({"event": "normal_release_complete", "release": robot.release_context}) + "\n"
            )
        finally:
            robot.close()
        if completed:
            if args.authority_only:
                completion = "Measured-pose authority hold and normal release are complete."
            elif args.stage_from_current:
                completion = (
                    "Initial positioning and posture validation are complete. "
                    "Release to stock control was requested."
                )
            else:
                completion = "Robot arm motion is complete. Release to stock control was requested."
            voice.announce(completion)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("kind", choices=["postures", "trajectories"])
    parser.add_argument("--suite", type=Path)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--label")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--execute", type=Path, metavar="REVIEWED_PLAN")
    parser.add_argument("--contract", type=Path)
    parser.add_argument("--stretch", type=float, default=1., help="Replay duration multiplier (>=1) for hardware")
    parser.add_argument(
        "--approach-duration-s",
        type=float,
        default=7.0,
        help="planned first-target approach duration; safety limits may make it longer (default: 7)",
    )
    parser.add_argument(
        "--stage-from-current",
        action="store_true",
        help="run a separately reviewed 05a initial-positioning phase from the measured arm pose",
    )
    parser.add_argument(
        "--authority-only",
        action="store_true",
        help="acquire full arm authority at the measured pose, hold, and perform normal release without 05a motion",
    )
    parser.add_argument(
        "--authority-hold-s",
        type=float,
        default=3.0,
        help="measured-pose full-authority hold duration for --authority-only (default: 3)",
    )
    parser.add_argument(
        "--staging-duration-s",
        type=float,
        default=10.0,
        help="requested initial-positioning duration; safety limits may make it longer (default: 10)",
    )
    parser.add_argument("--warning-seconds", type=float, default=5.0)
    parser.add_argument("--voice-interval-s", type=float, default=3.0)
    parser.add_argument("--tts-bin", default=os.environ.get("G1_VR_TTS_BIN", "spd-say"))
    parser.add_argument("--tts-language", default=os.environ.get("G1_VR_TTS_LANGUAGE", "en"))
    parser.add_argument("--voice-rate", type=int, default=int(os.environ.get("G1_VR_VOICE_RATE", "-35")))
    args = parser.parse_args()
    if not math.isfinite(args.stretch) or args.stretch < 1:
        parser.error("--stretch must be finite and at least 1")
    if not math.isfinite(args.approach_duration_s) or not 1 <= args.approach_duration_s <= 60:
        parser.error("--approach-duration-s must be finite and in [1, 60]")
    if not math.isfinite(args.staging_duration_s) or not 5 <= args.staging_duration_s <= 60:
        parser.error("--staging-duration-s must be finite and in [5, 60]")
    if not math.isfinite(args.warning_seconds) or not 0 <= args.warning_seconds <= 30:
        parser.error("--warning-seconds must be finite and in [0, 30]")
    if not math.isfinite(args.voice_interval_s) or not 1 <= args.voice_interval_s <= 30:
        parser.error("--voice-interval-s must be finite and in [1, 30]")
    if not math.isfinite(args.authority_hold_s) or not 1 <= args.authority_hold_s <= 30:
        parser.error("--authority-hold-s must be finite and in [1, 30]")
    if not args.tts_language or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for c in args.tts_language):
        parser.error("--tts-language must be a language identifier such as en")
    if args.execute:
        if not (args.contract and args.label and args.output):
            parser.error("hardware execution requires --label, --contract and a new --output")
        if args.stage_from_current and args.kind != "postures":
            parser.error("--stage-from-current is only available for 05a posture execution")
        if args.authority_only and args.kind != "postures":
            parser.error("--authority-only is only available with the 05a posture executor")
        if args.authority_only and args.stage_from_current:
            parser.error("--authority-only cannot be combined with --stage-from-current")
        def stop(*_):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM, stop)
        execute(args)
    elif args.suite:
        if args.stage_from_current:
            parser.error("--stage-from-current requires --execute")
        prepare(args)
    else:
        parser.error("offline preparation requires --suite")


if __name__ == "__main__":
    main()
