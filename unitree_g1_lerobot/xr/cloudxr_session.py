"""Bounded or interactive CloudXR service lifetime, independent of robot control."""
import argparse
import math
import signal
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-s", type=float, default=0)
    parser.add_argument("--cloudxr-env-config", required=True)
    args = parser.parse_args()
    if not math.isfinite(args.duration_s) or args.duration_s < 0:
        parser.error("duration-s must be finite and nonnegative")
    from isaacteleop.cloudxr.launcher import CloudXRLauncher

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    with CloudXRLauncher(env_config=args.cloudxr_env_config, accept_eula=True) as launcher:
        print("CloudXR ready: runtime and WSS proxy running", flush=True)
        print("Steady-state listening for headset connections (no robot video stream).", flush=True)
        deadline = time.monotonic() + args.duration_s if args.duration_s else math.inf
        while not stop.is_set() and time.monotonic() < deadline:
            launcher.health_check()
            stop.wait(0.1)
    print("CloudXR stopped cleanly", flush=True)


if __name__ == "__main__":
    main()
