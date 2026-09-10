"""Read the simulator's camera channel in a separate process, with optional Tk preview."""
import argparse
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image

from ..simulation.camera_frames import default_channel, read_frame


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-channel", type=Path, default=default_channel())
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration-s", type=float, default=0.)
    parser.add_argument("--save-frame", type=Path)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    if not np.isfinite(args.duration_s) or args.duration_s < 0:
        parser.error("duration-s must be finite and nonnegative")
    if args.headless and not args.duration_s:
        parser.error("Headless camera verification requires --duration-s")
    root = label = status = None
    if not args.headless:
        import tkinter as tk
        from PIL import ImageTk
        root = tk.Tk()
        root.title("Robot camera - local preview")
        placeholder = tk.PhotoImage(width=640, height=480)
        label = tk.Label(root, image=placeholder, width=640, height=480)
        label.image = placeholder
        label.pack()
        status = tk.StringVar(value="Waiting for camera")
        tk.Label(root, textvariable=status).pack(fill="x")
    started = time.monotonic()
    last_key = previous = None
    samples = []
    received = 0
    max_std = 0.
    closed = False

    def close():
        nonlocal closed
        closed = True
        if root is not None:
            root.destroy()

    def tick():
        nonlocal last_key, previous, received, max_std
        if args.duration_s and time.monotonic() - started >= args.duration_s:
            close()
            return
        result = read_frame(args.camera_channel)
        if result is None:
            if status is not None:
                status.set("No fresh camera frame")
        else:
            metadata, pixels = result
            key = (metadata["session_id"], metadata["sequence"])
            if key != last_key:
                age_ms = (time.monotonic_ns() - metadata["captured_monotonic_ns"]) / 1e6
                delta = 0. if previous is None or previous.shape != pixels.shape else float(np.mean(np.abs(pixels.astype(float)-previous)))
                std = float(pixels.std())
                received += 1
                max_std = max(max_std, std)
                if args.report:
                    samples.append(dict(metadata, age_ms=age_ms, pixel_std=std, pixel_delta=delta))
                previous, last_key = pixels.astype(float), key
                image = Image.fromarray(pixels)
                if args.save_frame:
                    args.save_frame.parent.mkdir(parents=True, exist_ok=True)
                    image.save(args.save_frame)
                if root is not None:
                    photo = ImageTk.PhotoImage(image)
                    label.configure(image=photo, width=image.width, height=image.height)
                    label.image = photo
                    status.set(f"{metadata['embodiment']} / {metadata['camera_id']} | "
                               f"frame {metadata['sequence']} | age {age_ms:.0f} ms")
        if root is not None and not closed:
            root.after(20, tick)

    try:
        if root is None:
            while not closed:
                tick()
                time.sleep(.01)
        else:
            root.protocol("WM_DELETE_WINDOW", close)
            tick()
            root.mainloop()
    except KeyboardInterrupt:
        close()
    finally:
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(dict(samples=samples, elapsed_s=time.monotonic()-started)) + "\n")
    if args.headless and (received < 2 or max_std < 5):
        raise RuntimeError("Camera verification failed: insufficient fresh, nonblank frames")
    print(f"Camera consumer: received {received} fresh frames", flush=True)


if __name__ == "__main__":
    main()
