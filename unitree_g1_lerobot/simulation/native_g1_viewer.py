"""Tk preview and raise-arm verification for the native G1-23 DDS simulator."""
import select
import sys
import threading
import time

import mujoco
import numpy as np
from PIL import Image, ImageTk
import tkinter as tk
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_


def raise_arm_diagnostic(env, args):
    publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
    publisher.Init()
    try:
        initial = env.snapshot()[env.plant.qadr]
        target = initial.copy()
        for j, name in enumerate(env.plant.joints):
            if "shoulder_pitch" in name:
                target[j] = -0.9
            elif "elbow" in name:
                target[j] = 1.1
        print("raising robot arm - verify the diagnostic motion of both G1-23 arms raising", flush=True)
        start = time.monotonic()
        prompted = False
        while not env.stop_event.is_set():
            elapsed = time.monotonic() - start
            alpha = min(1, elapsed / args.diagnostic_duration_s)
            desired = initial + (target - initial) * (alpha * alpha * (3 - 2 * alpha))
            msg = unitree_hg_msg_dds__LowCmd_()
            with env.lock:
                gravity = env.plant.gravity_torque()
            for j, index in enumerate(env.indices):
                motor = msg.motor_cmd[index]
                motor.mode = 1
                motor.q = float(desired[j])
                motor.kp, motor.kd = float(env.plant.kp[j]), float(env.plant.kd[j])
                motor.tau = float(gravity[j])
            msg.crc = env.crc.Crc(msg)
            publisher.Write(msg)
            if elapsed >= args.diagnostic_duration_s + 0.5:
                if args.no_diagnostic_confirm:
                    break
                if not prompted:
                    print("Press Enter after you verify the diagnostic motion of arm raising, or Ctrl+C to abort...", flush=True)
                    prompted = True
                if select.select([sys.stdin], [], [], 0)[0]:
                    if not sys.stdin.readline():
                        raise RuntimeError("Startup confirmation requires stdin; use --no-diagnostic-confirm for unattended tests")
                    break
            env.stop_event.wait(1 / 60)
        if not env.stop_event.is_set():
            result = "Startup diagnostic complete (confirmation skipped)" if args.no_diagnostic_confirm else "User verification complete"
            print(f"{result}; entering steady-state listening on DDS for g1_23.", flush=True)
    finally:
        publisher.Close()


def run_native(env, args, root=None):
    started = time.monotonic()
    if args.no_view:
        env.start()
        print("G1-23 physics and loopback DDS running; no viewer requested.", flush=True)
        deadline = threading.Timer(args.duration_s, env.stop_event.set) if args.duration_s else None
        if deadline:
            deadline.start()
        try:
            if not args.skip_startup_diagnostic:
                raise_arm_diagnostic(env, args)
            while not env.stop_event.wait(0.05):
                pass
        finally:
            if deadline:
                deadline.cancel()
                deadline.join()
        if env.failure:
            raise env.failure
        return
    root = tk.Tk() if root is None else root
    root.deiconify()
    root.title("G1-23 MuJoCo DDS - supported arms")
    label = tk.Label(root)
    label.pack()
    status = tk.StringVar(value="G1-23: opening viewer before diagnostic")
    tk.Label(root, textvariable=status).pack(fill="x")
    render_data = mujoco.MjData(env.plant.model)
    renderer = mujoco.Renderer(env.plant.model, height=720, width=720)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0.05, 0, 0.73]
    camera.distance, camera.azimuth, camera.elevation = 2.2, -135, -10
    option = mujoco.MjvOption()
    option.geomgroup[3] = 0
    diagnostic = None
    timer = None
    errors = []
    closed = False

    def close():
        nonlocal closed
        if closed:
            return
        closed = True
        env.stop_event.set()
        if timer is not None:
            root.after_cancel(timer)
        root.destroy()

    def run_diagnostic():
        try:
            raise_arm_diagnostic(env, args)
        except BaseException as exc:
            errors.append(exc)
            env.stop_event.set()

    def tick():
        nonlocal timer, diagnostic
        if env.stop_event.is_set() or (env.thread is not None and args.duration_s and time.monotonic() - started >= args.duration_s):
            close()
            return
        render_data.qpos[:] = env.snapshot()
        mujoco.mj_forward(env.plant.model, render_data)
        renderer.update_scene(render_data, camera=camera, scene_option=option)
        pixels = renderer.render().copy()
        if np.std(pixels) < 5:
            raise RuntimeError("Blank G1-23 viewer frame")
        image = Image.fromarray(pixels)
        if args.save_frame:
            image.save(args.save_frame)
        photo = ImageTk.PhotoImage(image)
        label.configure(image=photo)
        label.image = photo
        status.set(f"g1_23 | physics steps={env.steps} | DDS commands={env.received_commands} | rejected={env.rejected_commands} | {'holding' if env.holding else 'following DDS'}")
        if diagnostic is None and time.monotonic() - started > 0.8 and not args.skip_startup_diagnostic:
            diagnostic = threading.Thread(target=run_diagnostic, name="g1-23-diagnostic")
            diagnostic.start()
        timer = root.after(max(1, round(1000 / args.view_fps)), tick)

    def callback_error(kind, error, traceback):
        errors.append(error)
        close()

    root.report_callback_exception = callback_error
    root.protocol("WM_DELETE_WINDOW", close)
    try:
        # Paint the first frame before physics and the diagnostic are started.
        tick()
        root.update_idletasks()
        print("G1-23 Tk/X viewer open; physics starts before the raise-arm diagnostic.", flush=True)
        env.start()
        root.mainloop()
    finally:
        close()
        if diagnostic is not None:
            diagnostic.join(timeout=3)
        renderer.close()
    if errors:
        raise errors[0]
    if env.failure:
        raise env.failure
