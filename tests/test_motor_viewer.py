"""Optional real Tk/EGL interaction checks: G1_TEST_VIEWER=1 python -m unittest discover -s tests."""
import os
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np


@unittest.skipUnless(os.environ.get("G1_TEST_VIEWER") == "1", "requires an X display and EGL")
class MotorViewerTests(unittest.TestCase):
    def test_pair_and_summary_controls(self):
        import tkinter as tk
        from tkinter import ttk
        from unitree_g1_lerobot.diagnostics.compare_motor_configs import PairRenderer, comparison_cases, replay
        from unitree_g1_lerobot.diagnostics.motor_suite import SUITE
        from unitree_g1_lerobot.simulation.motor_bench import MotorPlant, mesh_directory, run_trial

        meshes = mesh_directory()
        tests = [next(t for t in SUITE if t.name == name)
                 for name in ("pitch_20deg_step", "extended_hold_1kg")]
        for comparison in ("lerobot", "summary"):
            with self.subTest(comparison=comparison):
                pairs = [[run_trial(MotorPlant(p, meshes, test.payload_kg, mode), test)
                          for p, mode in comparison_cases(comparison)] for test in tests]
                args = SimpleNamespace(width=400, height=380, camera_azimuth=-135,
                                       camera_elevation=-10, camera_distance=2.2, gui_seconds=5)
                renderer = PairRenderer(pairs[0], meshes, args)
                try:
                    before = np.asarray(renderer.frame(100)).astype(float)
                    after = np.asarray(renderer.frame(1000)).astype(float)
                    columns = 3 if comparison == "summary" else 2
                    for panel in range(len(pairs[0])):
                        row, column = divmod(panel, columns)
                        region = np.s_[row * 380 + 60:(row + 1) * 380, column * 400:(column + 1) * 400]
                        self.assertGreater(np.mean(np.abs(after[region] - before[region])), 0.1)
                finally:
                    renderer.close()
                events, paused_indices, frames = [], [], []
                state = {"paused": False}
                original_tk, original_frame = tk.Tk, PairRenderer.frame

                def frame(renderer, index):
                    if state["paused"]:
                        paused_indices.append(index)
                    result = original_frame(renderer, index)
                    if not frames:
                        frames.append(np.asarray(result))
                    return result

                def root_factory():
                    root = original_tk()

                    def controls():
                        children = root.winfo_children()[0].winfo_children()
                        buttons = {w.cget("text"): w for w in children if isinstance(w, ttk.Button)}
                        combos = [w for w in children if isinstance(w, ttk.Combobox)]
                        return buttons, combos

                    def pause():
                        controls()[0]["Pause"].invoke()
                        state["paused"] = True
                        events.append("pause")

                    def resume():
                        state["paused"] = False
                        controls()[0]["Play"].invoke()
                        events.append("resume")

                    def command(name, expected):
                        buttons, combos = controls()
                        buttons[name].invoke()
                        self.assertEqual(combos[0].current(), expected)
                        events.append(name)

                    def select():
                        _, combos = controls()
                        combos[0].current(1)
                        combos[0].event_generate("<<ComboboxSelected>>")
                        combos[1].set("2")
                        events.append("select/speed")

                    def schedule():
                        root.after(100, pause)
                        root.after(900, resume)
                        root.after(1200, lambda: command("Next", 1))
                        root.after(1900, lambda: command("Previous", 0))
                        root.after(2600, lambda: command("Restart", 0))
                        root.after(3300, select)

                    root.after_idle(schedule)
                    return root

                with patch.object(tk, "Tk", root_factory), patch.object(PairRenderer, "frame", frame):
                    replay(pairs, meshes, args)
                self.assertEqual(len(events), 6)
                self.assertGreaterEqual(len(paused_indices), 2)
                self.assertEqual(len(set(paused_indices)), 1)
                expected = (760, 1200, 3) if comparison == "summary" else (380, 800, 3)
                self.assertEqual(frames[0].shape, expected)
                self.assertGreater(np.std(frames[0]), 5)


if __name__ == "__main__":
    unittest.main()
