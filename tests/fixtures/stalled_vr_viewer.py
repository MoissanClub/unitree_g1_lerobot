"""Run the real simulator with a deliberately blocked viewer child, without X."""

from pathlib import Path
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
import g1_vr_service as service


def stalled_viewer(args):
    child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(300)"])
    (args.run_dir / "viewer.pid").write_text(str(child.pid))
    return child


if __name__ == "__main__":
    service.start_viewer = stalled_viewer
    service.main()
