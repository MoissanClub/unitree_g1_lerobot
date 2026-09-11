"""JSON evidence and source provenance helpers, with no execution dependencies."""
import hashlib
import json
import subprocess


def write_report(path, result):
    path.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")


def revision(path):
    return subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip()


def source_hashes(root, directory):
    return {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(directory.rglob("*.py"))}
