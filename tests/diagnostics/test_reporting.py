"""Evidence serialization and provenance checks without robot connections."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from unitree_g1_lerobot.diagnostics.shared.reporting import source_hashes, write_report


class ReportingTests(unittest.TestCase):
    def test_json_and_nonfinite_rejection(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "report.json"
            write_report(path, {"passed": True, "error_rad": .01})
            self.assertEqual(json.loads(path.read_text())["error_rad"], .01)
            before = path.read_text()
            with self.assertRaises(ValueError):
                write_report(path, {"error_rad": float("nan")})
            self.assertEqual(path.read_text(), before)

    def test_nested_module_hashes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            nested = root / "shared"
            nested.mkdir()
            source = nested / "metrics.py"
            source.write_text("value = 1\n")
            self.assertEqual(source_hashes(root, root), {
                "shared/metrics.py": hashlib.sha256(source.read_bytes()).hexdigest(),
            })
