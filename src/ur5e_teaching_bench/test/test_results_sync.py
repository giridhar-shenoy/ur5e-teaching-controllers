"""The README results block and docs/results.md must be exactly what the CSVs produce."""

import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "make_results.py"


@pytest.mark.skipif(
    not (ROOT / "results" / "tracking.csv").exists(), reason="no benchmark CSVs committed yet"
)
def test_readme_and_results_md_match_the_csvs():
    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--check", "--no-plots"], capture_output=True, text=True
    )
    assert result.returncode == 0, result.stdout + result.stderr
