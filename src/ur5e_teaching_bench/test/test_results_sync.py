"""The README's generated blocks and docs/results.md must be exactly what the CSVs produce.

make_results.py --check fails if a block is stale, if the README lacks one of the marker pairs
(headline, breath, results, in that order) or if results/runs_2026-10-31/ holds no run CSV.
"""

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
