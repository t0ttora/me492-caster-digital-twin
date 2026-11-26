"""Yeni kullanıcı, proje kökünden iki giriş komutunu bulabilmeli."""
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).parents[1]


@pytest.mark.parametrize("script", ["run_experiments.py", "verify_model.py"])
def test_entry_point_explains_usage(script):
    result = subprocess.run([sys.executable, str(ROOT/script), "--help"], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert "usage:" in result.stdout
