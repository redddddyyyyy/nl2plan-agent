"""Pytest config: make `nl2plan_agent` importable without colcon-installing it."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "nl2plan_agent"))
