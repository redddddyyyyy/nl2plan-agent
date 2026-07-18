"""Pytest config: make the packages importable without colcon-installing them."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "nl2plan_agent"))
sys.path.insert(0, str(ROOT / "src" / "perception_node"))
