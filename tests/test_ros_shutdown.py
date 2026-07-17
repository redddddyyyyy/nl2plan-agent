"""A backend process must exit cleanly.

A daemon executor thread still spinning at interpreter teardown aborts in
rclpy's C++ layer (SIGABRT) — caught live on 2026-07-17. Run a child
process that starts the backend node and check it exits 0. Needs rclpy
but no simulator.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

pytest.importorskip("rclpy")

_SRC = Path(__file__).resolve().parent.parent / "src" / "nl2plan_agent"

_SNIPPET = f"""
import sys
sys.path.insert(0, {str(_SRC)!r})
from nl2plan_agent.ros_backend.node import get_node
get_node()
print("node up")
"""


def test_backend_process_exits_cleanly():
    proc = subprocess.run(
        [sys.executable, "-c", _SNIPPET],
        capture_output=True, text=True, timeout=60,
    )
    assert "node up" in proc.stdout
    assert proc.returncode == 0, f"backend process aborted at exit:\n{proc.stderr}"
