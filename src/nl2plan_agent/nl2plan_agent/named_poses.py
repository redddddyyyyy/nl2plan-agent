"""Load named-pose mapping from config/named_poses.yaml."""

from __future__ import annotations

from pathlib import Path

import yaml


def load_named_poses(path: Path | str) -> dict[str, dict]:
    p = Path(path)
    if not p.exists():
        return {}
    with p.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}
