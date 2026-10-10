"""Score (曲谱) loading and validation."""

from __future__ import annotations

from typing import Tuple

import yaml

from app_melodion.key_config import MelodionConfig

REST = "REST"


def load_score(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def validate_score(score: dict, config: MelodionConfig) -> Tuple[bool, str]:
    if not isinstance(score, dict) or not isinstance(score.get("notes"), list):
        return False, "score missing 'notes' list"
    notes = score["notes"]
    if not notes:
        return False, "score has no notes"
    for i, n in enumerate(notes):
        name = n.get("name")
        beats = n.get("beats")
        if name is None or beats is None:
            return False, f"note[{i}] missing name/beats"
        if name == REST:
            continue
        if config.get_key(name) is None:
            return False, f"note[{i}] unknown note '{name}'"
        if not isinstance(beats, (int, float)) or beats <= 0:
            return False, f"note[{i}] non-positive beats ({name})"
    return True, "ok"
