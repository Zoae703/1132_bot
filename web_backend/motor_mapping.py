"""
Motor channel mapping persistence.

Stores PWM channel → physical thruster mapping as JSON.
"""

import json
import logging
import os
import tempfile
from pathlib import Path
from typing import List, Dict, Any

logger = logging.getLogger("opi_console.mapping")

def load_mapping(path: str | Path, *, channel_count: int = 8,
                 neutral_us: int = 1500, safe_min_us: int = 1450,
                 safe_max_us: int = 1550, min_absolute_us: int = 1300,
                 max_absolute_us: int = 1700) -> Dict[str, Any]:
    """Load motor mapping from JSON file."""
    path = Path(path)
    defaults_list = _default_mappings(
        channel_count, neutral_us, safe_min_us, safe_max_us)
    if not path.exists():
        return {"mappings": defaults_list}
    try:
        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)
        if not isinstance(data, dict) or not isinstance(data.get("mappings"), list):
            raise ValueError("mapping file must contain a mappings list")
        # Validate the persisted file as untrusted input before merging it.
        defaults = {m["channel"]: m for m in defaults_list}
        seen_channels = set()
        for m in data.get("mappings", []):
            if not isinstance(m, dict):
                raise ValueError("mapping entries must be objects")
            channel = m.get("channel")
            if (type(channel) is not int or channel not in defaults
                    or channel in seen_channels):
                raise ValueError(f"invalid mapping channel: {channel}")
            seen_channels.add(channel)

            physical_name = m.get("physical_name", m.get("name", ""))
            direction = m.get("direction", "")
            notes = m.get("notes", "")
            reversed_value = m.get("reversed", False)
            entry_neutral = m.get("neutral_us", neutral_us)
            entry_min = m.get("safe_min_us", m.get("min_us", safe_min_us))
            entry_max = m.get("safe_max_us", m.get("max_us", safe_max_us))

            if (not isinstance(physical_name, str)
                    or len(physical_name) > 128):
                raise ValueError(f"invalid physical_name for channel {channel}")
            if not isinstance(direction, str) or len(direction) > 64:
                raise ValueError(f"invalid direction for channel {channel}")
            if not isinstance(notes, str) or len(notes) > 500:
                raise ValueError(f"invalid notes for channel {channel}")
            if type(reversed_value) is not bool:
                raise ValueError(f"invalid reversed flag for channel {channel}")
            if type(entry_neutral) is not int or entry_neutral != neutral_us:
                raise ValueError(f"invalid neutral_us for channel {channel}")
            if type(entry_min) is not int or type(entry_max) is not int:
                raise ValueError(f"invalid PWM limits for channel {channel}")
            if not (
                min_absolute_us <= entry_min < neutral_us
                < entry_max <= max_absolute_us
            ):
                raise ValueError(f"unsafe PWM limits for channel {channel}")

            defaults[channel] = {
                "channel": channel,
                "physical_name": physical_name,
                "direction": direction,
                "reversed": reversed_value,
                "neutral_us": entry_neutral,
                "safe_min_us": entry_min,
                "safe_max_us": entry_max,
                "notes": notes,
            }
        return {"mappings": [defaults[channel] for channel in sorted(defaults)]}
    except Exception as e:
        logger.error("Failed to load motor mapping: %s", e)
        return {"mappings": defaults_list}


def save_mapping(mappings: List[Dict[str, Any]], path: str | Path):
    """Atomically save motor mapping to JSON."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as temp_file:
            temp_path = Path(temp_file.name)
            json.dump({"mappings": mappings}, temp_file, indent=2)
            temp_file.write("\n")
            temp_file.flush()
            os.fsync(temp_file.fileno())
        os.replace(temp_path, path)
        directory_fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    except Exception:
        if temp_path is not None:
            try:
                temp_path.unlink(missing_ok=True)
            except OSError as cleanup_error:
                logger.warning(
                    "Failed to clean temporary mapping file %s: %s",
                    temp_path, cleanup_error)
        raise
    logger.info("Motor mapping saved to %s", path)


def _default_mappings(channel_count: int, neutral_us: int,
                      safe_min_us: int, safe_max_us: int) -> List[Dict[str, Any]]:
    """Default (empty) motor mapping for 8 channels."""
    return [
        {
            "channel": ch,
            "physical_name": f"Motor {ch}",
            "direction": "",
            "reversed": False,
            "neutral_us": neutral_us,
            "safe_min_us": safe_min_us,
            "safe_max_us": safe_max_us,
            "notes": "",
        }
        for ch in range(channel_count)
    ]
