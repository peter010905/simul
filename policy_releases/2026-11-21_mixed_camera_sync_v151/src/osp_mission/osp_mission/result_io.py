"""Publish mission results atomically and tolerate legacy partial writes."""
import json
from pathlib import Path


def write_result(path, value):
    path = Path(path)
    temporary = path.with_name(path.name+'.tmp')
    temporary.write_text(json.dumps(value, indent=2))
    temporary.replace(path)


def read_result(path):
    try:
        value = json.loads(Path(path).read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) and isinstance(value.get('reason'), str) else None
