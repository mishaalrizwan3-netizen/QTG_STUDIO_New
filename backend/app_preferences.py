"""Per-test GUI preferences, independent of simulator execution."""
import json
import os
from pathlib import Path
import tempfile


def load(path):
    path = Path(path)
    if not path.exists():
        return {}
    data = json.loads(path.read_text(encoding='utf-8-sig'))
    if not isinstance(data, dict) or any(
            not isinstance(ref, str) or not isinstance(values, dict)
            or any(not isinstance(k, str) or not isinstance(v, str) for k, v in values.items())
            for ref, values in data.items()):
        raise ValueError('Invalid saved application preferences')
    return data


def save(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile('w', encoding='utf-8', dir=path.parent,
                                         prefix=path.stem + '.', suffix='.tmp', delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(data, stream, indent=2)
            stream.write('\n')
        os.replace(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
