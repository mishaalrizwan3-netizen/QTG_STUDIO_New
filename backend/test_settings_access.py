"""Local administrator password for Test Settings when no environment password is set."""

import base64
import hashlib
import hmac
import json
import os
from pathlib import Path
import secrets


_ITERATIONS = 200_000


def credential_path():
    local_data = os.environ.get('LOCALAPPDATA')
    root = Path(local_data) if local_data else Path.home() / 'AppData' / 'Local'
    return root / 'QTG Studio' / 'test_settings_admin.json'


def is_configured():
    return credential_path().exists()


def set_password(password):
    if not password or not password.strip():
        raise ValueError('Administrator password cannot be blank')
    path = credential_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    salt = secrets.token_bytes(16)
    digest = hashlib.pbkdf2_hmac('sha256', password.encode('utf-8'), salt, _ITERATIONS)
    record = {
        'version': 1,
        'iterations': _ITERATIONS,
        'salt': base64.b64encode(salt).decode('ascii'),
        'hash': base64.b64encode(digest).decode('ascii'),
    }
    with path.open('x', encoding='utf-8') as stream:
        json.dump(record, stream)


def verify_password(password):
    record = json.loads(credential_path().read_text(encoding='utf-8'))
    if record.get('version') != 1 or not isinstance(record.get('iterations'), int):
        raise ValueError('Unsupported administrator password file')
    salt = base64.b64decode(record['salt'], validate=True)
    expected = base64.b64decode(record['hash'], validate=True)
    digest = hashlib.pbkdf2_hmac('sha256', password.encode('utf-8'), salt, record['iterations'])
    return hmac.compare_digest(digest, expected)
