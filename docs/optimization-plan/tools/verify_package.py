#!/usr/bin/env python3
"""Verify development-package contents, not project implementation status."""
import hashlib
import json
from pathlib import Path
import sys


def main():
    root = Path(__file__).resolve().parents[1]
    manifest_path = root / 'SHA256SUMS.json'
    try:
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        expected = manifest['files_sha256']
        failures = []
        for rel, digest in expected.items():
            path = (root / rel).resolve()
            if not path.is_relative_to(root):
                raise ValueError('Manifest path escapes package')
            if not path.is_file():
                failures.append({'path': rel, 'reason': 'MISSING'})
            elif hashlib.sha256(path.read_bytes()).hexdigest() != digest:
                failures.append({'path': rel, 'reason': 'HASH_MISMATCH'})
        actual = {str(p.relative_to(root).as_posix()) for p in root.rglob('*')
                  if p.is_file() and '__pycache__' not in p.parts and p != manifest_path}
        extra = sorted(actual - set(expected))
        print(json.dumps({'status': 'PACKAGE_INTACT' if not failures and not extra else 'PACKAGE_CHANGED',
                          'files_checked': len(expected), 'failures': failures,
                          'unexpected_files': extra,
                          'scope': 'File integrity only. MATLAB validation remains NOT_RUN.'},
                         ensure_ascii=False, indent=2))
        return 0 if not failures and not extra else 2
    except (OSError, ValueError, KeyError) as exc:
        print(json.dumps({'status': 'ERROR', 'reason': str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 3


if __name__ == '__main__':
    raise SystemExit(main())
