#!/usr/bin/env python3
"""Read-only baseline fingerprint check; it does NOT validate MATLAB fixes.

Exit 0: exact audited HEAD and key files; 2: drift; 3: invalid input/tool error.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, required=True)
    args = parser.parse_args()
    pack = Path(__file__).resolve().parents[1]
    try:
        baseline = json.loads((pack / 'reference/BASELINE.json').read_text(encoding='utf-8'))
        repo = args.repo.expanduser().resolve(strict=True)
        result = subprocess.run(['git', '-C', str(repo), 'rev-parse', 'HEAD'],
                                capture_output=True, text=True, check=True)
        head = result.stdout.strip()
        dirty = subprocess.run(['git', '-C', str(repo), 'status', '--porcelain'],
                               capture_output=True, text=True, check=True).stdout.splitlines()
        changed = []
        for rel, expected in baseline['files_sha256'].items():
            path = (repo / rel).resolve()
            if not path.is_relative_to(repo):
                raise ValueError('Manifest path escapes repository')
            if not path.is_file():
                changed.append({'path': rel, 'status': 'MISSING'})
            elif hashlib.sha256(path.read_bytes()).hexdigest() != expected:
                changed.append({'path': rel, 'status': 'CHANGED'})
        match = head == baseline['commit'] and not changed
        print(json.dumps({
            'status': 'BASELINE_MATCH' if match else 'DRIFT',
            'audited_commit': baseline['commit'], 'actual_commit': head,
            'key_files_checked': len(baseline['files_sha256']),
            'changed_key_files': changed, 'worktree_entries': dirty,
            'scope': 'Version and key-file fingerprints only. No MATLAB execution or correctness verdict.'
        }, ensure_ascii=False, indent=2))
        return 0 if match else 2
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as exc:
        print(json.dumps({'status': 'ERROR', 'reason': str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 3


if __name__ == '__main__':
    raise SystemExit(main())
