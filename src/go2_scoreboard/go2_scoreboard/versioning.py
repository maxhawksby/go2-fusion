"""Identify the exact source used even when Docker cannot see .git."""

import hashlib
import os


def source_fingerprint(source_root='/ros2_ws/src'):
    digest = hashlib.sha256()
    found = False
    if not os.path.isdir(source_root):
        return ''
    for root, directories, files in os.walk(source_root):
        directories[:] = sorted(name for name in directories
                                if name not in ('__pycache__', '.pytest_cache', 'results'))
        for name in sorted(files):
            if not name.endswith(('.py', '.cpp', '.hpp', '.h', '.yaml', '.xml', '.msg', '.srv')):
                continue
            path = os.path.join(root, name)
            relative = os.path.relpath(path, source_root).replace(os.sep, '/')
            digest.update(relative.encode())
            try:
                with open(path, 'rb') as stream:
                    for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                        digest.update(chunk)
            except OSError:
                continue
            found = True
    return digest.hexdigest()[:16] if found else ''
