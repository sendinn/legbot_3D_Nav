"""Remove detached descendants identified by a unique per-run Gazebo partition."""
import os
from pathlib import Path
import signal
import time


def identity(pid, partition):
    try:
        proc = Path('/proc') / str(pid)
        env = (proc / 'environ').read_bytes().split(b'\0')
        if ('IGN_PARTITION=' + partition).encode() not in env:
            return None
        # Field 22 is starttime; comm may contain spaces and parentheses.
        fields = (proc / 'stat').read_text().rsplit(')', 1)[1].split()
        return fields[19] if fields[0] != 'Z' else None
    except (OSError, IndexError):
        return None


def cleanup_partition(partition, grace=3.0):
    if not partition or not partition.startswith(('legbot_navigation_', 'legbot_mapping_')):
        raise ValueError('Refusing cleanup without a unique legbot partition')
    own_pid = os.getpid()
    def members():
        found = {}
        for entry in Path('/proc').iterdir():
            if entry.name.isdigit() and int(entry.name) != own_pid:
                pid = int(entry.name)
                token = identity(pid, partition)
                if token is not None:
                    found[pid] = token
        return found
    removed = set()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        targets = members()
        if not targets:
            return sorted(removed)
        for pid, token in targets.items():
            # Recheck both ownership and process starttime before signaling.
            if identity(pid, partition) == token:
                try:
                    os.kill(pid, sig)
                    removed.add(pid)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + grace
        while time.monotonic() < deadline:
            if not members():
                return sorted(removed)
            time.sleep(0.1)
    remaining = members()
    if remaining:
        raise RuntimeError('Detached simulation processes still alive: ' + str(sorted(remaining)))
    return sorted(removed)
