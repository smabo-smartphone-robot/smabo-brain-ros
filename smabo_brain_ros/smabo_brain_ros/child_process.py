"""Run a ``ros2 launch`` / ``ros2 run`` as its own process group and stop it.

Shared by mode_manager and nav_manager, which each run one heavy stack as a
child process and swap it on request.
"""

from __future__ import annotations

import os
import signal
import subprocess
import time


def start_group(args: list[str]) -> subprocess.Popen:
    """Start ``args`` in a new session, so stop_group can signal all its children."""
    return subprocess.Popen(args, start_new_session=True)


def stop_group(p: subprocess.Popen | None, sigint_wait: float = 10.0, settle: float = 1.0) -> None:
    """Stop the whole process group: SIGINT, then SIGTERM, then SIGKILL.

    Does nothing when ``p`` is None or has already exited. ``settle`` seconds
    are waited afterwards so the old nodes are gone (e.g. the old map -> odom
    publisher) before the next stack starts.
    """
    if p is None or p.poll() is not None:
        return
    for sig, wait in ((signal.SIGINT, sigint_wait), (signal.SIGTERM, 5.0), (signal.SIGKILL, 5.0)):
        try:
            os.killpg(p.pid, sig)
        except ProcessLookupError:
            return
        try:
            p.wait(timeout=wait)
            break
        except subprocess.TimeoutExpired:
            continue
    time.sleep(settle)
