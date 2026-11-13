"""Saved maps for nav_manager: names, paths and the list shown in smabo-web.

Pure Python (no ROS) so it can be tested anywhere. A map is the pair
``<maps_dir>/<name>.yaml`` + ``<name>.pgm`` that nav2_map_server's
map_saver writes and map_server / AMCL load.
"""

from __future__ import annotations

import os
import re
from datetime import datetime, timezone

_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")

DEFAULT_MAPS_DIR = os.path.expanduser("~/smabo-maps")


def maps_dir(configured: str | None = None) -> str:
    """The maps folder: parameter, else ``SMABO_MAPS_DIR``, else ~/smabo-maps."""
    return os.path.abspath(configured or os.environ.get("SMABO_MAPS_DIR") or DEFAULT_MAPS_DIR)


def check_name(name: str) -> str:
    """A map name usable as a file name (letters, digits, ``_ . -``)."""
    name = (name or "").strip()
    if name.endswith(".yaml"):
        name = name[:-5]
    if not _NAME.match(name) or ".." in name:
        raise ValueError("map names use letters, digits, '_', '.' and '-' (max 64), e.g. room_1")
    return name


def yaml_path(directory: str, name: str) -> str:
    return os.path.join(directory, check_name(name) + ".yaml")


def _read_yaml_fields(path: str) -> dict:
    """The few map-yaml fields we show (no PyYAML needed for this flat format)."""
    out = {}
    try:
        with open(path, encoding="utf-8") as f:
            for line in f:
                key, sep, value = line.partition(":")
                if sep and key.strip() in ("image", "resolution"):
                    out[key.strip()] = value.strip()
    except OSError:
        pass
    return out


def list_maps(directory: str) -> list[dict]:
    """Saved maps, newest first: ``{name, saved, resolution}``."""
    if not os.path.isdir(directory):
        return []
    out = []
    for fn in os.listdir(directory):
        if not fn.endswith(".yaml"):
            continue
        name = fn[:-5]
        try:
            check_name(name)
        except ValueError:
            continue
        path = os.path.join(directory, fn)
        fields = _read_yaml_fields(path)
        image = fields.get("image", name + ".pgm")
        if not os.path.exists(os.path.join(directory, image)):
            continue                               # half-written or broken
        mtime = os.path.getmtime(path)
        try:
            res = float(fields.get("resolution", "nan"))
        except ValueError:
            res = float("nan")
        out.append({"name": name, "mtime": mtime,
                    "saved": datetime.fromtimestamp(mtime, timezone.utc).isoformat(timespec="seconds"),
                    "resolution": None if res != res else res})
    out.sort(key=lambda m: m["mtime"], reverse=True)
    for m in out:
        del m["mtime"]
    return out
