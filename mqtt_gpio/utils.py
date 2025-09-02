# -*- coding: utf-8 -*-
import re

_SAFE_ID_RE = re.compile(r"[^a-z0-9_]+")

def to_object_id(text: str) -> str:
    """Make a Home Assistant-safe object_id (lowercase, underscores, a-z0-9_)."""
    obj = str(text).strip().lower().replace(" ", "_")
    return _SAFE_ID_RE.sub("", obj)
