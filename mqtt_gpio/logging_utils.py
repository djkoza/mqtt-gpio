# -*- coding: utf-8 -*-
import logging
from logging.handlers import RotatingFileHandler

def _parse_size(val) -> int:
    """Parse human-readable size like '1M', '512K' into bytes."""
    if isinstance(val, int):
        return val
    s = str(val).strip().upper().replace("B", "")
    mult = 1
    if s.endswith("K"):
        mult = 1024
        s = s[:-1]
    elif s.endswith("M"):
        mult = 1024 * 1024
        s = s[:-1]
    elif s.endswith("G"):
        mult = 1024 * 1024 * 1024
        s = s[:-1]
    return int(float(s) * mult)

def setup_rotating_logger(name: str, conf: dict) -> logging.Logger:
    """Create a rotating file logger based on config."""
    level = getattr(logging, str(conf.get("level", "INFO")).upper(), logging.INFO)
    path = conf.get("path", "./server.log")
    max_bytes = _parse_size(conf.get("max_bytes", "1M"))
    backups = max(0, int(conf.get("backup_count", 4)) - 1)

    logger = logging.getLogger(name)
    logger.setLevel(level)
    handler = RotatingFileHandler(path, maxBytes=max_bytes, backupCount=backups)
    fmt = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
    handler.setFormatter(fmt)
    logger.addHandler(handler)
    return logger
