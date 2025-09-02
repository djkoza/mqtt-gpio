# -*- coding: utf-8 -*-
from typing import Any, Dict
import yaml

def load_config(path: str = "config.yaml") -> Dict[str, Any]:
    """Load YAML configuration file."""
    with open(path, "r") as f:
        return yaml.safe_load(f)
