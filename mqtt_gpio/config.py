# -*- coding: utf-8 -*-
from typing import Any, Dict
import os
import yaml

def load_config(default_path: str = "config.yaml") -> Dict[str, Any]:
    """
    Load YAML configuration file.
    Priority:
      1. Environment variable MQTT_GPIO_CONFIG
      2. Provided default_path
    """
    path = os.environ.get("MQTT_GPIO_CONFIG", default_path)
    with open(path, "r") as f:
        return yaml.safe_load(f)
