# -*- coding: utf-8 -*-
from typing import Optional
import RPi.GPIO as GPIO
import logging

logger = logging.getLogger("inputs")

def ensure_board_mode():
    if GPIO.getmode() is None:
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(True)

class GpioInput:
    """Simple GPIO input (pull-up), returns logical pressed=True when pin reads LOW."""
    def __init__(self, pin: int):
        ensure_board_mode()
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.pin = pin

    def read_pressed(self) -> bool:
        return not bool(GPIO.input(self.pin))  # active-low button


# ExpanderPi adapter (lazy import)
class ExpanderInput:
    """ExpanderPi MCP23017 input via vendor module."""
    def __init__(self, io, pin: int):
        # 'io' is ExpanderPi.IO()
        self.io = io
        self.pin = int(pin)

    def read_pressed(self) -> bool:
        # Expander returns 1 on high level; we assume pull-ups so pressed==low
        v = self.io.read_pin(self.pin)
        return not bool(v)


def make_input(source: str, pin: int, expander_io: Optional[object]) -> object:
    """
    Factory for input sources. 'source' in {'gpio','expander'}.
    """
    src = str(source).lower().strip()
    if src == "gpio":
        return GpioInput(int(pin))
    if src == "expander":
        if expander_io is None:
            raise RuntimeError("Expander source requested but no IO instance provided")
        return ExpanderInput(expander_io, int(pin))
    raise ValueError(f"Unsupported input source: {source}")
