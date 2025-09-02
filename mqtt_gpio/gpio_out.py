# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO

def ensure_board_mode():
    """Set GPIO board mode once."""
    if GPIO.getmode() is None:
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(True)

def setup_output(pin: int):
    ensure_board_mode()
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

def write(pin: int, high: bool):
    """Write logic to output pin."""
    GPIO.output(pin, GPIO.HIGH if high else GPIO.LOW)

def safe_cleanup_low(pins: list[int]):
    """Drive LOW then cleanup (idempotent)."""
    try:
        for p in pins:
            try:
                GPIO.output(p, GPIO.LOW)
            except Exception:
                pass
        if GPIO.getmode() is not None:
            GPIO.cleanup()
    except Exception:
        pass
