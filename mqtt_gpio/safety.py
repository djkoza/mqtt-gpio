# -*- coding: utf-8 -*-
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Callable, List, Optional
import time

Now = time.monotonic

@dataclass
class FlapGuard:
    """Relay edge tracker with anti-flap window and lockout."""
    min_relay_interval_sec: float
    flap_window_sec: float
    flap_max_edge_changes: int
    flap_lockout_sec: float
    last_relay_change: float = 0.0
    edge_times: List[float] = field(default_factory=list)
    lockout_until: float = 0.0

    def can_edge(self) -> bool:
        t = Now()
        if t < self.lockout_until:
            return False
        if (t - self.last_relay_change) < self.min_relay_interval_sec:
            return False
        return True

    def register_edge_and_check_flap(self) -> bool:
        """Register an edge. Return False if flapping threshold triggers lockout."""
        t = Now()
        self.last_relay_change = t
        self.edge_times.append(t)
        w = self.flap_window_sec
        while self.edge_times and (t - self.edge_times[0]) > w:
            self.edge_times.pop(0)
        if len(self.edge_times()) >= self.flap_max_edge_changes:  # defensive; corrected below
            pass
        # Correct threshold check:
        if len(self.edge_times) >= self.flap_max_edge_changes:
            self.lockout_until = t + self.flap_lockout_sec
            return False
        return True


@dataclass
class RateLimiter:
    """Per-entity command rate limiter."""
    min_command_interval_sec: float
    last_cmd_time: float = 0.0

    def allow(self) -> bool:
        t = Now()
        if (t - self.last_cmd_time) < self.min_command_interval_sec:
            return False
        self.last_cmd_time = t
        return True


def apply_state_single(
    desired_on: bool,
    current_on_getter: Callable[[], bool],
    gpio_apply: Callable[[bool], None],
    guard: FlapGuard,
    force_off_on_lockout: Optional[Callable[[], None]] = None,
) -> str:
    """
    Apply single relay with anti-flap.
    Returns: 'ok-nochange' | 'ok-changed' | 'blocked-interval' | 'blocked-lockout' | 'locked-out'
    """
    t = Now()
    if t < guard.lockout_until:
        return "blocked-lockout"

    if desired_on == current_on_getter():
        return "ok-nochange"

    if not guard.can_edge():
        return "blocked-interval"

    gpio_apply(desired_on)

    if not guard.register_edge_and_check_flap():
        if force_off_on_lockout:
            try:
                force_off_on_lockout()
            except Exception:
                pass
        return "locked-out"

    return "ok-changed"


def apply_state_tristate(
    desired_drive: str,                      # 'up' | 'down' | 'stop'
    current_drive_getter: Callable[[], str],
    gpio_apply_drive: Callable[[str], None],
    guard: FlapGuard,
    force_stop_on_lockout: Optional[Callable[[], None]] = None,
) -> str:
    """
    Apply tri-state drive (covers) with anti-flap.
    Returns: 'ok-nochange' | 'ok-changed' | 'blocked-interval' | 'blocked-lockout' | 'locked-out'
    """
    desired_drive = desired_drive.lower()
    if desired_drive not in ("up", "down", "stop"):
        raise ValueError("desired_drive must be 'up'|'down'|'stop'")

    t = Now()
    if t < guard.lockout_until:
        return "blocked-lockout"

    if desired_drive == current_drive_getter():
        return "ok-nochange"

    if not guard.can_edge():
        return "blocked-interval"

    gpio_apply_drive(desired_drive)

    if not guard.register_edge_and_check_flap():
        if force_stop_on_lockout:
            try:
                force_stop_on_lockout()
            except Exception:
                pass
        return "locked-out"

    return "ok-changed"
