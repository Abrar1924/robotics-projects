"""Risk-aware graceful degradation for RTK navigation, modelled on a circuit breaker.

NOMINAL   RTK fixed, small uncertainty           -> full speed
DEGRADED  float fix or dead reckoning            -> slow down as uncertainty nears its limit
HOLD      k-sigma uncertainty exceeds the        -> stop (uncertainty stops growing at rest)
          clearance to the nearest hazard           and wait for RTK to recover
SKIP      hold timed out next to a hazard        -> the mission defers that hazard-adjacent
          (an event, not a state)                   strip and keeps mowing open grass
RETREAT   hold timed out and we are lost         -> drive back to the last known-good fix
          (uncertainty above sigma_max)

The stop threshold depends on where the robot is: in open grass it may keep mowing on a
float fix or dead reckoning, but next to a trunk, wall or flower bed it needs RTK fixed.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum

from .gnss import FIXED


class Mode(IntEnum):
    NOMINAL = 0
    DEGRADED = 1
    HOLD = 2
    RETREAT = 3


@dataclass
class SupervisorConfig:
    v_nominal: float = 0.50         # m/s
    v_degraded: float = 0.20        # m/s at the uncertainty limit
    v_retreat: float = 0.30         # m/s
    sigma_nominal: float = 0.04     # m (1-sigma): full speed only with RTK fixed and below this
    sigma_floor: float = 0.05       # m: tightest stop threshold, right next to a hazard
    sigma_max: float = 0.25         # m: above this we are "lost", even in open grass
    risk_k: float = 3.0             # stop when k-sigma exceeds the clearance to the nearest hazard
    resume_ratio: float = 0.6       # hysteresis for leaving HOLD
    hold_timeout: float = 10.0      # s to wait for RTK before skipping or retreating
    recovery_timeout: float = 20.0  # s to wait at the end of a retreat before resuming anyway
    max_skips_without_fix: int = 2  # after this many skips without RTK fixed, go re-acquire it
    defer_radius: float = 3.5       # m: search radius for hazard-adjacent waypoints to defer
    defer_buffer: float = 0.4       # m: width added around each deferred waypoint
    lookahead: float = 1.0          # m of upcoming path checked for clearance


@dataclass
class Supervisor:
    cfg: SupervisorConfig = field(default_factory=SupervisorConfig)
    mode: Mode = Mode.NOMINAL
    hold_since: float = 0.0
    hold_expired: bool = False      # set for one update when a hold times out
    transitions: list = field(default_factory=list)
    _sigma: float = 0.0
    _limit: float = 1.0

    def limit(self, clearance):
        """Largest acceptable 1-sigma position error at this clearance to a hazard (m)."""
        c = self.cfg
        return min(c.sigma_max, max(c.sigma_floor, clearance / c.risk_k))

    def update(self, t, quality, sigma, clearance=float("inf")):
        c, m = self.cfg, self.mode
        limit = self.limit(clearance)
        self._sigma, self._limit = sigma, limit
        self.hold_expired = False
        good = quality == FIXED and sigma < c.sigma_nominal
        if m == Mode.RETREAT:
            nxt = Mode.NOMINAL if good else Mode.RETREAT
        elif m == Mode.HOLD:
            if good or sigma < c.resume_ratio * limit:
                nxt = Mode.NOMINAL if good else Mode.DEGRADED
            elif t - self.hold_since >= c.hold_timeout:
                self.hold_expired = True
                nxt = Mode.RETREAT if sigma >= c.sigma_max else Mode.DEGRADED
            else:
                nxt = Mode.HOLD
        elif sigma >= limit:
            nxt, self.hold_since = Mode.HOLD, t
        else:
            nxt = Mode.NOMINAL if good else Mode.DEGRADED
        self._set(t, nxt)
        return self.mode

    def force(self, t, mode):
        self._set(t, mode)

    def _set(self, t, mode):
        if mode != self.mode:
            self.transitions.append((t, self.mode, mode))
            self.mode = mode

    @property
    def speed_limit(self):
        """Full speed when nominal; when degraded, slow down as uncertainty nears its limit."""
        c = self.cfg
        if self.mode == Mode.DEGRADED:
            headroom = min(max(1.0 - self._sigma / self._limit, 0.0), 1.0)
            return c.v_degraded + (c.v_nominal - c.v_degraded) * headroom
        return {Mode.NOMINAL: c.v_nominal, Mode.HOLD: 0.0, Mode.RETREAT: c.v_retreat}[self.mode]
