"""GNSS fault and attack scenarios injected into receiver output."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Scenario:
    name: str
    kind: str = "nominal"           # nominal | jamming | multipath | float | jump_spoof | drift_spoof
    start: float = 60.0             # s
    duration: float = math.inf      # s; spoofers persist
    magnitude: float = 0.0          # m for jumps and steps, m/s^2 for drift spoofing
    max_speed: float = 2.0          # m/s, drift spoofer's velocity cap
    direction: tuple = (1.0, 0.0)   # direction of the *reported* offset
    array: str = "single"           # spoofer's transmit geometry, seen by a dual-antenna receiver:
                                    #   single   one transmit antenna: the measured baseline collapses
                                    #   frozen   antenna array replaying the baseline seen at onset
                                    #   tracking attitude-aware array: correct baseline at every instant

    @property
    def is_attack(self):
        return self.kind in ("jump_spoof", "drift_spoof")

    @property
    def should_alarm(self):
        """Faults where the receiver output is wrong but claims to be fine."""
        return self.kind in ("multipath", "jump_spoof", "drift_spoof")

    def active(self, t):
        return self.kind != "nominal" and 0.0 <= t - self.start < self.duration

    def drift(self, tau):
        """Drift spoofer: accelerate the fake offset gently, then hold a capped drag speed,
        so no single fix looks like a jump."""
        a, vmax = self.magnitude, self.max_speed
        t1 = vmax / a
        return 0.5 * a * tau * tau if tau < t1 else 0.5 * a * t1 * t1 + vmax * (tau - t1)

    def apply(self, t, z, sigma, rng, memory):
        """Corrupt one receiver output. Returns (position or None, reported sigma)."""
        if not self.active(t):
            return z, sigma
        tau = t - self.start
        d = np.asarray(self.direction, dtype=float)
        d = d / np.linalg.norm(d)
        if self.kind == "jamming":
            return None, None
        if self.kind == "multipath":
            return z + self.magnitude * d, sigma
        if self.kind == "float":
            # Honest degradation: decimetre-level correlated error, and the receiver says so.
            a = math.exp(-0.1 / 20.0)
            memory["bias"] = a * memory.get("bias", np.zeros(2)) + math.sqrt(1 - a * a) * 0.15 * rng.normal(size=2)
            return z + memory["bias"] + rng.normal(0.0, 0.08, 2), 0.30
        if self.kind == "jump_spoof":
            return z + self.magnitude * d, sigma
        if self.kind == "drift_spoof":
            return z + self.drift(tau) * d, sigma
        raise ValueError(self.kind)

    def apply_baseline(self, t, b, sigma_b, rng, memory):
        """Corrupt one moving-base (antenna-to-antenna) solution. Returns (vector or None, sigma)."""
        if not self.active(t):
            return b, sigma_b
        if self.kind == "jamming":
            return None, None
        if self.kind == "multipath":
            # Each antenna sees its own reflections, so the baseline is distorted too. Modelled
            # as a 6 cm error in a fixed direction for the episode (an assumption, not a measurement).
            if "mp_dir" not in memory:
                ang = rng.uniform(0.0, 2.0 * math.pi)
                memory["mp_dir"] = np.array([math.cos(ang), math.sin(ang)])
            return b + 0.06 * memory["mp_dir"], sigma_b
        if self.kind == "float":
            # The 0.2 m antenna-to-antenna baseline keeps its fix: errors common to both antennas
            # (atmosphere, corrections) cancel over so short a distance.
            return b, sigma_b
        if self.is_attack:
            if self.array == "single":
                return rng.normal(0.0, sigma_b, 2), sigma_b    # both antennas solve to the same point
            if self.array == "frozen":
                memory.setdefault("frozen", np.array(b, dtype=float))
                return memory["frozen"] + rng.normal(0.0, sigma_b, 2), sigma_b
            if self.array == "tracking":
                return b, sigma_b
            raise ValueError(self.array)
        raise ValueError(self.kind)


def standard_scenarios():
    return [
        Scenario("Nominal"),
        Scenario("Multipath step (1.5 m, 10 s)", "multipath", duration=10.0, magnitude=1.5, direction=(0.6, 0.8)),
        Scenario("Honest RTK float (receiver reports 0.3 m, 30 s)", "float", duration=30.0),
        Scenario("Jamming (20 s outage)", "jamming", duration=20.0),
        Scenario("Crude spoof (8 m jump)", "jump_spoof", magnitude=8.0),
        Scenario("Drift spoof, aggressive (0.2 m/s²)", "drift_spoof", magnitude=0.2),
        Scenario("Drift spoof, moderate (0.1 m/s²)", "drift_spoof", magnitude=0.1),
        Scenario("Drift spoof, gentle (0.03 m/s²)", "drift_spoof", magnitude=0.03),
        Scenario("Gentle drift spoof from an array replaying a frozen baseline", "drift_spoof",
                 magnitude=0.03, array="frozen"),
        Scenario("Gentle drift spoof from an attitude-aware array", "drift_spoof",
                 magnitude=0.03, array="tracking"),
    ]
