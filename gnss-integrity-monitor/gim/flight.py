"""Reference flight: a home-security drone patrolling a property perimeter."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Property:
    width: float = 60.0    # m, east-west
    height: float = 40.0   # m, north-south
    inset: float = 5.0     # m, patrol path distance inside the boundary (the geofence margin)

    def contains(self, xy):
        xy = np.atleast_2d(xy)
        return (np.abs(xy[:, 0]) <= self.width / 2) & (np.abs(xy[:, 1]) <= self.height / 2)


def patrol(prop=Property(), speed=5.0, corner_radius=8.0, duration=180.0, dt=0.02):
    """Constant-speed laps on a rounded rectangle `prop.inset` metres inside the property.

    Returns (t, p, v, a): time and world-frame position, velocity, acceleration (east, north),
    in a local frame centred on the property.
    """
    hw, hh, r = prop.width / 2 - prop.inset, prop.height / 2 - prop.inset, corner_radius
    lx, ly, arc = 2 * (hw - r), 2 * (hh - r), 0.5 * math.pi * r
    segments = [  # (length, kind, geometry): counter-clockwise from the south-west corner
        (lx, "line", ((-hw + r, -hh), (1.0, 0.0))),
        (arc, "arc", ((hw - r, -hh + r), -math.pi / 2)),
        (ly, "line", ((hw, -hh + r), (0.0, 1.0))),
        (arc, "arc", ((hw - r, hh - r), 0.0)),
        (lx, "line", ((hw - r, hh), (-1.0, 0.0))),
        (arc, "arc", ((-hw + r, hh - r), math.pi / 2)),
        (ly, "line", ((-hw, hh - r), (0.0, -1.0))),
        (arc, "arc", ((-hw + r, -hh + r), math.pi)),
    ]
    lengths = np.array([s[0] for s in segments])
    ends = np.cumsum(lengths)
    starts = ends - lengths
    t = np.arange(0.0, duration, dt)
    s = (speed * t) % ends[-1]
    which = np.searchsorted(ends, s, side="right")
    p, v, a = (np.zeros((len(t), 2)) for _ in range(3))
    for k, (_, kind, geom) in enumerate(segments):
        m = which == k
        u = s[m] - starts[k]
        if kind == "line":
            (x0, y0), (dx, dy) = geom
            p[m] = np.c_[x0 + dx * u, y0 + dy * u]
            v[m] = speed * np.array([dx, dy])
        else:
            (cx, cy), th0 = geom
            th = th0 + u / r
            p[m] = np.c_[cx + r * np.cos(th), cy + r * np.sin(th)]
            v[m] = speed * np.c_[-np.sin(th), np.cos(th)]
            a[m] = -(speed**2 / r) * np.c_[np.cos(th), np.sin(th)]
    return t, p, v, a


def lap_time(prop=Property(), speed=5.0, corner_radius=8.0):
    """Seconds for one lap of the patrol path."""
    hw, hh, r = prop.width / 2 - prop.inset, prop.height / 2 - prop.inset, corner_radius
    return (4 * (hw - r) + 4 * (hh - r) + 2 * math.pi * r) / speed
