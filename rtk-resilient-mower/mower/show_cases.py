"""Three GPS situations the ROS 2 node must answer the same way as the supervisor.

Open grass: a fixed reading, far from anything, full speed.
Under a tree: a float reading, still room around the mower, slow down.
Next to a wall: a bad reading closer than the error allows, stop.
"""
from __future__ import annotations

from .gnss import FIXED, FLOAT
from .supervisor import Mode, Supervisor

# NavSatFix status values used on the wire.
GBAS_FIX = 2   # treated as an RTK fixed reading
PLAIN_FIX = 0  # a reading that is not RTK fixed (float / single point)


def situations():
    return [
        {
            "name": "open grass",
            "status": GBAS_FIX,
            "sigma_m": 0.02,
            "clearance_m": 5.0,
            "mode": Mode.NOMINAL,
            "speed_mps": 0.50,
        },
        {
            "name": "under a tree",
            "status": PLAIN_FIX,
            "sigma_m": 0.15,
            "clearance_m": 3.0,
            "mode": Mode.DEGRADED,
        },
        {
            "name": "next to a wall",
            "status": PLAIN_FIX,
            "sigma_m": 0.20,
            "clearance_m": 0.30,
            "mode": Mode.HOLD,
            "speed_mps": 0.0,
        },
    ]


def quality_from_status(status):
    if status >= GBAS_FIX:
        return FIXED
    if status >= 0:
        return FLOAT
    return 0


def decide(status, sigma_m, clearance_m, t=0.0, supervisor=None):
    sup = supervisor or Supervisor()
    mode = sup.update(t, quality_from_status(status), sigma_m, clearance_m)
    return mode, sup.speed_limit, sup.hold_expired
