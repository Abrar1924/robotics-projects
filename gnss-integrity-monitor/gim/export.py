"""Write simulated flights as plain-text sensor logs that the C++ monitor replays.

One line per input the dual-antenna tests consume, in the order they arrive:

  H heading0                                   initial heading given to the heading filter (rad)
  G t rate dt                                  yaw-rate gyro sample (rad/s) over dt seconds
  E t has_b bx by sigma_b vx vy c00 c01 c11 d  one GNSS epoch: moving-base vector (m) and its
                                               per-axis sigma, the navigation filter's velocity
                                               (m/s) with its covariance, and the commanded
                                               drift angle (rad). has_b is 0 when the receiver
                                               produced no baseline.

Angles are measured from the x axis towards the y axis (east towards north in ENU).

    python -m gim.export            # rewrites ros2/replay/*.txt and ros2/replay/expected.json
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .detectors import DUAL, dual_alarms, first_alarm
from .faults import Scenario
from .sim import fly

REPLAY_DIR = Path(__file__).resolve().parents[1] / "ros2" / "replay"
START, DURATION, SEED = 20.0, 70.0, 3


def replay_cases():
    """The four flights replayed through the C++ monitor: one healthy, one per spoofer geometry."""
    gentle = dict(kind="drift_spoof", magnitude=0.03, start=START)
    return [
        ("clean", Scenario("Nominal")),
        ("single_transmitter", Scenario("Gentle drift spoof, one transmit antenna", array="single", **gentle)),
        ("frozen_baseline", Scenario("Gentle drift spoof, array replaying a frozen baseline", array="frozen", **gentle)),
        ("attitude_aware", Scenario("Gentle drift spoof, attitude-aware array", array="tracking", **gentle)),
    ]


def record(scenario, seed=SEED, duration=DURATION):
    """Fly once without protection. Returns (flight log, tapped inputs)."""
    tap = []
    log = fly(scenario, seed=seed, duration=duration, tap=tap)
    return log, tap


def lines(tap, fmt="{!r}"):
    """Tapped inputs as text lines. `fmt` formats each number ("{!r}" keeps every digit)."""
    f = lambda x: fmt.format(float(x))  # noqa: E731
    out = []
    for item in tap:
        if item[0] == "H":
            out.append(f"H {f(item[1])}")
        elif item[0] == "G":
            _, t, rate, dt = item
            out.append(f"G {f(t)} {f(rate)} {f(dt)}")
        else:
            _, t, b, sigma_b, v, P, drift = item
            if b is None:
                bx = by = sb = 0.0
            else:
                (bx, by), sb = b, sigma_b
            out.append(" ".join(["E", f(t), "0" if b is None else "1", f(bx), f(by), f(sb), f(v[0]), f(v[1]),
                                 f(P[0, 0]), f(P[0, 1]), f(P[1, 1]), f(drift)]))
    return out


def expected(log):
    """What the Python tests decided on this flight: first alarm of each test, and the first overall."""
    alarms = dual_alarms(log)
    first = {name: first_alarm(log.ep_t, alarms[name], 0.0) for name in DUAL}
    seen = {k: v for k, v in first.items() if np.isfinite(v)}
    source = min(seen, key=lambda k: (seen[k], DUAL.index(k))) if seen else ""
    return {"scenario": log.scenario.name, "attack_start": log.scenario.start if log.scenario.is_attack else None,
            "epochs": int(len(log.ep_t)),
            "first_alarm": {k: (round(v, 3) if np.isfinite(v) else None) for k, v in first.items()},
            "alarm_source": source, "alarm_time": round(seen[source], 3) if source else None}


def main():
    REPLAY_DIR.mkdir(parents=True, exist_ok=True)
    summary = {}
    for slug, scenario in replay_cases():
        log, tap = record(scenario)
        (REPLAY_DIR / f"{slug}.txt").write_text("\n".join(lines(tap, fmt="{:.9g}")) + "\n")
        summary[slug] = expected(log)
        print(f"{slug}: {summary[slug]['alarm_source'] or 'no alarm'}"
              + (f" at {summary[slug]['alarm_time']} s" if summary[slug]["alarm_source"] else ""))
    (REPLAY_DIR / "expected.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"wrote {REPLAY_DIR}")


if __name__ == "__main__":
    main()
