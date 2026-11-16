"""Replay a real receiver log through the same speed rule as the yard.

The file is a u-blox F9K drive in Mountain View on 2020-02-06, published with
Google's gps-measurement-tools. It is a car on public roads, not a mower in a
yard. What it does show is how a real RTK receiver behaves once it leaves open
sky: how often the fix is lost, and for how long.

The receiver reports RTK fixed (quality 4), RTK float (quality 5) and
differential (quality 2). It does not say whether the car was under a tree or
beside a wall, and it does not include an error in metres. Those two gaps are
filled in here, and the result says so.

  fixed        -> 2 cm, the centimetre class a fixed solution is
  float        -> 15 cm, the decimetre class a float solution is
  differential -> 50 cm
  other        -> 1 m

The rule is then run twice on that same quality sequence, with the receiver's
own time stamps: once with 5 m of room, as in open grass, and once with 30 cm
of room, as beside a wall.

    python -m mower.replay_nmea            # print the numbers
    python -m mower.replay_nmea --write    # also rewrite results/real_gps.md and .png
"""
from __future__ import annotations

import math
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

from mower.gnss import FIXED, FLOAT
from mower.supervisor import Mode, Supervisor, SupervisorConfig

DEFAULT_LOG = Path("data/MTV.Local1.ublox-F9K.20200206-181434.nmea")
SIGMA = {4: 0.02, 5: 0.15, 2: 0.50}
DEFAULT_SIGMA = 1.0
QUALITY_NAME = {4: "RTK fixed", 5: "RTK float", 2: "differential"}
CASES = (("5 m, open grass", 5.0), ("0.30 m, beside a wall", 0.30))


@dataclass(frozen=True)
class Reading:
    t: float        # s since the first reading, from the receiver's own clock
    lat: float      # degrees
    lon: float      # degrees
    quality: int    # NMEA GGA quality indicator


def read_log(path=DEFAULT_LOG):
    """Every GGA sentence that carries a position and a quality code."""
    readings, t0, day = [], None, 0.0
    for line in Path(path).read_text(errors="replace").splitlines():
        if "GGA" not in line[:8]:
            continue
        f = line.split(",")
        if len(f) < 7 or not f[6].isdigit() or len(f[6]) != 1 or len(f[1]) < 6 or not f[2] or not f[4]:
            continue
        tod = int(f[1][:2]) * 3600 + int(f[1][2:4]) * 60 + float(f[1][4:])
        if t0 is None:
            t0 = tod
        if readings and tod + day - t0 < readings[-1].t:   # the log crossed midnight UTC
            day += 86400.0
        lat = (int(f[2][:2]) + float(f[2][2:]) / 60.0) * (-1.0 if f[3] == "S" else 1.0)
        lon = (int(f[4][:3]) + float(f[4][3:]) / 60.0) * (-1.0 if f[5] == "W" else 1.0)
        readings.append(Reading(tod + day - t0, lat, lon, int(f[6])))
    return readings


def read_qualities(path=DEFAULT_LOG):
    return [r.quality for r in read_log(path)]


def quality_code(gga):
    if gga == 4:
        return FIXED
    if gga in (5, 2):
        return FLOAT
    return 0


def route_xy(readings):
    """East and north of the first reading, in metres (flat-earth, fine over a few km)."""
    lat0, lon0 = readings[0].lat, readings[0].lon
    k = math.cos(math.radians(lat0))
    return [((r.lon - lon0) * 111320.0 * k, (r.lat - lat0) * 110574.0) for r in readings]


def fix_losses(readings):
    """Every stretch without an RTK fixed solution, as (start time, seconds until fixed again).

    A stretch runs from the first reading that is not fixed to the next fixed reading, so it
    includes seconds in which the receiver printed nothing at all.
    """
    out, start = [], None
    for r in readings:
        if r.quality != 4 and start is None:
            start = r.t
        elif r.quality == 4 and start is not None:
            out.append((start, r.t - start))
            start = None
    if start is not None:
        out.append((start, readings[-1].t - start + 1.0))
    return out


def modes(readings, clearance):
    """The supervisor's mode at every reading, with the receiver's own time stamps."""
    sup = Supervisor()
    return [sup.update(r.t, quality_code(r.quality), SIGMA.get(r.quality, DEFAULT_SIGMA), clearance)
            for r in readings]


def run(readings, clearance):
    """Mode counts and the number of readings at which the mower stands still (HOLD)."""
    if readings and not isinstance(readings[0], Reading):   # a plain list of quality codes, 1 s apart
        readings = [Reading(float(i), 0.0, 0.0, q) for i, q in enumerate(readings)]
    seq = modes(readings, clearance)
    counts = Counter(Mode(m).name for m in seq)
    return counts, counts["HOLD"]


def summary(readings):
    losses = [d for _, d in fix_losses(readings)]
    timeout = SupervisorConfig().hold_timeout
    xy = route_xy(readings)
    steps = [math.dist(a, b) for a, b in zip(xy, xy[1:])]
    return {
        "readings": len(readings),
        "duration_s": readings[-1].t,
        "route_km": sum(steps) / 1000.0,
        "quality": dict(sorted(Counter(r.quality for r in readings).items())),
        "fix_losses": len(losses),
        "longest_loss_s": max(losses),
        "median_loss_s": sorted(losses)[len(losses) // 2],
        "losses_over_timeout": sum(1 for d in losses if d > timeout),
        "seconds_in_long_losses": sum(d for d in losses if d > timeout),
        "hold_timeout_s": timeout,
        "longest_gap_s": max(b.t - a.t for a, b in zip(readings, readings[1:])),
        "cases": {name: dict(run(readings, c)[0]) for name, c in CASES},
    }


def markdown(path, s):
    q = s["quality"]
    lines = [
        "# Real receiver replay", "",
        f"File: `{path.as_posix()}`", "",
        "Source: a u-blox F9K on a car in Mountain View, 6 February 2020, published in Google's "
        "gps-measurement-tools.", "",
        f"https://github.com/google/gps-measurement-tools/blob/master/NmeaUtils/example/{path.name}", "",
        "This is a car on public roads, not a mower in a yard. It is here because it is a real RTK "
        "receiver leaving open sky, which the simulated yard can only assume.", "",
        "## What the receiver did", "",
        f"{s['readings']:,} readings over {s['duration_s'] / 60:.0f} minutes and about {s['route_km']:.1f} km. "
        f"The receiver reported {q.get(4, 0)} RTK fixed, {q.get(5, 0)} RTK float and {q.get(2, 0)} differential "
        f"readings, so it held a fixed solution for {100 * q.get(4, 0) / s['readings']:.0f}% of them.", "",
        f"It lost the fixed solution {s['fix_losses']} times. Half of the losses were over within "
        f"{s['median_loss_s']:.0f} s. {s['losses_over_timeout']} of them lasted longer than the "
        f"{s['hold_timeout_s']:.0f} s the supervisor waits before it gives up on a spot, and the longest "
        f"lasted {s['longest_loss_s']:.0f} s. Those long losses add up to "
        f"{s['seconds_in_long_losses'] / 60:.1f} minutes of the drive. "
        f"The longest silence, with no output at all, was {s['longest_gap_s']:.0f} s.", "",
        "![Route coloured by fix quality, and the supervisor's answer](real_gps.png)", "",
        "## What the supervisor does with it", "",
        "The file does not label trees or walls, and it does not give the error in metres. The replay "
        "uses 2 cm for a fixed reading, 15 cm for a float reading and 50 cm for a differential reading. "
        "The gap to a hazard is then assumed in two ways.", "",
        "| Room around the mower | Full speed | Slow | Stopped | Retreating to the last fixed spot |",
        "|---|---|---|---|---|",
    ]
    for name, _ in CASES:
        c = s["cases"][name]
        lines.append(f"| {name} | {c.get('NOMINAL', 0)} | {c.get('DEGRADED', 0)} | {c.get('HOLD', 0)} | "
                     f"{c.get('RETREAT', 0)} |")
    lines += ["",
              f"Each row adds up to {s['readings']:,} readings. Every fixed reading runs at full speed. With 5 m "
              "of room most float readings only slow the mower; with 30 cm of room they stop it. The same "
              "receiver output is safe to keep working on in one place and not in the other, which is why "
              "the stop rule depends on the clearance and not on the fix type alone.", ""]
    return "\n".join(lines)


def figure(readings, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    q_color = {4: "#2a9d8f", 5: "#e9a23b", 2: "#d1495b"}
    m_color = {Mode.NOMINAL: "#2a9d8f", Mode.DEGRADED: "#e9a23b", Mode.HOLD: "#d1495b", Mode.RETREAT: "#6d597a"}
    xy = route_xy(readings)
    t = [r.t / 60.0 for r in readings]
    fig = plt.figure(figsize=(11, 6.4))
    grid = fig.add_gridspec(3, 2, width_ratios=(1.0, 1.25), hspace=0.55, wspace=0.16)
    ax = fig.add_subplot(grid[:, 0])
    ax.plot([p[0] for p in xy], [p[1] for p in xy], color="#ced4da", lw=0.8, zorder=1)
    for code in (4, 5, 2):
        pts = [p for p, r in zip(xy, readings) if r.quality == code]
        ax.scatter([p[0] for p in pts], [p[1] for p in pts], s=5, color=q_color[code], zorder=2)
    ax.set_aspect("equal")
    ax.set_xlabel("East (m)", fontsize=8)
    ax.set_ylabel("North (m)", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.set_title("Where the receiver held RTK fixed, and where it did not", fontsize=9)
    ax.legend(handles=[Line2D([], [], marker="o", ls="", ms=4, color=q_color[c], label=QUALITY_NAME[c])
                       for c in (4, 5, 2)], fontsize=7, frameon=False, loc="lower left")

    rows = [("Receiver: fix quality", [q_color.get(r.quality, "#adb5bd") for r in readings])]
    for name, clearance in CASES:
        rows.append((f"Supervisor with {name}", [m_color[Mode(m)] for m in modes(readings, clearance)]))
    for i, (title, colors) in enumerate(rows):
        strip = fig.add_subplot(grid[i, 1])
        strip.bar(t, [1.0] * len(t), width=1.2 / 60.0, color=colors, linewidth=0)
        strip.set_xlim(0.0, t[-1])
        strip.set_ylim(0.0, 1.0)
        strip.set_yticks([])
        strip.tick_params(labelsize=7)
        strip.set_title(title, fontsize=9, loc="left")
        if i == len(rows) - 1:
            strip.set_xlabel("Minutes into the drive (white: no receiver output)", fontsize=8)
    fig.legend(handles=[Patch(fc=m_color[m], label=label) for m, label in (
        (Mode.NOMINAL, "Full speed"), (Mode.DEGRADED, "Slow"), (Mode.HOLD, "Stopped"),
        (Mode.RETREAT, "Retreating to the last fixed spot"))],
        fontsize=7, frameon=False, ncol=4, loc="lower right", bbox_to_anchor=(0.9, 0.0))
    fig.suptitle("A real RTK receiver away from open sky (u-blox F9K, car, Mountain View, public log)", fontsize=10)
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    path = Path(args[0]) if args else DEFAULT_LOG
    readings = read_log(path)
    s = summary(readings)
    print(f"file: {path.name}")
    print(f"readings: {s['readings']} over {s['duration_s']:.0f} s, about {s['route_km']:.1f} km")
    print("receiver quality codes:", s["quality"])
    print(f"fixed solution lost {s['fix_losses']} times; longest {s['longest_loss_s']:.0f} s; "
          f"{s['losses_over_timeout']} longer than {s['hold_timeout_s']:.0f} s "
          f"({s['seconds_in_long_losses']:.0f} s in total)")
    for name, _ in CASES:
        print(f"{name}: {s['cases'][name]}")
    if "--write" in sys.argv:
        out = Path("results")
        out.mkdir(exist_ok=True)
        (out / "real_gps.md").write_text(markdown(path, s))
        figure(readings, out / "real_gps.png")
        print("wrote results/real_gps.md and results/real_gps.png")


if __name__ == "__main__":
    main()
