"""RTK readiness report: will an RTK mower work in this yard, and where will it need help?

An RTK mower sold into a yard it can't handle turns into a support ticket or a return. A dealer
or installer needs a quick answer before the sale: what share of the lawn the robot can mow on
its own, where it will slow down, which zones it will skip, and what would fix them.

This module turns mission logs (the robot's track and the receiver's fix type at every step)
into that answer:

  1. Grid the mowable area into 1 m cells.
  2. Per cell: share of time with RTK fixed, and share of missions in which the robot deferred it.
  3. Classify each cell: ready (full speed), degraded (mowed, but slowly), at risk (the robot
     skips it in most missions, so it needs a mitigation), or not visited.
  4. Merge at-risk cells into problem zones, name the cause and suggest a mitigation.
  5. Give the yard a verdict from the share of the lawn at risk.

In the field the track would come from the robot's own estimate and the cause from receiver
diagnostics (satellite count, signal strength, fix type). In this simulation the track is the
true path and the cause is the simulated sky-view zone, so treat causes as labels, not inference.
The verdict thresholds are product decisions, set here as placeholders.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import shapely
from shapely.geometry import Point
from shapely.ops import unary_union

from .gnss import FIXED, FLOAT, NONE
from .world import CANOPY_DEEP, CANOPY_LIGHT, MULTIPATH, OPEN, SEVERITY

READY, DEGRADED, AT_RISK, UNSEEN = "ready", "degraded", "at risk", "not visited"
CLASSES = (READY, DEGRADED, AT_RISK, UNSEEN)

CAUSE = {
    CANOPY_DEEP: "dense canopy: no RTK solution at all",
    CANOPY_LIGHT: "light canopy: mostly RTK float, brief fixed windows",
    MULTIPATH: "wall reflections: float fixes biased away from the wall",
    OPEN: "open sky, but RTK still re-converging after a canopy or wall zone (4–10 s)",
}
ADVICE = {
    CANOPY_DEEP: "Add camera or LiDAR navigation here, a short boundary-wire island, prune the canopy, "
                 "or leave it for hand mowing.",
    CANOPY_LIGHT: "The robot slows to 0.2 m/s and skips strips next to trunks; camera assist would recover them.",
    MULTIPATH: "Keep the wider wall margin, or add a camera check for the strip along the wall.",
    OPEN: "Plan lanes so the robot exits degraded zones into open lawn, not toward a hazard.",
}


@dataclass(frozen=True)
class ReadinessConfig:
    cell: float = 1.0             # m: grid resolution
    min_seconds: float = 2.0      # time a cell needs across all missions to count as visited
    ready_fix: float = 0.95       # share of time with RTK fixed for a cell to count as ready
    risk_deferred: float = 0.50   # deferred in at least this share of missions: at risk
    deferred_cover: float = 0.30  # a cell counts as deferred in a mission if this share of it was deferred
    min_zone: float = 0.5         # m²: smaller at-risk fragments are counted but not listed as zones
    go_risk: float = 0.03         # verdict: GO if at most 3% of the lawn is at risk (placeholder)
    mitigate_risk: float = 0.15   # GO WITH MITIGATION up to 15%, NO-GO above (placeholder)


@dataclass
class ProblemZone:
    geometry: object
    area: float
    cause: str
    near: str
    advice: str


@dataclass
class Readiness:
    cells: object                 # shapely polygons, one per cell (clipped to the mowable area)
    area: np.ndarray              # m² of mowable grass in each cell
    fix_rate: np.ndarray          # share of time with RTK fixed (NaN if never visited)
    deferred_share: np.ndarray    # share of missions that deferred the cell
    klass: np.ndarray             # one of CLASSES per cell
    zones: list = field(default_factory=list)
    missions: int = 0
    mission_minutes: tuple = (0.0, 0.0, 0.0)   # mean, min, max
    unreported_missed: float = 0.0              # share of the lawn neither mowed nor reported deferred
    cfg: ReadinessConfig = field(default_factory=ReadinessConfig)

    @property
    def total(self):
        return float(self.area.sum())

    def share(self, klass):
        return float(self.area[self.klass == klass].sum()) / self.total

    @property
    def verdict(self):
        risk = self.share(AT_RISK)
        if risk <= self.cfg.go_risk:
            return "GO"
        return "GO WITH MITIGATION" if risk <= self.cfg.mitigate_risk else "NO-GO"


def _grid(yard, cfg):
    mowable = yard.mowable_area()
    minx, miny, maxx, maxy = mowable.bounds
    xs = np.arange(np.floor(minx), maxx, cfg.cell)
    ys = np.arange(np.floor(miny), maxy, cfg.cell)
    X, Y = np.meshgrid(xs, ys)
    boxes = shapely.box(X.ravel(), Y.ravel(), X.ravel() + cfg.cell, Y.ravel() + cfg.cell)
    cells = shapely.intersection(boxes, mowable)
    area = shapely.area(cells)
    keep = area > 0.05 * cfg.cell * cfg.cell
    xe = np.r_[xs, xs[-1] + cfg.cell]
    ye = np.r_[ys, ys[-1] + cfg.cell]
    return cells[keep], area[keep], keep, xe, ye


def _sky(yard, p):
    """Worst sky-view class at a point (the simulation's ground-truth cause label)."""
    worst = OPEN
    for z in yard.zones:
        if SEVERITY[z.kind] > SEVERITY[worst] and z.geometry.contains(p):
            worst = z.kind
    return worst


def _place(yard, geom, kind):
    """Where the problem zone is, in words: the zone of the dominant sky-view class that
    overlaps it most, or for an open-sky cause the zone it borders most closely."""
    if not yard.zones:
        return "Open lawn"
    if kind == OPEN:
        near = min(yard.zones, key=lambda z: z.geometry.distance(geom))
        return f"Open lawn beside the {near.name}"
    candidates = [z for z in yard.zones if z.kind == kind] or list(yard.zones)
    name = max(candidates, key=lambda z: z.geometry.intersection(geom).area).name
    return f"Along the {name}" if kind == MULTIPATH else f"Under the {name}"


def readiness(yard, missions, cfg=ReadinessConfig()):
    """Build the readiness report from mission results (mower.sim.MissionResult)."""
    cells, area, keep, xe, ye = _grid(yard, cfg)
    counts = {q: np.zeros((len(ye) - 1, len(xe) - 1)) for q in (FIXED, FLOAT, NONE)}
    deferred_hits = np.zeros(len(cells))
    for r in missions:
        x, y, q = r.true[:, 0], r.true[:, 1], r.quality
        for code in counts:
            m = q == code
            counts[code] += np.histogram2d(y[m], x[m], bins=[ye, xe])[0]
        if r.deferred is not None and not r.deferred.is_empty:
            frac = shapely.area(shapely.intersection(cells, r.deferred)) / area
            deferred_hits += frac >= cfg.deferred_cover
    dt = float(np.median(np.diff(missions[0].t)))
    fixed = counts[FIXED].ravel()[keep]
    total = sum(c.ravel()[keep] for c in counts.values())
    seen = total * dt >= cfg.min_seconds
    fix_rate = np.where(seen, fixed / np.maximum(total, 1), np.nan)
    deferred_share = deferred_hits / len(missions)

    klass = np.full(len(cells), UNSEEN, dtype=object)
    klass[seen & (fix_rate >= cfg.ready_fix)] = READY
    klass[seen & (fix_rate < cfg.ready_fix)] = DEGRADED
    klass[deferred_share >= cfg.risk_deferred] = AT_RISK

    zones = []
    merged = unary_union(list(cells[klass == AT_RISK])) if np.any(klass == AT_RISK) else None
    if merged is not None and not merged.is_empty:
        parts = list(getattr(merged, "geoms", [merged]))
        idx_risk = np.flatnonzero(klass == AT_RISK)
        for part in sorted(parts, key=lambda g: -g.area):
            if part.area < cfg.min_zone:
                continue
            inside = [i for i in idx_risk if part.buffer(1e-6).contains(cells[i])]
            causes = [_sky(yard, cells[i].representative_point()) for i in inside] or [OPEN]
            weights = {c: sum(area[i] for i, cc in zip(inside, causes) if cc == c) for c in set(causes)}
            cause = max(weights, key=weights.get)
            zones.append(ProblemZone(geometry=part, area=float(part.area), cause=CAUSE[cause],
                                     near=_place(yard, part, cause), advice=ADVICE[cause]))

    minutes = np.array([r.mission_time / 60.0 for r in missions])
    mowable = yard.mowable_area().area
    missed = [max(0.0, 1.0 - r.coverage - (r.deferred_area / mowable)) for r in missions]
    return Readiness(cells=cells, area=area, fix_rate=fix_rate, deferred_share=deferred_share,
                     klass=klass, zones=zones, missions=len(missions),
                     mission_minutes=(float(minutes.mean()), float(minutes.min()), float(minutes.max())),
                     unreported_missed=float(np.mean(missed)), cfg=cfg)


def report_markdown(name, rep, reference_minutes=None):
    """Plain-language report for a dealer or installer."""
    risk = rep.share(AT_RISK)
    lines = [f"### {name}: {rep.verdict} ({100 * risk:.1f}% of the lawn at risk)", "",
             f"Based on {rep.missions} simulated missions over {rep.total:.0f} m² of mowable lawn.", "",
             "| Class | Area (m²) | Share of lawn | Meaning |", "|---|---|---|---|"]
    meaning = {READY: "RTK fixed at least 95% of the time: mowed at full speed",
               DEGRADED: "Mowed, but slowly: RTK float or no fix for more than 5% of the time",
               AT_RISK: "Skipped and reported in most missions: needs a mitigation",
               UNSEEN: "Never reached by the robot in these missions"}
    for k in CLASSES:
        a = float(rep.area[rep.klass == k].sum())
        lines.append(f"| {k.capitalize()} | {a:.0f} | {100 * a / rep.total:.1f}% | {meaning[k]} |")
    mean, lo, hi = rep.mission_minutes
    extra = f" (+{100 * (mean / reference_minutes - 1):.0f}%)" if reference_minutes else ""
    lines += ["", f"Mission time: {mean:.0f} min ({lo:.0f}–{hi:.0f})"
              + (f", against {reference_minutes:.0f} min with perfect positioning{extra}." if reference_minutes else "."),
              f"Grass neither mowed nor reported as deferred: {100 * rep.unreported_missed:.1f}% "
              "(edges the deck can't reach between passes).", ""]
    if rep.zones:
        lines += ["| Problem zone | Area (m²) | Cause | What would fix it |", "|---|---|---|---|"]
        for i, z in enumerate(rep.zones, 1):
            lines.append(f"| {i}. {z.near} | {z.area:.1f} | {z.cause} | {z.advice} |")
        small = float(rep.area[rep.klass == AT_RISK].sum()) - sum(z.area for z in rep.zones)
        if small > 0.05:
            lines += ["", f"Plus {small:.1f} m² of at-risk fragments under {rep.cfg.min_zone} m² each."]
    else:
        lines.append("No problem zones.")
    return "\n".join(lines) + "\n"


def summary(rep):
    """Machine-readable summary (for results.json)."""
    return {"verdict": rep.verdict, "missions": rep.missions, "lawn_m2": rep.total,
            "share": {k: rep.share(k) for k in CLASSES},
            "mission_minutes": dict(zip(("mean", "min", "max"), rep.mission_minutes)),
            "unreported_missed": rep.unreported_missed,
            "zones": [{"near": z.near, "area_m2": z.area, "cause": z.cause, "advice": z.advice} for z in rep.zones]}


def classify_point(rep, x, y):
    """Class of the cell containing (x, y), or None outside the lawn."""
    p = Point(x, y)
    for c, k in zip(rep.cells, rep.klass):
        if c.buffer(1e-9).contains(p):
            return k
    return None
