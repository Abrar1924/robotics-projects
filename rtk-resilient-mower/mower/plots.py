"""Figures for the README (matplotlib, headless)."""
from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Circle, Patch, PathPatch  # noqa: E402
from matplotlib.path import Path  # noqa: E402
from shapely.geometry.polygon import orient  # noqa: E402

from .gnss import FIXED, FLOAT, NONE  # noqa: E402
from .planner import MOW, TRANSIT, polygons  # noqa: E402
from .supervisor import Mode  # noqa: E402
from .world import CANOPY_DEEP, CANOPY_LIGHT, MULTIPATH  # noqa: E402

GRASS, EDGE = "#eef5e9", "#5b8c4a"
ZONE_STYLE = {MULTIPATH: ("#f4a259", 0.30, "Multipath (near walls)"),
              CANOPY_LIGHT: ("#8ecae6", 0.35, "Light canopy (mostly float)"),
              CANOPY_DEEP: ("#1d6fa5", 0.45, "Dense canopy (no fix)")}
MODE_STYLE = {Mode.NOMINAL: ("#2d6a4f", "Nominal"), Mode.DEGRADED: ("#e9a23b", "Degraded (slower)"),
              Mode.HOLD: ("#d62828", "Hold"), Mode.RETREAT: ("#7b2cbf", "Re-acquire RTK")}
QUALITY_STYLE = {FIXED: ("#2d6a4f", "RTK fixed"), FLOAT: ("#e9a23b", "RTK float"),
                 NONE: ("#d62828", "No fix")}
LABEL = {"oracle": "Ground-truth pose (reference)", "gnss_only": "GNSS only",
         "ekf": "EKF fusion", "resilient": "EKF + graceful degradation"}
SHORT = {"oracle": "Ground truth\n(reference)", "gnss_only": "GNSS only",
         "ekf": "EKF fusion", "resilient": "EKF + graceful\ndegradation"}


def _patch(geom, **kw):
    paths = []
    for poly in polygons(geom):
        poly = orient(poly, sign=1.0)
        for ring in (poly.exterior, *poly.interiors):
            xy = np.asarray(ring.coords)
            codes = [Path.MOVETO] + [Path.LINETO] * (len(xy) - 2) + [Path.CLOSEPOLY]
            paths.append(Path(xy, codes))
    return PathPatch(Path.make_compound_path(*paths), **kw) if paths else None


def _add(ax, geom, **kw):
    if geom is None or geom.is_empty:
        return
    patch = _patch(geom, **kw)
    if patch is not None:
        ax.add_patch(patch)


def _yard(ax, yard, faint=False):
    _add(ax, yard.boundary, facecolor=GRASS, edgecolor=EDGE, lw=1.2, zorder=0)
    for z in yard.zones:
        color, alpha, _ = ZONE_STYLE[z.kind]
        _add(ax, z.geometry, facecolor=color, alpha=alpha * (0.45 if faint else 1.0),
             edgecolor="none", zorder=1)
    for name, geom in yard.keep_out.items():
        color = "#a3a8ad" if name == "house" else "#c9a27c" if "bed" in name else "#6b4f2a"
        _add(ax, geom, facecolor=color, edgecolor="#4d4d4d", lw=0.6, zorder=4)
    hx, hy = yard.keep_out["house"].centroid.coords[0]
    ax.text(hx, hy, "House", ha="center", va="center", fontsize=9, color="#333333", zorder=5)
    minx, miny, maxx, maxy = yard.boundary.bounds
    ax.set_xlim(minx - 0.8, maxx + 0.8)
    ax.set_ylim(miny - 0.8, maxy + 0.8)
    ax.set_aspect("equal")
    ax.set_xlabel("East (m)", fontsize=8)
    ax.set_ylabel("North (m)", fontsize=8)
    ax.tick_params(labelsize=7)


def _segments(xy, mask):
    m = mask[1:]
    return np.stack([xy[:-1][m], xy[1:][m]], axis=1)


def scenario(yard, plan, path):
    fig, ax = plt.subplots(figsize=(9.0, 5.6))
    _yard(ax, yard)
    ax.add_collection(LineCollection(_segments(plan.xy, plan.kind == MOW), color="#1b4332", lw=0.6, zorder=3))
    ax.add_collection(LineCollection(_segments(plan.xy, plan.kind == TRANSIT), color="#6c757d", lw=0.9,
                                     linestyle=(0, (2, 2)), zorder=3))
    x, y, th = yard.dock
    ax.plot(x, y, marker=(3, 0, np.degrees(th) - 90), color="black", ms=10, zorder=6)
    handles = [Patch(fc=c, alpha=a, label=l) for c, a, l in ZONE_STYLE.values()]
    handles += [Patch(fc="#6b4f2a", label="Keep-out: trunks, flower bed, house"),
                Line2D([], [], color="#1b4332", lw=1, label="Mowing path"),
                Line2D([], [], color="#6c757d", lw=1, ls="--", label="Transit"),
                Line2D([], [], marker="^", color="black", ls="", label="Dock")]
    ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=8, frameon=False)
    ax.set_title(f"Simulated yard, RTK sky-view zones and coverage plan "
                 f"({plan.length:.0f} m, sweep angle {plan.angle_deg:.0f}°)", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def comparison(yard, results, path, policies=("gnss_only", "ekf", "resilient")):
    mowable = yard.mowable_area()
    fig, axes = plt.subplots(1, len(policies), figsize=(5.4 * len(policies), 4.6))
    for ax, name in zip(axes, policies):
        r = results[name]
        _yard(ax, yard, faint=True)
        missed = mowable.difference(r.covered)
        if r.deferred is not None and not r.deferred.is_empty:
            missed = missed.difference(r.deferred)
            _add(ax, r.deferred, facecolor="#e2d4f7", edgecolor="#7b2cbf", hatch="////", lw=0.4, zorder=2)
        _add(ax, missed, facecolor="#e63946", alpha=0.7, edgecolor="none", zorder=2)
        xy = r.true[::4, :2]
        if name == "resilient":
            colors = [MODE_STYLE[Mode(m)][0] for m in r.mode[::4][1:]]
            ax.add_collection(LineCollection(np.stack([xy[:-1], xy[1:]], axis=1), colors=colors, lw=0.5, zorder=3))
        else:
            ax.plot(xy[:, 0], xy[:, 1], color="#1b4332", lw=0.45, zorder=3)
        if r.violations:
            ax.plot(r.violation_points[:, 0], r.violation_points[:, 1], "X", color="#d62828",
                    mec="white", mew=0.8, ms=9, zorder=7)
        plural = "s" if r.violations != 1 else ""
        ax.set_title(f"{LABEL[name]}\n{r.coverage * 100:.1f}% mowed · {r.violations} contact event{plural}",
                     fontsize=10)
    handles = [Patch(fc="#e63946", alpha=0.7, label="Missed grass"),
               Patch(fc="#e2d4f7", ec="#7b2cbf", hatch="////", label="Deferred to a later pass (reported)"),
               Line2D([], [], marker="X", color="#d62828", mec="white", ls="", ms=9,
                      label="Contact with obstacle or lawn edge")]
    handles += [Line2D([], [], color=c, lw=2, label=f"Mode: {l}") for c, l in MODE_STYLE.values()]
    fig.legend(handles=handles, loc="lower center", ncol=4, fontsize=8, frameon=False)
    fig.tight_layout(rect=(0, 0.1, 1, 1))
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def timeline(result, path):
    r, t = result, result.t / 60.0
    fig, axes = plt.subplots(3, 1, figsize=(10.5, 6.4), sharex=True,
                             gridspec_kw=dict(height_ratios=[2.2, 1.0, 1.4]))
    ax = axes[0]
    ax.fill_between(t, 0, 3 * r.sigma, color="#adb5bd", alpha=0.6, lw=0, label="EKF 3σ uncertainty")
    ax.plot(t, r.loc_error, color="#1d3557", lw=0.7, label="Actual position error")
    ax.set_ylabel("Position error (m)", fontsize=9)
    ax.set_ylim(0, max(0.6, float(np.percentile(r.loc_error, 99.9)) * 1.25))
    ax.legend(loc="upper right", fontsize=8, frameon=False)
    ax = axes[1]
    for q, (c, label) in QUALITY_STYLE.items():
        ax.fill_between(t, 0.55, 1.0, where=r.quality == q, color=c, step="post", lw=0, label=label)
    for m, (c, label) in MODE_STYLE.items():
        ax.fill_between(t, 0.0, 0.45, where=r.mode == int(m), color=c, step="post", lw=0, alpha=0.85,
                        label=f"Mode: {label}")
    ax.set_yticks([0.225, 0.775], ["Mode", "RTK"], fontsize=8)
    ax.set_ylim(0, 1)
    ax.legend(loc="upper left", bbox_to_anchor=(1.0, 1.25), fontsize=7, frameon=False)
    ax = axes[2]
    ax.plot(t, r.speed, color="#2d6a4f", lw=0.5)
    ax.set_ylabel("Speed command (m/s)", fontsize=9)
    ax.set_xlabel("Mission time (min)", fontsize=9)
    for a in axes:
        a.tick_params(labelsize=8)
    fig.suptitle("Resilient policy: speed and stops follow position uncertainty (seed 0)", fontsize=11)
    fig.tight_layout(rect=(0, 0, 0.84, 0.97))
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def summary(rows, path):
    names = [n for n in ("oracle", "gnss_only", "ekf", "resilient") if rows.get(n)]
    x = np.arange(len(names))
    cov = [100 * np.array([r["coverage"] for r in rows[n]]) for n in names]
    dfr = [np.array([r["deferred_pct"] for r in rows[n]]) for n in names]
    viol = [np.array([r["violations"] for r in rows[n]]) for n in names]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4.0))
    a1.bar(x, [c.mean() for c in cov], width=0.6, color="#2d6a4f", label="Mowed")
    a1.bar(x, [d.mean() for d in dfr], width=0.6, bottom=[c.mean() for c in cov], color="#c3a6ee",
           label="Deferred (reported, not missed)")
    for i, c in enumerate(cov):
        a1.text(i, c.mean() / 2, f"{c.mean():.1f}%", ha="center", va="center", color="white", fontsize=8)
    a1.set_xticks(x, [SHORT[n] for n in names], fontsize=8)
    a1.set_ylim(0, 120)
    a1.set_yticks(range(0, 101, 20))
    a1.set_ylabel("% of mowable area", fontsize=9)
    a1.legend(fontsize=8, frameon=False, loc="upper center", ncol=2)
    a1.set_title("Coverage", fontsize=10)
    colors = {"oracle": "#adb5bd", "gnss_only": "#e63946", "ekf": "#f4a259", "resilient": "#2d6a4f"}
    a2.bar(x, [v.mean() for v in viol], width=0.6, color=[colors[n] for n in names])
    rng = np.random.default_rng(0)
    for i, v in enumerate(viol):
        a2.scatter(i + rng.uniform(-0.15, 0.15, len(v)), v, s=9, color="black", zorder=3)
        a2.text(i + 0.33, max(v.mean(), 0.4), f"mean {v.mean():.1f}", va="center", fontsize=7)
    a2.set_xticks(x, [SHORT[n] for n in names], fontsize=8)
    a2.set_ylabel("Contact events per mission", fontsize=9)
    a2.set_title("Safety (dots = individual missions)", fontsize=10)
    fig.suptitle(f"{len(rows[names[0]])} simulated missions per policy", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def animate(yard, result, path, frames=160, fps=14):
    r = result
    fig, ax = plt.subplots(figsize=(7.4, 5.4))
    _yard(ax, yard)
    trail = LineCollection([], lw=0.8, zorder=3)
    ax.add_collection(trail)
    robot = Circle((0, 0), 0.25, fc="#1d3557", ec="white", lw=0.8, zorder=6)
    halo = Circle((0, 0), 0.1, fc="none", ec="#d62828", lw=1.0, ls="--", zorder=6)
    ax.add_patch(robot)
    ax.add_patch(halo)
    title = ax.set_title("", fontsize=9)
    idx = np.linspace(0, len(r.t) - 1, frames).astype(int)
    xy = r.true[:, :2]

    def update(f):
        k = idx[f]
        sub, modes = xy[:k + 1:8], r.mode[:k + 1:8]
        if len(sub) > 1:
            trail.set_segments(np.stack([sub[:-1], sub[1:]], axis=1))
            trail.set_color([MODE_STYLE[Mode(m)][0] for m in modes[1:]])
        robot.center = tuple(xy[k])
        halo.center = tuple(r.est[k, :2])
        halo.radius = max(3.0 * r.sigma[k], 0.05)
        title.set_text(f"t = {r.t[k] / 60:4.1f} min   mode: {MODE_STYLE[Mode(r.mode[k])][1]:<17}"
                       f"   {QUALITY_STYLE[r.quality[k]][1]}   (dashed: EKF 3σ)")
        return trail, robot, halo, title

    FuncAnimation(fig, update, frames=frames).save(path, writer=PillowWriter(fps=fps), dpi=80)
    plt.close(fig)


READINESS_STYLE = {"ready": ("#95d5b2", "Ready: RTK fixed ≥ 95% of the time, full speed"),
                   "degraded": ("#f6bd60", "Degraded: mowed, but slowly"),
                   "at risk": ("#e63946", "At risk: skipped in most missions"),
                   "not visited": ("#ced4da", "Not visited")}


def readiness_map(yards, reports, path):
    """One panel per yard variant: cells coloured by readiness class, problem zones numbered."""
    fig, axes = plt.subplots(1, len(reports), figsize=(7.2 * len(reports), 6.4), squeeze=False)
    for ax, (name, rep) in zip(axes[0], reports.items()):
        yard = yards[name]
        _add(ax, yard.boundary, facecolor="white", edgecolor=EDGE, lw=1.2, zorder=0)
        for cell, k in zip(rep.cells, rep.klass):
            _add(ax, cell, facecolor=READINESS_STYLE[k][0], edgecolor="white", lw=0.4, zorder=1)
        for z in yard.zones:
            _add(ax, z.geometry, facecolor="none", edgecolor="#1d3557", lw=0.7, ls=(0, (3, 2)), zorder=2)
        for nm, geom in yard.keep_out.items():
            color = "#a3a8ad" if nm == "house" else "#c9a27c" if "bed" in nm else "#6b4f2a"
            _add(ax, geom, facecolor=color, edgecolor="#4d4d4d", lw=0.6, zorder=4)
        hx, hy = yard.keep_out["house"].centroid.coords[0]
        ax.text(hx, hy, "House", ha="center", va="center", fontsize=9, color="#333333", zorder=5)
        for i, zone in enumerate(rep.zones, 1):
            _add(ax, zone.geometry, facecolor="none", edgecolor="#6a040f", lw=1.6, zorder=6)
            if zone.area >= 1.0:
                p = zone.geometry.representative_point()
                ax.text(p.x, p.y, str(i), ha="center", va="center", fontsize=8, fontweight="bold",
                        color="white", zorder=7,
                        bbox=dict(boxstyle="circle,pad=0.2", fc="#6a040f", ec="none"))
        minx, miny, maxx, maxy = yard.boundary.bounds
        ax.set_xlim(minx - 0.8, maxx + 0.8)
        ax.set_ylim(miny - 0.8, maxy + 0.8)
        ax.set_aspect("equal")
        ax.set_xlabel("East (m)", fontsize=8)
        ax.set_ylabel("North (m)", fontsize=8)
        ax.tick_params(labelsize=7)
        shares = " · ".join(f"{100 * rep.share(k):.0f}% {k}" for k in ("ready", "degraded", "at risk"))
        ax.set_title(f"{name}: {rep.verdict}\n{shares}", fontsize=10)
    handles = [Patch(fc=c, label=l) for c, l in READINESS_STYLE.values()]
    handles += [Patch(fc="none", ec="#6a040f", lw=1.6, label="Problem zone (numbered in the report)"),
                Line2D([], [], color="#1d3557", lw=0.8, ls=(0, (3, 2)), label="Sky-view zone (canopy, walls)")]
    fig.legend(handles=handles, loc="lower center", ncol=3, fontsize=8, frameon=False, bbox_to_anchor=(0.5, 0.0))
    fig.suptitle("RTK readiness survey: where the mower can work on its own, and where it needs help",
                 fontsize=11, y=0.99)
    fig.tight_layout(rect=(0, 0.13, 1, 0.94))
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
