#!/usr/bin/env python3
"""Benchmark the four navigation policies over many simulated missions and write the
figures and results table to results/."""
from __future__ import annotations

import argparse
import json
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from mower import plots
from mower.planner import plan_coverage
from mower.sim import POLICIES, run_mission
from mower.world import suburban_yard

YARD = suburban_yard()
PLAN = plan_coverage(YARD.free_space(), YARD.deck_width, start=YARD.dock[:2])
MOWABLE = YARD.mowable_area().area


def _job(args):
    policy, seed = args
    r = run_mission(YARD, PLAN, policy, seed=seed)
    err = r.loc_error
    row = dict(policy=policy, seed=seed, completed=r.completed, minutes=r.mission_time / 60.0,
               coverage=r.coverage, deferred_pct=100.0 * r.deferred_area / MOWABLE,
               violations=r.violations, violation_s=r.violation_time,
               loc_p95=float(np.percentile(err, 95)), loc_max=float(err.max()),
               xte_p95=float(np.percentile(r.xte, 95)), skips=r.skips, retreats=r.retreats)
    return row, (r if seed == 0 else None)


def table(rows):
    def stat(vals, fmt):
        vals = np.asarray(vals, dtype=float)
        return f"{fmt.format(vals.mean())} ({fmt.format(vals.min())}–{fmt.format(vals.max())})"

    lines = ["| Policy | Mowed (%) | Deferred, reported (%) | Contact events / mission | Missions with contact "
             "| P95 position error (m) | Mission time (min) |",
             "|---|---|---|---|---|---|---|"]
    for p in POLICIES:
        rs = rows[p]
        lines.append("| {} | {} | {} | {} | {} / {} | {} | {} |".format(
            plots.LABEL[p], stat([100 * r["coverage"] for r in rs], "{:.1f}"),
            stat([r["deferred_pct"] for r in rs], "{:.1f}"),
            stat([r["violations"] for r in rs], "{:.1f}"),
            sum(r["violations"] > 0 for r in rs), len(rs),
            stat([r["loc_p95"] for r in rs], "{:.2f}"),
            stat([r["minutes"] for r in rs], "{:.0f}")))
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seeds", type=int, default=10, help="missions per policy")
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--out", default="results")
    ap.add_argument("--gif", action="store_true", help="also render an animated GIF of the resilient run")
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(exist_ok=True)
    print(f"Yard: {MOWABLE:.0f} m² mowable, plan {PLAN.length:.0f} m at a {PLAN.angle_deg:.0f}° sweep angle")

    t0 = time.time()
    rows, showcase = {p: [] for p in POLICIES}, {}
    jobs = [(p, s) for s in range(args.seeds) for p in POLICIES]
    with Pool(args.workers) as pool:
        for row, result in pool.imap_unordered(_job, jobs):
            rows[row["policy"]].append(row)
            if result is not None:
                showcase[row["policy"]] = result
            print(f"  {row['policy']:<10} seed {row['seed']:>2}: {100 * row['coverage']:5.1f}% mowed, "
                  f"{row['violations']:>2} contact events, {row['minutes']:4.0f} min", flush=True)
    for p in POLICIES:
        rows[p].sort(key=lambda r: r["seed"])

    md = table(rows)
    print("\n" + md + f"\n\n({time.time() - t0:.0f} s)")
    (out / "results.md").write_text(md + "\n")
    (out / "results.json").write_text(json.dumps(rows, indent=1))

    plots.scenario(YARD, PLAN, out / "scenario.png")
    plots.comparison(YARD, showcase, out / "comparison.png")
    plots.timeline(showcase["resilient"], out / "timeline_resilient.png")
    plots.summary(rows, out / "summary.png")
    if args.gif:
        plots.animate(YARD, showcase["resilient"], out / "resilient.gif")
    print(f"Figures written to {out}/")


if __name__ == "__main__":
    main()
