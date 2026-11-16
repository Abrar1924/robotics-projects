#!/usr/bin/env python3
"""RTK readiness survey: run resilient missions in a yard, map where RTK holds, and give the
yard a verdict with problem zones and mitigations. Also runs a what-if (the maple's dense core
thinned out) to show how the tool supports an installer's decision. Writes to results/."""
from __future__ import annotations

import argparse
import json
import time
from multiprocessing import Pool
from pathlib import Path

from mower import plots
from mower.planner import plan_coverage
from mower.readiness import readiness, report_markdown, summary
from mower.sim import run_mission
from mower.world import suburban_yard

YARDS = {"As is": suburban_yard(), "Maple's dense core thinned": suburban_yard(dense_maple=False)}
PLANS = {k: plan_coverage(y.free_space(), y.deck_width, start=y.dock[:2]) for k, y in YARDS.items()}


def _job(args):
    name, policy, seed = args
    return name, policy, run_mission(YARDS[name], PLANS[name], policy, seed=seed)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seeds", type=int, default=5, help="missions per yard")
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--out", default="results")
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(exist_ok=True)
    t0 = time.time()
    jobs = [(n, "resilient", s) for n in YARDS for s in range(args.seeds)] + [(n, "oracle", 0) for n in YARDS]
    runs = {n: [] for n in YARDS}
    reference = {}
    with Pool(args.workers) as pool:
        for name, policy, r in pool.imap_unordered(_job, jobs):
            if policy == "oracle":
                reference[name] = r.mission_time / 60.0
            else:
                runs[name].append(r)
                print(f"  {name:<28} seed {r.seed}: {100 * r.coverage:.1f}% mowed, "
                      f"{100 * r.deferred_area / YARDS[name].mowable_area().area:.1f}% deferred", flush=True)
    reports = {n: readiness(YARDS[n], sorted(runs[n], key=lambda r: r.seed)) for n in YARDS}
    md = ["## RTK readiness survey", ""]
    for n, rep in reports.items():
        md.append(report_markdown(n, rep, reference.get(n)))
    text = "\n".join(md)
    (out / "readiness.md").write_text(text)
    (out / "readiness.json").write_text(json.dumps({n: summary(r) for n, r in reports.items()}, indent=1))
    plots.readiness_map(YARDS, reports, out / "readiness_map.png")
    print(text)
    print(f"done in {time.time() - t0:.0f} s; results in {out}/")


if __name__ == "__main__":
    main()
