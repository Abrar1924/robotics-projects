#!/usr/bin/env python3
"""Train and calibrate the learned detector on clean flights, evaluate every detector on
the fault scenarios and on a drift-spoofing sweep, and write tables and figures to results/."""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from gim import plots
from gim.detectors import (CLASSICAL, DUAL, NAMES, LearnedDetector, alarm_events, classical_alarms,
                           dual_alarms, first_alarm)
from gim.faults import Scenario, standard_scenarios
from gim.flight import Property, lap_time
from gim.sim import fly

HORIZON = 60.0    # s after onset within which a detection counts
RESPONSE = 10.0   # s the drone needs to hold or land after an alarm
GEOFENCE = 5.0    # m between the patrol path and the property line
DETECTORS = (*CLASSICAL, "iforest", *DUAL)
SWEEP_DETECTORS = (*CLASSICAL, "iforest", "baseline")


def _fly(job):
    scenario, seed, protect, dual = job
    return fly(scenario, seed=seed, protect=protect, dual=dual)


def variant(sc, seed, n):
    """Spoofers that fake the antenna baseline are caught by tests that depend on where the drone
    is in its lap (the next turn, the next leg across the drag). Spread their onset evenly over
    one lap so the result does not hinge on one lucky or unlucky starting point."""
    if sc.is_attack and sc.array != "single":
        return dataclasses.replace(sc, start=sc.start + (seed % n) * lap_time() / n)
    return sc


def alarms_for(log, learned):
    a = classical_alarms(log)
    a["iforest"] = learned.alarms(log)
    a.update(dual_alarms(log))
    a["any"] = a["snapshot"] | a["window"] | a["bias"]
    a["any_dual"] = a["any"] | a["baseline"] | a["heading"] | a["course"]
    return a


def held(logs):
    """Worst error from onset until the drone has had RESPONSE seconds to react to the alarm."""
    out = []
    for lg in logs:
        start = lg.scenario.start
        end = lg.alarm_time + RESPONSE if not math.isnan(lg.alarm_time) else start + 30
        out.append(lg.error[(lg.t >= start) & (lg.t <= end)].max())
    return out


def breach_time(accel, max_speed=2.0, margin=GEOFENCE):
    t1 = max_speed / accel
    d1 = 0.5 * accel * t1 * t1
    return math.sqrt(2 * margin / accel) if margin <= d1 else t1 + (margin - d1) / max_speed


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seeds", type=int, default=10, help="flights per scenario")
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--out", default="results")
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(exist_ok=True)
    t0 = time.time()
    scenarios = standard_scenarios()
    clean, honest = scenarios[0], scenarios[2]

    with Pool(args.workers) as pool:
        run = lambda jobs: pool.map(_fly, jobs)  # noqa: E731
        # 1. Learned detector: trained and calibrated on clean + honestly degraded flights only.
        train = run([(clean, s, False, False) for s in range(1000, 1020)]
                    + [(honest, s, False, False) for s in range(1100, 1110)])
        calib = run([(clean, s, False, False) for s in range(2000, 2020)]
                    + [(honest, s, False, False) for s in range(2100, 2110)])
        learned = LearnedDetector().fit(train).calibrate(calib)
        print(f"Isolation Forest trained on {len(train)} flights, threshold {learned.threshold:.3f}")

        # 2. False alarms on unseen clean and honestly degraded flights.
        benign = run([(clean, s, False, False) for s in range(3000, 3030)]
                     + [(honest, s, False, False) for s in range(3100, 3110)])
        hours = sum(lg.t[-1] for lg in benign) / 3600.0
        false_alarms = {d: sum(alarm_events(alarms_for(lg, learned)[d]) for lg in benign) / hours
                        for d in (*DETECTORS, "any", "any_dual")}
        print(f"False alarms per hour over {hours:.1f} h of benign flight: "
              + ", ".join(f"{d} {v:.2f}" for d, v in false_alarms.items()))

        # 3. Scenarios: detection on unprotected flights, outcome with the monitor in the loop.
        table, examples = [], {}
        for sc in scenarios:
            seeds = range(args.seeds)
            unprot = run([(variant(sc, s, args.seeds), s, False, False) for s in seeds])
            prot = run([(variant(sc, s, args.seeds), s, True, False) for s in seeds])
            prot_dual = run([(variant(sc, s, args.seeds), s, True, True) for s in seeds])
            examples[sc.name] = (unprot[0], prot[0], prot_dual[0])
            row = {"scenario": sc.name, "kind": sc.kind, "detected": {}, "median_delay": {}, "delay_range": {},
                   "error_at_alarm": {}}
            for d in (*DETECTORS, "any", "any_dual"):
                delays = [first_alarm(lg.ep_t, alarms_for(lg, learned)[d], lg.scenario.start) for lg in unprot]
                hits = [x for x in delays if not math.isnan(x) and x <= HORIZON]
                row["detected"][d] = len(hits) / len(unprot)
                row["median_delay"][d] = float(np.median(hits)) if hits else None
                row["delay_range"][d] = [float(min(hits)), float(max(hits))] if hits else None
                at_alarm = [float(np.interp(lg.scenario.start + x, lg.t, lg.error))
                            for lg, x in zip(unprot, delays) if not math.isnan(x) and x <= HORIZON]
                row["error_at_alarm"][d] = float(np.median(at_alarm)) if at_alarm else None
            drag = [lg.error[(lg.t >= lg.scenario.start) & (lg.t <= lg.scenario.start + 30)].max() for lg in unprot]
            kept, kept_dual = held(prot), held(prot_dual)
            row.update(unprotected_error=float(np.median(drag)), protected_error=float(np.median(kept)),
                       protected_dual_error=float(np.median(kept_dual)),
                       dual_alarm_sources=sorted({lg.alarm_source for lg in prot_dual if lg.alarm_source}),
                       unprotected_breach=float(np.mean(np.array(drag) > GEOFENCE)),
                       protected_breach=float(np.mean(np.array(kept) > GEOFENCE)),
                       protected_dual_breach=float(np.mean(np.array(kept_dual) > GEOFENCE)))
            table.append(row)
            print(f"  {sc.name:<60} any-classical {row['detected']['any']:.0%}, with dual {row['detected']['any_dual']:.0%}, "
                  f"error {row['unprotected_error']:.1f} m -> {row['protected_error']:.1f} m "
                  f"-> {row['protected_dual_error']:.1f} m", flush=True)

        # 4. Drift-spoofing sweep: how gentle can the attacker be?
        sweep = []
        for accel in (0.01, 0.02, 0.03, 0.05, 0.1, 0.2, 0.5):
            sc = Scenario(f"Drift spoof {accel} m/s²", "drift_spoof", magnitude=accel)
            logs = run([(sc, s, False, False) for s in range(args.seeds)])
            entry = {"accel": accel, "detected": {}, "median_delay": {}}
            for d in SWEEP_DETECTORS:
                delays = [first_alarm(lg.ep_t, alarms_for(lg, learned)[d], sc.start) for lg in logs]
                hits = [x for x in delays if not math.isnan(x) and x <= HORIZON]
                entry["detected"][d] = len(hits) / len(logs)
                entry["median_delay"][d] = float(np.median(hits)) if hits else None
            sweep.append(entry)
            print(f"  sweep {accel:>5} m/s²: " + ", ".join(
                f"{d} {entry['detected'][d]:.0%}" for d in SWEEP_DETECTORS), flush=True)

    md = write_tables(table, false_alarms, sweep, hours, args.seeds)
    (out / "results.md").write_text(md)
    (out / "results.json").write_text(json.dumps(
        {"scenarios": table, "false_alarms_per_hour": false_alarms, "sweep": sweep}, indent=1))
    moderate = next(s for s in scenarios if "moderate" in s.name)
    gentle = next(s for s in scenarios if "gentle" in s.name)
    plots.trajectory(Property(), *examples[moderate.name][:2], out / "spoof_trajectory.png")
    plots.traces([examples[moderate.name][0], examples[gentle.name][0]], learned, out / "detector_traces.png")
    plots.sweep(sweep, {e["accel"]: breach_time(e["accel"]) for e in sweep}, out / "detection_vs_aggressiveness.png")
    frozen = next(s for s in scenarios if s.array == "frozen")
    tracking = next(s for s in scenarios if s.array == "tracking")
    plots.dual_antenna(examples[gentle.name], examples[frozen.name], examples[tracking.name],
                       out / "dual_antenna.png")
    print(md)
    print(f"done in {time.time() - t0:.0f} s; results in {out}/")


def write_tables(table, false_alarms, sweep, hours, n):
    def cell(row, d):
        if row["kind"] in ("nominal", "float"):
            return f"{row['detected'][d] * n:.0f}/{n} false alarms"
        if row["kind"] == "jamming":
            return "n/a (no fixes to test)"
        k = round(row["detected"][d] * n)
        delay = row["median_delay"][d]
        return f"{k}/{n}" + (f", {delay:.1f} s" if delay is not None else "")

    lines = [f"### Detection ({n} flights per scenario; detections within {HORIZON:.0f} s, median delay)", "",
             "| Scenario | " + " | ".join(NAMES[d] for d in DETECTORS) + " |",
             "|---|" + "---|" * len(DETECTORS)]
    lines += ["| " + r["scenario"] + " | " + " | ".join(cell(r, d) for d in DETECTORS) + " |" for r in table]
    lines += ["", f"### Outcome with the monitor in the loop (median of {n} flights)", "",
              "| Scenario | Error without monitor (30 s after onset) | Error with monitor, IMU tests (until alarm + 10 s) "
              "| Error with monitor + dual antenna | Flights dragged past the 5 m geofence margin: none → IMU → dual |",
              "|---|---|---|---|---|"]
    lines += [f"| {r['scenario']} | {r['unprotected_error']:.1f} m | {r['protected_error']:.1f} m | "
              f"{r['protected_dual_error']:.1f} m | {r['unprotected_breach'] * n:.0f}/{n} → "
              f"{r['protected_breach'] * n:.0f}/{n} → {r['protected_dual_breach'] * n:.0f}/{n} |" for r in table]
    fa = (*DETECTORS, "any", "any_dual")
    lines += ["", f"### False alarms per hour ({hours:.1f} h of unseen clean and honestly degraded flight)", "",
              "| " + " | ".join(NAMES[d] for d in fa) + " |",
              "|" + "---|" * len(fa),
              "| " + " | ".join(f"{false_alarms[d]:.2f}" for d in fa) + " |"]
    lines += ["", "### Drift-spoofing sweep, single transmit antenna (share of flights detected within 60 s)", "",
              "| Fake acceleration (m/s²) | " + " | ".join(NAMES[d] for d in SWEEP_DETECTORS) + " |",
              "|---|" + "---|" * len(SWEEP_DETECTORS)]
    lines += [f"| {e['accel']} | " + " | ".join(f"{e['detected'][d]:.0%}" for d in SWEEP_DETECTORS) + " |"
              for e in sweep]
    arrays = [r for r in table if "array" in r["scenario"]]
    if arrays:
        lines += ["", "### Spoofers that fake the antenna baseline (onset spread evenly over one lap)", "",
                  "| Scenario | Test that catches it | Detected | Delay: median (range) | Drag at the alarm (median) |",
                  "|---|---|---|---|---|"]
        for r in arrays:
            d = "heading" if "frozen" in r["scenario"] else "course"
            lo, hi = r["delay_range"][d] or (float("nan"), float("nan"))
            lines.append(f"| {r['scenario']} | {NAMES[d]} | {round(r['detected'][d] * n)}/{n} | "
                         f"{r['median_delay'][d]:.1f} s ({lo:.1f}–{hi:.1f} s) | {r['error_at_alarm'][d]:.1f} m |")
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    main()
