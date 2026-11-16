"""Figures for the README (matplotlib, headless)."""
from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402
from scipy.stats import chi2  # noqa: E402

from .detectors import ALPHA, BIAS_LIMIT, NAMES, WINDOW, rolling_sum  # noqa: E402
from .dual_antenna import (BASELINE, COURSE_WINDOW, SIGMA_B, SIGMA_SLIP, THRESHOLD, wrap)  # noqa: E402

COLORS = {"snapshot": "#457b9d", "window": "#1d3557", "bias": "#e9a23b", "iforest": "#9d4edd", "any": "#2d6a4f",
          "baseline": "#2a9d8f", "heading": "#e76f51", "course": "#6d597a"}


def trajectory(prop, unprotected, protected, path):
    sc = unprotected.scenario
    fig, ax = plt.subplots(figsize=(8.4, 6.0))
    ax.add_patch(Rectangle((-prop.width / 2, -prop.height / 2), prop.width, prop.height,
                           fc="#eef5e9", ec="#5b8c4a", lw=1.5, zorder=0, label="Property line"))
    lap = unprotected.t < unprotected.t[0] + 30
    ax.plot(*unprotected.truth[lap].T, color="#adb5bd", lw=1.0, ls="--", label="Planned patrol (5 m inside)")
    after = (unprotected.t >= sc.start) & (unprotected.t <= sc.start + 30)
    ax.plot(*unprotected.flown[after].T, color="#d62828", lw=1.6, label="Drone without integrity monitor")
    window = (protected.t >= sc.start) & (protected.t <= protected.alarm_time + 10)
    ax.plot(*protected.flown[window].T, color="#2d6a4f", lw=2.0, label="Drone with monitor (alarm, then hold)")
    onset = np.searchsorted(unprotected.t, sc.start)
    ax.plot(*unprotected.flown[onset], "o", color="black", ms=6, label="Spoofing starts", zorder=5)
    alarm = np.searchsorted(protected.t, protected.alarm_time)
    ax.plot(*protected.flown[alarm], "*", color="#2d6a4f", mec="black", ms=14,
            label=f"Alarm after {protected.alarm_time - sc.start:.1f} s", zorder=6)
    ax.set_aspect("equal")
    ax.set_xlabel("East (m)", fontsize=9)
    ax.set_ylabel("North (m)", fontsize=9)
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=8, frameon=False)
    ax.set_title(f"{sc.name}: 30 s after onset", fontsize=10)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def traces(logs, learned, path):
    """Test statistics around the attack for each unprotected flight in `logs` (one column each)."""
    fig, axes = plt.subplots(5, len(logs), figsize=(6.2 * len(logs), 9.5), sharex="col", squeeze=False)
    for col, log in enumerate(logs):
        sc = log.scenario
        t = log.ep_t - sc.start
        ok = log.ep_ok
        nis = np.where(ok, log.nis, np.nan)
        win = rolling_sum(np.where(ok, log.nis, 0.0), WINDOW) / chi2.ppf(1 - ALPHA, 2 * WINDOW)
        rows = [
            (nis, chi2.ppf(1 - ALPHA, 2), "NIS (one fix)", "snapshot", True),
            (win, 1.0, "5 s χ² / threshold", "window", False),
            (np.abs(log.bias).max(axis=1), BIAS_LIMIT, "|accel bias est.| (m/s²)", "bias", False),
            (learned.scores(log), learned.threshold, "Isolation Forest score", "iforest", False),
        ]
        for ax, (y, thr, label, key, logy) in zip(axes[:4, col], rows):
            ax.plot(t, y, color=COLORS[key], lw=0.8)
            ax.axhline(thr, color="#d62828", lw=0.9, ls="--")
            if logy:
                ax.set_yscale("log")
            ax.set_ylabel(label, fontsize=8)
        ax = axes[4, col]
        ax.plot(log.t - sc.start, log.error, color="#d62828", lw=1.0)
        ax.axhline(5.0, color="black", lw=0.8, ls=":")
        ax.text(-58, 5.3, "geofence margin (5 m)", fontsize=7)
        ax.set_ylabel("Drag without monitor (m)", fontsize=8)
        ax.set_ylim(0, 12)
        ax.set_xlabel("Time since spoofing started (s)", fontsize=9)
        for a in axes[:, col]:
            a.axvline(0, color="black", lw=0.8)
            a.tick_params(labelsize=7)
            a.set_xlim(-60, 60)
        axes[0, col].set_title(sc.name, fontsize=10)
    fig.suptitle("What each detector sees (dashed red = alarm threshold)", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def sweep(rows, breach, path):
    """Median detection delay vs. spoofer acceleration, against the time to drag the drone 5 m."""
    accels = sorted({r["accel"] for r in rows})
    fig, ax = plt.subplots(figsize=(8.2, 4.8))
    xs = np.array(accels)
    ax.fill_between(xs, [breach[a] for a in accels], 70, color="#f8d7da", alpha=0.7, lw=0,
                    label="Too late: drone already dragged 5 m")
    ax.plot(xs, [breach[a] for a in accels], color="#d62828", lw=1.2)
    for key in ("snapshot", "window", "bias", "iforest", "baseline"):
        med = [next(r for r in rows if r["accel"] == a)["median_delay"][key] for a in accels]
        frac = [next(r for r in rows if r["accel"] == a)["detected"][key] for a in accels]
        y = np.array([m if f >= 0.5 else np.nan for m, f in zip(med, frac)], dtype=float)
        ax.plot(xs, y, "o-", color=COLORS[key], lw=1.6, ms=5, label=NAMES[key])
        for a, yy in zip(xs, y):
            if np.isnan(yy):
                ax.plot(a, 66, "x", color=COLORS[key], ms=7)
    ax.text(xs[0], 67.5, "x = missed in most flights", fontsize=7, va="bottom")
    ax.set_xscale("log")
    ax.set_ylim(0, 70)
    ax.set_xlabel("Spoofer's fake acceleration (m/s², log scale)", fontsize=9)
    ax.set_ylabel("Median detection delay (s)", fontsize=9)
    ax.set_title("Drift spoofing from one transmit antenna: detection delay vs. how gently the attacker drags",
                 fontsize=10)
    ax.legend(fontsize=8, frameon=False, loc="upper right")
    ax.tick_params(labelsize=8)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def _turning(log, thr=0.1):
    """Epoch-time mask of when the drone is turning (|yaw rate| > thr rad/s), from its true path."""
    v = np.diff(log.truth, axis=0) / np.diff(log.t)[:, None]
    hdg = np.arctan2(v[:, 1], v[:, 0])
    rate = np.r_[0.0, np.abs([wrap(d) for d in np.diff(hdg)]) / np.diff(log.t)[1:], 0.0]
    return np.interp(log.ep_t, log.t, rate) > thr


def dual_antenna(single, frozen, tracking, path):
    """Three attackers the IMU tests miss, the dual-antenna test that sees each, and the drag prevented.

    Each argument is (unprotected, protected IMU-only, protected with dual antenna) logs of the same
    gentle drift spoof from: one transmit antenna, an array replaying a frozen baseline, and an
    attitude-aware array that reproduces the true baseline.
    """
    fig, axes = plt.subplots(2, 3, figsize=(15.5, 6.8), sharex=True)
    band = np.sqrt(THRESHOLD) * SIGMA_B
    course_limit = np.degrees(np.sqrt(THRESHOLD * ((SIGMA_B / BASELINE) ** 2 / COURSE_WINDOW + SIGMA_SLIP**2)))
    cases = ((single, "One transmit antenna: the baseline collapses"),
             (frozen, "Array replaying a frozen baseline: heading stops turning"),
             (tracking, "Attitude-aware array: course bends away from heading"))
    for col, (logs, title) in enumerate(cases):
        free, _, guarded = logs
        sc = free.scenario
        t = free.ep_t - sc.start
        ax = axes[0, col]
        if col == 0:
            ax.axhspan(BASELINE - band, BASELINE + band, color="#e9f5f2", lw=0, label="Healthy band (alarm outside)")
            ax.plot(t, free.bl_len, color=COLORS["baseline"], lw=0.9, label="Measured antenna separation")
            ax.axhline(BASELINE, color="#6c757d", lw=0.8, ls=":")
            ax.set_ylabel("Baseline length (m)", fontsize=8)
            ax.set_ylim(-0.02, 0.26)
            loc = "center left"
        elif col == 1:
            ax.fill_between(t, 1e-3, 1e5, where=_turning(free), color="#f1f3f5", lw=0, label="Drone turning")
            ax.plot(t, np.clip(free.h_nis, 1e-3, None), color=COLORS["heading"], lw=0.8,
                    label="Antenna heading vs gyro (NIS)")
            ax.axhline(THRESHOLD, color="#d62828", lw=0.9, ls="--", label="Alarm threshold")
            ax.set_yscale("log")
            ax.set_ylim(1e-3, 1e5)
            ax.set_ylabel("Heading NIS", fontsize=8)
            loc = "upper left"
        else:
            res = np.degrees(free.c_res)
            kernel = np.ones(COURSE_WINDOW) / COURSE_WINDOW
            mean = np.convolve(np.nan_to_num(res), kernel, mode="full")[:len(res)]
            ax.axhspan(-course_limit, course_limit, color="#efeaf2", lw=0, label="Healthy band for the 3 s mean")
            ax.plot(t, res, color="#b8adc2", lw=0.6, label="Course minus antenna heading")
            ax.plot(t, mean, color=COLORS["course"], lw=1.4, label="3 s mean")
            ax.set_ylim(-16, 16)
            ax.set_ylabel("Course - heading (deg)", fontsize=8)
            loc = "upper left"
        ax.legend(fontsize=7, loc=loc, framealpha=0.95, edgecolor="none")
        ax.set_title(title, fontsize=9)
        ax = axes[1, col]
        ax.plot(free.t - sc.start, free.error, color="#d62828", lw=1.2, label="No monitor, or IMU tests only (no alarm)")
        if np.isfinite(guarded.alarm_time):
            win = guarded.t <= guarded.alarm_time + 10
            ax.plot(guarded.t[win] - sc.start, guarded.error[win], color="#2d6a4f", lw=1.6,
                    label=f"Monitor + dual antenna (alarm at {guarded.alarm_time - sc.start:.1f} s, then hold)")
        ax.axhline(5.0, color="black", lw=0.8, ls=":")
        ax.text(-9.5, 5.3, "geofence margin (5 m)", fontsize=7)
        ax.set_ylim(0, 12)
        ax.set_ylabel("Drag off the patrol path (m)", fontsize=8)
        ax.set_xlabel("Time since spoofing started (s)", fontsize=9)
        ax.legend(fontsize=7, loc="upper left", framealpha=0.95, edgecolor="none")
        for a in axes[:, col]:
            a.axvline(0, color="black", lw=0.8)
            a.set_xlim(-10, 30)
            a.tick_params(labelsize=7)
    fig.suptitle("A gentle 0.03 m/s² spoof hides from every IMU test; each dual-antenna test catches a "
                 "harder attacker", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
