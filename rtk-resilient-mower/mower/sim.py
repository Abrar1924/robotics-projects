"""Run one mowing mission end to end and score it."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import shapely
from shapely.geometry import LineString, MultiPoint, Point
from shapely.ops import nearest_points

from .control import PurePursuit
from .ekf import MowerEKF
from .gnss import FIXED, FLOAT, RTKReceiver
from .planner import TransitPlanner, densify
from .robot import DiffDriveMower, wrap
from .supervisor import Mode, Supervisor, SupervisorConfig

POLICIES = ("oracle", "gnss_only", "ekf", "resilient")
BODY_RADIUS = 0.25                      # m, robot footprint used for safety checks
FIXED_SIGMA, FLOAT_WHITE_SIGMA = 0.02, 0.12


class ClearanceMap:
    """Distance from any point to the nearest hazard (obstacle or lawn edge) minus the
    robot's footprint radius, precomputed on a 5 cm grid for O(1) lookups."""

    def __init__(self, yard, res=0.05, body_radius=BODY_RADIUS):
        minx, miny, maxx, maxy = yard.boundary.bounds
        self.x0, self.y0, self.res = minx - 1.0, miny - 1.0, res
        nx = int(math.ceil((maxx - minx + 2.0) / res)) + 1
        ny = int(math.ceil((maxy - miny + 2.0) / res)) + 1
        X, Y = np.meshgrid(self.x0 + res * np.arange(nx), self.y0 + res * np.arange(ny))
        pts = shapely.points(X.ravel(), Y.ravel())
        d_obstacle = shapely.distance(yard.obstacles, pts)
        d_edge = shapely.distance(yard.boundary.exterior, pts)
        inside = shapely.contains_xy(yard.boundary, X.ravel(), Y.ravel())
        self.grid = (np.where(inside, np.minimum(d_obstacle, d_edge), -d_edge) - body_radius).reshape(ny, nx)

    def __call__(self, x, y):
        j = min(max(int(round((x - self.x0) / self.res)), 0), self.grid.shape[1] - 1)
        i = min(max(int(round((y - self.y0) / self.res)), 0), self.grid.shape[0] - 1)
        return float(self.grid[i, j])


_CLEARANCE_CACHE = {}


def clearance_map(yard):
    if id(yard) not in _CLEARANCE_CACHE:
        _CLEARANCE_CACHE[id(yard)] = ClearanceMap(yard)
    return _CLEARANCE_CACHE[id(yard)]


@dataclass
class MissionResult:
    policy: str
    seed: int
    t: np.ndarray
    true: np.ndarray
    est: np.ndarray
    sigma: np.ndarray
    quality: np.ndarray
    mode: np.ndarray
    speed: np.ndarray
    xte: np.ndarray
    completed: bool
    mission_time: float
    coverage: float
    covered: object
    deferred: object
    violations: int
    violation_time: float
    violation_points: np.ndarray
    skips: int
    retreats: int

    @property
    def loc_error(self):
        return np.hypot(*(self.est[:, :2] - self.true[:, :2]).T)

    @property
    def deferred_area(self):
        return 0.0 if self.deferred is None else float(self.deferred.area)


def _upcoming(ctrl, metres):
    """Planned waypoints in the next `metres` of the controller's path."""
    j = min(int(np.searchsorted(ctrl.s, ctrl.s[ctrl.idx] + metres)), len(ctrl.path) - 1)
    return ctrl.path[ctrl.idx:j + 1]


def deferral_zone(est, ctrl, clearance, sigma, cfg):
    """Area to skip after a failed hold: nearby remaining waypoints that sit too close to a
    hazard to be mowed without RTK fixed (3-sigma of the current uncertainty), plus the
    blocked spot itself. Open grass nearby stays in the plan."""
    remaining = ctrl.path[ctrl.idx:]
    near = remaining[np.hypot(remaining[:, 0] - est[0], remaining[:, 1] - est[1]) < cfg.defer_radius]
    need = max(cfg.risk_k * sigma, cfg.risk_k * cfg.sigma_floor) + 0.05
    risky = [tuple(p) for p in near if clearance(p[0], p[1]) < need]
    return MultiPoint([tuple(est[:2]), *risky]).buffer(cfg.defer_buffer)


def _inside(area, p):
    """p itself if it lies in `area`, otherwise the nearest point of `area`."""
    if area.buffer(1e-6).contains(Point(p)):
        return (float(p[0]), float(p[1]))
    q = nearest_points(area, Point(p))[0]
    return (q.x, q.y)


def travel_space(free_space, deferred, sigma, cfg, base):
    """Free space minus deferred zones, shrunk so transits keep k-sigma from hazards.
    `base` is the body clearance free space already provides."""
    safe = free_space if deferred is None else free_space.difference(deferred.buffer(0.10))
    extra = min(max(cfg.risk_k * sigma - base, 0.0), 0.50)
    shrunk = safe.buffer(-extra) if extra > 0 else safe
    return shrunk if not shrunk.is_empty else safe


def nearest_anchor(anchors, deferred, p, keep_off=0.5):
    """Closest remembered stable-fix spot that is not inside a deferred zone."""
    pts = np.asarray(anchors[-3000:])
    if deferred is not None:
        pts = pts[~shapely.contains_xy(deferred.buffer(keep_off), pts[:, 0], pts[:, 1])]
    if len(pts) == 0:
        return None
    i = int(np.argmin(np.hypot(pts[:, 0] - p[0], pts[:, 1] - p[1])))
    return (float(pts[i, 0]), float(pts[i, 1]))


def connect(planner, area, a, b):
    """Route a -> b: step into the travel area, cross it on the visibility graph, step out."""
    a_in, b_in = _inside(area, a), _inside(area, b)
    route = planner.path(a_in, b_in)
    pts = [tuple(a)] + [p for p in route if math.dist(p, a) > 1e-6]
    if math.dist(pts[-1], b) > 1e-6:
        pts.append(tuple(b))
    return pts


def reroute(remaining, deferred, free_space, start, sigma, cfg, base, min_run=0.3):
    """Drop waypoints inside deferred zones and stitch the rest together with transits that
    keep k-sigma away from hazards."""
    inside = shapely.contains_xy(deferred.buffer(0.10), remaining[:, 0], remaining[:, 1])
    runs, current = [], []
    for p, bad in zip(remaining, inside):
        if bad:
            if len(current) > 1:
                runs.append(np.array(current))
            current = []
        else:
            current.append(p)
    if len(current) > 1:
        runs.append(np.array(current))
    runs = [r for r in runs if np.hypot(*np.diff(r, axis=0).T).sum() >= min_run]
    area = travel_space(free_space, deferred, sigma, cfg, base)
    planner = TransitPlanner(area)
    pts = [tuple(start)]
    for run in runs:
        pts.extend(connect(planner, area, pts[-1], tuple(run[0]))[1:])
        pts.extend(map(tuple, run[1:]))
    return densify(pts) if len(pts) > 1 else np.array([start, start], dtype=float)


def run_mission(yard, plan, policy="resilient", seed=0, cfg=None, dt=0.05, t_max=None):
    """Simulate one mission.

    Policies:
      oracle     controller fed the true pose (isolates controller error)
      gnss_only  position straight from the receiver, heading from gyro + course over ground
      ekf        EKF fusion at full speed, no degradation logic
      resilient  EKF + risk-aware graceful degradation + defer-and-re-route
    """
    if policy not in POLICIES:
        raise ValueError(f"policy must be one of {POLICIES}")
    cfg = cfg or SupervisorConfig()
    rng = np.random.default_rng(seed)
    robot = DiffDriveMower(yard.dock, rng)
    receiver = RTKReceiver(yard, rng)
    ekf = MowerEKF(yard.dock)
    supervisor = Supervisor(cfg)
    clearance = clearance_map(yard)
    free = yard.free_space()
    base = yard.clearance - BODY_RADIUS  # body clearance free space already guarantees
    coverage_ctrl, retreat_ctrl = PurePursuit(plan.xy), None
    t_max = t_max or 3.0 * plan.length / cfg.v_nominal + 600.0

    quality, next_fix = FIXED, 0.0
    raw_xy, raw_th, last_fix_xy = np.array(yard.dock[:2], float), float(yard.dock[2]), None
    last_good = tuple(yard.dock[:2])
    anchors, fixed_since = [tuple(yard.dock[:2])], 0.0   # spots with a stable (>= 3 s) RTK fix
    deferred, skips, retreats, parked_at, skips_since_fix = None, 0, 0, None, 0
    rec = {k: [] for k in ("t", "true", "est", "sigma", "quality", "mode", "speed", "xte")}

    t, completed = 0.0, False
    while t < t_max:
        v_wheel, w_gyro = robot.wheel_speed(), robot.gyro()
        ekf.predict(v_wheel, w_gyro, dt)
        raw_th = wrap(raw_th + w_gyro * dt)
        if t + 1e-9 >= next_fix:
            next_fix += receiver.period
            quality, z, _ = receiver.measure(t, robot.x, robot.y)
            if z is not None:
                ekf.update_position(z, FIXED_SIGMA if quality == FIXED else FLOAT_WHITE_SIGMA,
                                    float_solution=quality == FLOAT)
                # GNSS-only baseline: nudge heading toward course over ground on straights.
                if last_fix_xy is not None:
                    d = z - last_fix_xy
                    if math.hypot(d[0], d[1]) > 0.08 and abs(w_gyro) < 0.2:
                        raw_th = wrap(raw_th + 0.2 * wrap(math.atan2(d[1], d[0]) - raw_th))
                raw_xy, last_fix_xy = z, z

        if policy == "oracle":
            est = (robot.x, robot.y, robot.th)
        elif policy == "gnss_only":
            est = (float(raw_xy[0]), float(raw_xy[1]), raw_th)
        else:
            est = tuple(float(v) for v in ekf.pose)
        sigma = ekf.sigma_pos
        active, v_limit, mode = coverage_ctrl, cfg.v_nominal, Mode.NOMINAL

        if policy == "resilient":
            previous = supervisor.mode
            if previous == Mode.RETREAT:
                mode = supervisor.update(t, quality, sigma)
            else:
                ahead = _upcoming(coverage_ctrl, cfg.lookahead)
                margin = min([clearance(est[0], est[1])] + [clearance(px, py) for px, py in ahead[::2]])
                mode = supervisor.update(t, quality, sigma, margin)
            if quality != FIXED:
                fixed_since = t
            if mode == Mode.NOMINAL and quality == FIXED:
                last_good, skips_since_fix = est[:2], 0
                if t - fixed_since >= 3.0 and math.dist(anchors[-1], est[:2]) >= 0.5:
                    anchors.append(est[:2])

            if supervisor.hold_expired:
                # RTK did not come back: defer the hazard-adjacent strip we could not mow.
                zone = deferral_zone(est, coverage_ctrl, clearance, sigma, cfg)
                deferred = zone if deferred is None else deferred.union(zone)
                if mode != Mode.RETREAT and skips_since_fix >= cfg.max_skips_without_fix:
                    supervisor.force(t, Mode.RETREAT)   # stop skipping blind: re-acquire RTK first
                    mode = supervisor.mode
                if mode == Mode.RETREAT:      # head to the nearest spot with a stable RTK fix
                    retreats += 1
                    area = travel_space(free, deferred, sigma, cfg, base)
                    target = nearest_anchor(anchors, deferred, est[:2]) or last_good
                    route = connect(TransitPlanner(area), area, est[:2], target)
                    retreat_ctrl, parked_at = PurePursuit(densify(route)), None
                else:                         # blocked by a hazard: skip it, keep mowing
                    skips += 1
                    skips_since_fix += 1
                    coverage_ctrl.set_path(reroute(coverage_ctrl.path[coverage_ctrl.idx:],
                                                   deferred, free, est[:2], sigma, cfg, base))
            elif mode == Mode.RETREAT and retreat_ctrl.done:
                parked_at = parked_at if parked_at is not None else t
                if t - parked_at > cfg.recovery_timeout:   # RTK still absent: resume anyway
                    supervisor.force(t, Mode.DEGRADED)
                    mode = supervisor.mode
            if previous == Mode.RETREAT and mode != Mode.RETREAT:
                coverage_ctrl.set_path(reroute(coverage_ctrl.path[coverage_ctrl.idx:],
                                               deferred, free, est[:2], sigma, cfg, base))
                retreat_ctrl = None
            v_limit = supervisor.speed_limit
            if mode == Mode.RETREAT:
                active = retreat_ctrl

        v_cmd, w_cmd = active.command(est[0], est[1], est[2], v_limit)
        rec["t"].append(t)
        rec["true"].append((robot.x, robot.y, robot.th))
        rec["est"].append(est)
        rec["sigma"].append(sigma)
        rec["quality"].append(quality)
        rec["mode"].append(int(mode))
        rec["speed"].append(v_cmd)
        rec["xte"].append(active.cross_track(robot.x, robot.y))
        robot.step(v_cmd, w_cmd, dt)
        t += dt
        if active is coverage_ctrl and coverage_ctrl.done:
            completed = True
            break
    return _score(yard, policy, seed, rec, completed, deferred, skips, retreats, dt)


def _score(yard, policy, seed, rec, completed, deferred, skips, retreats, dt):
    true, est = np.asarray(rec["true"]), np.asarray(rec["est"])
    mowable = yard.mowable_area()
    swept = LineString(true[:, :2]).simplify(0.01).buffer(yard.deck_width / 2.0, quad_segs=4)
    covered = swept.intersection(mowable)
    deferred_left = None
    if deferred is not None:
        deferred_left = deferred.intersection(mowable).difference(covered)
    pts = shapely.points(true[:, 0], true[:, 1])
    near_obstacle = shapely.distance(yard.obstacles, pts) < BODY_RADIUS
    off_lawn = ~shapely.contains_xy(yard.boundary.buffer(-BODY_RADIUS), true[:, 0], true[:, 1])
    bad = near_obstacle | off_lawn
    starts = np.flatnonzero(bad & ~np.concatenate([[False], bad[:-1]]))
    return MissionResult(
        policy=policy, seed=seed, t=np.asarray(rec["t"]), true=true, est=est,
        sigma=np.asarray(rec["sigma"]), quality=np.asarray(rec["quality"]),
        mode=np.asarray(rec["mode"]), speed=np.asarray(rec["speed"]), xte=np.asarray(rec["xte"]),
        completed=completed, mission_time=float(rec["t"][-1]) if rec["t"] else 0.0,
        coverage=float(covered.area / mowable.area), covered=covered, deferred=deferred_left,
        violations=int(len(starts)), violation_time=float(bad.sum() * dt),
        violation_points=true[starts, :2], skips=skips, retreats=retreats)
