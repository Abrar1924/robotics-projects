"""Complete-coverage path planning for a mower deck.

1. Headland loops: drive the free-space boundary first (lawn edge, then around obstacles).
2. Boustrophedon swaths: parallel passes at the sweep angle that needs the fewest
   turns (the same brute-force angle search Fields2Cover uses).
3. Greedy ordering of swath pieces, joined by the shortest obstacle-free transit
   found on a visibility graph of free-space corners.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

import numpy as np
from shapely import affinity
from shapely.geometry import LineString, Polygon
from shapely.prepared import prep

MOW, TRANSIT = 0, 1


def polygons(geom):
    """Flatten a (Multi)Polygon / GeometryCollection into a list of polygons."""
    if geom is None or geom.is_empty:
        return []
    if isinstance(geom, Polygon):
        return [geom]
    out = []
    for g in getattr(geom, "geoms", []):
        out.extend(polygons(g))
    return out


def _lines(geom):
    if geom.is_empty:
        return []
    if isinstance(geom, LineString):
        return [geom]
    out = []
    for g in getattr(geom, "geoms", []):
        out.extend(_lines(g))
    return out


def densify(points, step=0.1):
    """Resample a polyline so consecutive points are at most `step` apart."""
    pts = np.asarray(points, dtype=float)
    out = [pts[0]]
    for p, q in zip(pts[:-1], pts[1:]):
        n = max(1, int(math.ceil(math.dist(p, q) / step)))
        out.extend(p + (q - p) * (k / n) for k in range(1, n + 1))
    return np.array(out)


def swath_rows(region, spacing, angle_deg):
    """Parallel passes `spacing` apart at `angle_deg`, clipped to `region`.

    Returns one list per pass of ((x0, y0), (x1, y1)) pieces; obstacles split a pass
    into several pieces.
    """
    origin = region.centroid
    rotated = affinity.rotate(region, -angle_deg, origin=origin)
    minx, miny, maxx, maxy = rotated.bounds
    n = max(1, math.ceil((maxy - miny) / spacing))
    rows = []
    for y in np.linspace(miny + 1e-3, maxy - 1e-3, n + 1):
        cut = rotated.intersection(LineString([(minx - 1.0, y), (maxx + 1.0, y)]))
        pieces = sorted((ln for ln in _lines(cut) if ln.length > 0.10),
                        key=lambda ln: min(ln.coords[0][0], ln.coords[-1][0]))
        row = []
        for ln in pieces:
            a, b = sorted([ln.coords[0], ln.coords[-1]])
            seg = affinity.rotate(LineString([a, b]), angle_deg, origin=origin)
            row.append((tuple(seg.coords[0]), tuple(seg.coords[-1])))
        if row:
            rows.append(row)
    return rows


class TransitPlanner:
    """Shortest collision-free connections on a visibility graph of free-space corners."""

    def __init__(self, free_space, margin=0.05):
        self.free = free_space
        self._inside = prep(free_space.buffer(1e-6))
        shrunk = free_space.buffer(-margin, quad_segs=2).simplify(0.02)
        nodes = []
        for poly in polygons(shrunk):
            for ring in (poly.exterior, *poly.interiors):
                nodes.extend(tuple(p) for p in ring.coords[:-1])
        self.nodes = nodes
        self.adj = [[] for _ in nodes]
        for i in range(len(nodes)):
            for j in range(i + 1, len(nodes)):
                if self.visible(nodes[i], nodes[j]):
                    d = math.dist(nodes[i], nodes[j])
                    self.adj[i].append((j, d))
                    self.adj[j].append((i, d))

    def visible(self, a, b):
        if math.dist(a, b) < 1e-9:
            return True
        return self._inside.contains(LineString([a, b]))

    def path(self, a, b):
        """Waypoints from a to b that stay inside free space (Dijkstra on the graph)."""
        a, b = (float(a[0]), float(a[1])), (float(b[0]), float(b[1]))
        if self.visible(a, b):
            return [a, b]
        n = len(self.nodes)
        start, goal = n, n + 1
        first = [(i, math.dist(a, p)) for i, p in enumerate(self.nodes) if self.visible(a, p)]
        last = {i: math.dist(p, b) for i, p in enumerate(self.nodes) if self.visible(p, b)}
        dist, prev = {start: 0.0}, {}
        heap = [(0.0, start)]
        while heap:
            d, u = heapq.heappop(heap)
            if u == goal:
                break
            if d > dist.get(u, math.inf):
                continue
            edges = first if u == start else self.adj[u] + ([(goal, last[u])] if u in last else [])
            for v, w in edges:
                if d + w < dist.get(v, math.inf):
                    dist[v], prev[v] = d + w, u
                    heapq.heappush(heap, (d + w, v))
        if goal not in prev:
            return [a, b]  # disconnected free space: fall back to a straight line
        route, u = [b], prev[goal]
        while u != start:
            route.append(self.nodes[u])
            u = prev[u]
        route.append(a)
        return route[::-1]


@dataclass
class CoveragePlan:
    xy: np.ndarray       # (N, 2) waypoints ~0.1 m apart
    kind: np.ndarray     # MOW or TRANSIT for each waypoint
    angle_deg: float     # chosen sweep angle
    rows: list           # swath pieces, for plotting

    @property
    def length(self):
        d = np.diff(self.xy, axis=0)
        return float(np.hypot(d[:, 0], d[:, 1]).sum())


def plan_coverage(free_space, deck_width=0.5, overlap=0.10, start=(0.0, 0.0),
                  angles=range(0, 180, 5), transit=None):
    spacing = deck_width * (1.0 - overlap)
    # Swaths end where the headland pass's deck coverage begins.
    region = free_space.buffer(-(deck_width / 2.0 - 0.05))
    best = None
    for angle in angles:
        rows = swath_rows(region, spacing, angle)
        key = (sum(len(r) for r in rows), sum(math.dist(a, b) for r in rows for a, b in r))
        if best is None or key < best[0]:
            best = (key, angle, rows)
    _, angle, rows = best
    transit = transit or TransitPlanner(free_space)

    pts, kinds = [(float(start[0]), float(start[1]))], [TRANSIT]

    def extend(seq, kind):
        dense = densify([pts[-1], *seq])
        pts.extend(map(tuple, dense[1:]))
        kinds.extend([kind] * (len(dense) - 1))

    def travel_to(p):
        extend(transit.path(pts[-1], p)[1:], TRANSIT)

    def mow_loop(loop):
        i = min(range(len(loop)), key=lambda k: math.dist(pts[-1], loop[k]))
        order = loop[i:] + loop[:i] + [loop[i]]
        travel_to(order[0])
        extend(order[1:], MOW)

    loops = [[tuple(p) for p in ring.coords[:-1]]
             for poly in polygons(free_space) for ring in (poly.exterior, *poly.interiors)]
    loops.sort(key=lambda ring: -Polygon(ring).area)
    mow_loop(loops[0])  # lawn edge first, like a commercial mower's perimeter pass
    loops = loops[1:]
    swaths = [piece for row in rows for piece in row]
    while swaths or loops:
        here = pts[-1]
        best = None
        for i, (a, b) in enumerate(swaths):
            for entry, exit_ in ((a, b), (b, a)):
                d = math.dist(here, entry)
                if best is None or d < best[0]:
                    best = (d, "swath", i, entry, exit_)
        for i, loop in enumerate(loops):
            d = min(math.dist(here, p) for p in loop)
            if best is None or d < best[0]:
                best = (d, "loop", i, None, None)
        _, kind, i, entry, exit_ = best
        if kind == "swath":
            swaths.pop(i)
            travel_to(entry)
            extend([exit_], MOW)
        else:
            mow_loop(loops.pop(i))
    return CoveragePlan(xy=np.array(pts), kind=np.array(kinds), angle_deg=float(angle), rows=rows)
