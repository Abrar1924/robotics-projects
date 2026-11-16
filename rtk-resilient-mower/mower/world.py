"""The simulated yard: lawn boundary, keep-out obstacles and RTK sky-view zones.

Coordinates are local ENU metres (x = east, y = north), the same kind of frame
robot_localization's navsat_transform produces from GPS fixes in ROS 2.
"""
from __future__ import annotations

from dataclasses import dataclass

from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union

# Sky-view classes, best to worst.
OPEN = "open"                  # clear sky: RTK fixed once ambiguities are resolved
MULTIPATH = "multipath"        # next to walls: reflections bias the float solution
CANOPY_LIGHT = "canopy_light"  # thin canopy: mostly float, brief fixed windows
CANOPY_DEEP = "canopy_deep"    # dense canopy: no usable solution at all

SEVERITY = {OPEN: 0, MULTIPATH: 1, CANOPY_LIGHT: 2, CANOPY_DEEP: 3}


@dataclass(frozen=True)
class Zone:
    name: str
    kind: str
    geometry: Polygon


@dataclass
class Yard:
    boundary: Polygon
    keep_out: dict
    zones: list
    dock: tuple                # (x, y, heading_rad)
    deck_width: float = 0.50   # cutting width (m)
    clearance: float = 0.40    # distance kept between the robot's centre and any edge (m)

    @property
    def obstacles(self):
        return unary_union(list(self.keep_out.values()))

    def free_space(self):
        """Where the robot's centre may drive."""
        inner = self.boundary.buffer(-self.clearance, join_style="mitre")
        return inner.difference(self.obstacles.buffer(self.clearance, quad_segs=4))

    def mowable_area(self):
        """Grass the cutting deck can physically reach: the coverage denominator."""
        reach = self.clearance - self.deck_width / 2.0
        inner = self.boundary.buffer(-reach, join_style="mitre")
        return inner.difference(self.obstacles.buffer(reach, quad_segs=8))


def suburban_yard(dense_maple: bool = True) -> Yard:
    """An L-shaped back yard (~390 m²) wrapping a house, with a flower bed and three trees.

    `dense_maple=False` is the what-if where the maple's dense core has been thinned out, so the
    whole maple canopy behaves like light canopy (float with brief fixed windows).
    """
    boundary = Polygon([(0, 0), (26, 0), (26, 18), (11, 18), (11, 11), (0, 11)])
    house = box(0, 11, 11, 18)
    trees = {  # centre, trunk radius, canopy radius
        "oak": (Point(5.0, 5.0), 0.30, 3.2),
        "maple": (Point(21.5, 13.5), 0.40, 4.2),
        "birch": (Point(23.0, 3.5), 0.25, 2.6),
    }
    keep_out = {
        "house": house,
        "flower_bed": Polygon([(15.5, 4.5), (19.5, 4.0), (20.5, 6.5), (18.0, 8.5), (15.0, 7.0)]),
    }
    # The two-storey corner of the house reflects signals onto the lawn beside it.
    zones = [Zone("two-storey walls", MULTIPATH,
                  box(8.0, 8.5, 13.5, 11.0).union(box(11.0, 11.0, 13.5, 18.0)).intersection(boundary))]
    for name, (centre, trunk, canopy) in trees.items():
        keep_out[f"{name}_trunk"] = centre.buffer(trunk, quad_segs=8)
        zones.append(Zone(f"{name} canopy", CANOPY_LIGHT, centre.buffer(canopy, quad_segs=16)))
    if dense_maple:
        zones.append(Zone("maple dense core", CANOPY_DEEP, trees["maple"][0].buffer(2.8, quad_segs=16)))
    return Yard(boundary=boundary, keep_out=keep_out, zones=zones, dock=(1.0, 1.0, 0.0))
