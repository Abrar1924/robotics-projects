from shapely.geometry import LineString, Point, box

from mower.planner import TransitPlanner, plan_coverage


def test_plan_covers_free_space_with_an_obstacle():
    free = box(0, 0, 10, 6).difference(Point(5, 3).buffer(1.0))
    plan = plan_coverage(free, deck_width=0.5, start=(0.5, 0.5))
    covered = LineString(plan.xy).buffer(0.25)
    assert covered.intersection(free).area / free.area > 0.995
    assert free.buffer(1e-6).contains(LineString(plan.xy))  # never drives through the obstacle


def test_transit_goes_around_a_wall():
    free = box(0, 0, 10, 6).difference(box(4, 0, 6, 5))  # wall from the bottom, gap at the top
    route = LineString(TransitPlanner(free).path((1, 1), (9, 1)))
    assert free.buffer(1e-6).contains(route)
    assert route.length > 11.5  # up, over the wall and back down


def test_sweep_angle_follows_the_long_side():
    plan = plan_coverage(box(0, 0, 20, 4), deck_width=0.5, start=(0.5, 0.5))
    assert plan.angle_deg in (0.0, 180.0)  # passes along the 20 m side need the fewest turns
