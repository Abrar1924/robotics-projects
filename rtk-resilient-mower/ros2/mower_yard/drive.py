"""Move the mower in Gazebo and publish the GPS that place produces.

Clearance is the free space between the mower's body, at the pose Gazebo
reports, and the tree trunk or the wall face. GPS quality is scripted from that same pose, not sensed: under
the canopy the reading is a float, next to the wall it is worse, and open grass
stays fixed. The C++ supervisor answers on /mower/mode. This file does not
decide the speed.

One stop repeats the tree with the message a real driver can send: status
GBAS_FIX and a 3 cm covariance while the receiver itself says "float". The
supervisor has to believe the receiver's quality code and slow down.
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String, UInt8

ROOT = Path(os.environ.get("MOWER_ROOT", Path(__file__).resolve().parents[2]))
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from mower.replay_nmea import quality_code  # noqa: E402
from mower.supervisor import Supervisor  # noqa: E402

TREE_X, TREE_Y = 10.0, 1.5
TRUNK_R = 0.2
CANOPY_R = 4.0
WALL_FACE_X = 19.9
BODY_RADIUS = 0.35   # the mower body in yard.world is 0.7 m long

# Where to put the mower, the timestamp on the GPS message, and what the driver claims.
# None: the driver reports what the place produces. Otherwise (NavSatStatus, sigma in metres).
OPTIMISTIC = (2, 0.03)   # GBAS_FIX and 3 cm, sent while the receiver says "float"
STOPS = (
    (0.0, 0.0, 0.0, None),
    (7.0, 0.0, 1.0, None),
    (7.0, 0.0, 1.5, OPTIMISTIC),
    (19.2, 0.0, 2.0, None),
    (19.2, 0.0, 30.0, None),
)
GGA_FIXED, GGA_FLOAT = 4, 5


def clearance_m(x, y):
    """Free space between the mower's body and the nearest hazard, in metres."""
    trunk = math.hypot(x - TREE_X, y - TREE_Y) - TRUNK_R - BODY_RADIUS
    wall = WALL_FACE_X - x - BODY_RADIUS
    return min(trunk, wall)


def gnss_for_pose(x, y):
    """Return status, sigma metres, clearance metres, and a short place name."""
    gap = clearance_m(x, y)
    dist = math.hypot(x - TREE_X, y - TREE_Y)
    wall_gap = WALL_FACE_X - x
    if wall_gap < 1.0:
        return 0, 0.20, gap, "next to a wall"
    if dist < CANOPY_R:
        return 0, 0.15, gap, "under a tree"
    return 2, 0.02, gap, "open grass"


class Drive(Node):
    def __init__(self):
        super().__init__("yard_drive")
        from gazebo_msgs.srv import GetEntityState, SetEntityState

        self._Get = GetEntityState
        self._Set = SetEntityState
        self.set_cli = self.create_client(SetEntityState, "/set_entity_state")
        self.get_cli = self.create_client(GetEntityState, "/get_entity_state")
        self.clearance_pub = self.create_publisher(Float64, "/hazard_clearance", 10)
        self.fix_pub = self.create_publisher(NavSatFix, "/gnss/fix", 10)
        self.quality_pub = self.create_publisher(UInt8, "/gnss/rtk_quality", 10)
        self.create_subscription(String, "/mower/mode", self._on_mode, 10)
        self.create_subscription(Float64, "/mower/speed_limit", self._on_speed, 10)
        self.create_subscription(String, "/mower/event", self._on_event, 10)
        self.mode = None
        self.speed = None
        self.events = []

    def _on_mode(self, msg):
        self.mode = msg.data

    def _on_speed(self, msg):
        self.speed = msg.data

    def _on_event(self, msg):
        self.events.append(msg.data)

    def gazebo_ready(self, executor):
        deadline = self.get_clock().now().nanoseconds + 90 * 1e9
        while self.get_clock().now().nanoseconds < deadline:
            executor.spin_once(timeout_sec=0.2)
            if self.set_cli.service_is_ready() and self.get_cli.service_is_ready():
                return True
        return False

    def _call(self, client, request, executor, timeout=20.0):
        future = client.call_async(request)
        end = self.get_clock().now().nanoseconds + timeout * 1e9
        while self.get_clock().now().nanoseconds < end and not future.done():
            executor.spin_once(timeout_sec=0.1)
        if not future.done() or future.result() is None:
            return None
        return future.result()

    def move(self, x, y, executor):
        request = self._Set.Request()
        request.state.name = "mower"
        request.state.reference_frame = "world"
        request.state.pose.position.x = float(x)
        request.state.pose.position.y = float(y)
        request.state.pose.position.z = 0.2
        request.state.pose.orientation.w = 1.0
        ask = self._Get.Request()
        ask.name = "mower"
        ask.reference_frame = "world"
        for _ in range(8):
            moved = self._call(self.set_cli, request, executor)
            got = self._call(self.get_cli, ask, executor)
            if got is not None and got.success:
                pose = got.state.pose.position
                if math.hypot(pose.x - x, pose.y - y) <= 0.05:
                    return pose.x, pose.y
            if moved is None:
                for _spin in range(10):
                    executor.spin_once(timeout_sec=0.1)
        return None


def _stamp(msg, t):
    msg.header.stamp.sec = int(t)
    msg.header.stamp.nanosec = int(round((t - int(t)) * 1e9))


def main():
    rclpy.init()
    node = Drive()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    failures = []

    if not node.gazebo_ready(executor):
        print("FAILED")
        print("Gazebo did not start. The log is /tmp/mower-gazebo.log")
        rclpy.shutdown()
        sys.exit(1)

    print("Gazebo moved the mower. Tree trunk at x=10 m, 1.5 m beside the lane. Wall face at x=19.9 m.")
    oracle = Supervisor()

    for index, (x, y, t, claim) in enumerate(STOPS):
        pose = node.move(x, y, executor)
        if pose is None:
            failures.append(f"Gazebo did not move the mower to x={x}")
            break
        if math.hypot(pose[0] - x, pose[1] - y) > 0.05:
            failures.append(f"Gazebo pose {pose[0]:.2f},{pose[1]:.2f} is not {x},{y}")
            break
        status, sigma, gap, place = gnss_for_pose(pose[0], pose[1])
        gga = GGA_FIXED if status == 2 else GGA_FLOAT   # what the receiver itself reports here
        if claim is not None:
            status, sigma = claim
            place += ", driver says GBAS_FIX and 3 cm, receiver says float"
        node.mode = None
        node.speed = None
        node.events = []

        clearance = Float64()
        clearance.data = float(gap)
        node.clearance_pub.publish(clearance)
        quality = UInt8()
        quality.data = gga
        node.quality_pub.publish(quality)
        for _ in range(10):
            executor.spin_once(timeout_sec=0.05)

        fix = NavSatFix()
        _stamp(fix, t)
        fix.status.status = int(status)
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.position_covariance[0] = float(sigma) ** 2
        fix.position_covariance[4] = float(sigma) ** 2
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        node.fix_pub.publish(fix)
        node.mode = None
        node.speed = None
        node.events = []

        last = index == len(STOPS) - 1
        end = node.get_clock().now().nanoseconds + 5 * 1e9
        while node.get_clock().now().nanoseconds < end:
            executor.spin_once(timeout_sec=0.1)
            if node.mode is None or node.speed is None:
                continue
            if last and "skip" not in node.events:
                continue
            break

        want_mode = oracle.update(t, quality_code(gga), sigma, gap)
        want_speed = oracle.speed_limit
        want_skip = oracle.hold_expired
        last = index == len(STOPS) - 1
        if last:
            ok = "skip" in node.events
            line = "next to a wall, still bad after the wait: skip the strip"
            print(line)
            if not ok:
                failures.append("beside the wall after the wait: no skip")
        else:
            ok = node.mode == want_mode.name and node.speed is not None and abs(node.speed - want_speed) < 1e-3
            speed_txt = "none" if node.speed is None else f"{node.speed:.2f}"
            line = f"{place}: mode {node.mode}, speed {speed_txt}"
            print(line)
            if not ok:
                failures.append(f"{line} (wanted {want_mode.name} {want_speed:.2f})")
        if want_skip and not last and "skip" not in node.events:
            failures.append(f"{place}: skip missing")

    node.destroy_node()
    rclpy.shutdown()
    if failures:
        print("FAILED")
        for item in failures:
            print(item)
        sys.exit(1)
    print("PASSED")


if __name__ == "__main__":
    main()
