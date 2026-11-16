"""Drive the mower with Nav2 in the Gazebo yard, under the C++ supervisor.

Nav2's controller follows the path. This script is the mission layer above it:

  NOMINAL, DEGRADED  pass the supervisor's speed on to Nav2 (nav2_msgs/SpeedLimit).
  HOLD               cancel the path. Nav2 reads a speed limit of 0 as "no limit", and a crawl
                     is not a stop, so the action is cancelled and the mower stands still.
  skip               the hold timed out: give up the strip by the wall, tell the supervisor to
                     retreat, and send Nav2 a path back towards the last RTK-fixed spot.
  RETREAT            follow that path at the retreat speed until RTK is fixed again.

The yard is a straight lane: open grass, a tree canopy, open grass, then a wall. GPS quality is
scripted from the pose Gazebo reports (drive.py says how). The check passes when the mower ran
at full speed in the open, slowed under the tree, stood still beside the wall with a zero
command, retreated, recovered full speed, and never touched the wall.
"""
from __future__ import annotations

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowPath
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String, UInt8
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

TREE_X, TREE_Y = 10.0, 1.5
TRUNK_R = 0.2
CANOPY_R = 4.0
WALL_FACE_X = 19.9
BODY_RADIUS = 0.35   # the mower body in yard.world is 0.7 m long
RETREAT_TO_X = 15.0       # open grass between the canopy and the wall
GGA_FIXED, GGA_FLOAT = 4, 5


def clearance_m(x, y):
    """Free space between the mower's body and the nearest hazard, in metres."""
    trunk = math.hypot(x - TREE_X, y - TREE_Y) - TRUNK_R - BODY_RADIUS
    wall = WALL_FACE_X - x - BODY_RADIUS
    return min(trunk, wall)


def gnss_for_pose(x, y):
    gap = clearance_m(x, y)
    dist = math.hypot(x - TREE_X, y - TREE_Y)
    wall_gap = WALL_FACE_X - x
    if wall_gap < 1.0:
        return 0, 0.20, gap, "next to a wall"
    if dist < CANOPY_R:
        return 0, 0.15, gap, "under a tree"
    return 2, 0.02, gap, "open grass"


class Nav2Drive(Node):
    def __init__(self):
        super().__init__("nav2_drive")
        self.x = None
        self.y = None
        self.vx = 0.0
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        static = TransformStamped()
        static.header.stamp = self.get_clock().now().to_msg()
        static.header.frame_id = "map"
        static.child_frame_id = "odom"
        static.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(static)
        self.mode = None
        self.speed = None
        self.cmd = None
        self.events = []
        self.goal_future = None
        self.goal_handle = None
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(String, "/mower/mode", self._on_mode, 10)
        self.create_subscription(Float64, "/mower/speed_limit", self._on_speed, 10)
        self.create_subscription(String, "/mower/event", self._on_event, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.clearance_pub = self.create_publisher(Float64, "/hazard_clearance", 10)
        self.fix_pub = self.create_publisher(NavSatFix, "/gnss/fix", 10)
        self.quality_pub = self.create_publisher(UInt8, "/gnss/rtk_quality", 10)
        self.command_pub = self.create_publisher(String, "/mower/command", 10)
        self.limit_pub = self.create_publisher(SpeedLimit, "/speed_limit", 10)
        self.change = self.create_client(ChangeState, "/controller_server/change_state")
        self.follow = ActionClient(self, FollowPath, "follow_path")
        self._last_fix = 0.0

    def _on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.vx = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        pose = TransformStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "odom"
        pose.child_frame_id = "base_link"
        pose.transform.translation.x = self.x
        pose.transform.translation.y = self.y
        pose.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(pose)

    def _on_mode(self, msg):
        self.mode = msg.data

    def _on_speed(self, msg):
        self.speed = float(msg.data)
        if self.speed > 0.0:          # a stop is not a speed limit: see hold() below
            limit = SpeedLimit()
            limit.percentage = False
            limit.speed_limit = self.speed
            self.limit_pub.publish(limit)

    def _on_event(self, msg):
        self.events.append(msg.data)

    def _on_cmd(self, msg):
        self.cmd = float(msg.linear.x)

    def publish_fix(self):
        if self.x is None:
            return
        now = time.time()
        if now - self._last_fix < 0.2:
            return
        self._last_fix = now
        status, sigma, gap, _place = gnss_for_pose(self.x, self.y)
        clearance = Float64()
        clearance.data = float(gap)
        self.clearance_pub.publish(clearance)
        quality = UInt8()
        quality.data = GGA_FIXED if status == 2 else GGA_FLOAT
        self.quality_pub.publish(quality)
        fix = NavSatFix()
        fix.header.stamp.sec = int(now)
        fix.header.stamp.nanosec = int(round((now - int(now)) * 1e9))
        fix.status.status = int(status)
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.position_covariance[0] = float(sigma) ** 2
        fix.position_covariance[4] = float(sigma) ** 2
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.fix_pub.publish(fix)

    def transition(self, executor, transition_id):
        if not self.change.wait_for_service(timeout_sec=30.0):
            return False
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self.change.call_async(request)
        end = time.time() + 90.0
        while time.time() < end and not future.done():
            executor.spin_once(timeout_sec=0.1)
            self.publish_fix()
        return future.done() and future.result() is not None and future.result().success

    def send_path(self, path):
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "goal_checker"
        self.goal_handle = None
        self.goal_future = self.follow.send_goal_async(goal)

    def hold(self):
        """Stop for real: cancel the path. Returns True once Nav2 has been told."""
        if self.goal_future is not None and self.goal_future.done():
            self.goal_handle = self.goal_future.result()
            self.goal_future = None
        if self.goal_handle is None:
            return self.goal_future is None
        self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        return True

    def command(self, text):
        msg = String()
        msg.data = text
        self.command_pub.publish(msg)


def line_path(x0, x1, step=0.5):
    """Straight lane along y = 0 from x0 to x1, poses facing the direction of travel."""
    path = Path()
    path.header.frame_id = "odom"
    forward = x1 >= x0
    n = max(1, int(math.ceil(abs(x1 - x0) / step)))
    for i in range(n + 1):
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.pose.position.x = x0 + (x1 - x0) * i / n
        if forward:
            pose.pose.orientation.w = 1.0
        else:
            pose.pose.orientation.z = 1.0   # yaw = pi
        path.poses.append(pose)
    return path


def main():
    rclpy.init()
    node = Nav2Drive()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    failures = []
    steps = ("open grass", "under a tree", "next to a wall", "hold timed out", "retreat", "back in open grass")
    seen = dict.fromkeys(steps, False)

    end = time.time() + 40.0
    while node.x is None and time.time() < end:
        executor.spin_once(timeout_sec=0.1)
    if node.x is None:
        print("FAILED")
        print("Gazebo did not publish the mower position.")
        rclpy.shutdown()
        sys.exit(1)

    for _ in range(20):
        node.publish_fix()
        executor.spin_once(timeout_sec=0.05)

    if not node.transition(executor, Transition.TRANSITION_CONFIGURE):
        failures.append("Nav2 controller did not configure. See /tmp/mower-nav2.log")
    elif not node.transition(executor, Transition.TRANSITION_ACTIVATE):
        failures.append("Nav2 controller did not start. See /tmp/mower-nav2.log")
    elif not node.follow.wait_for_server(timeout_sec=20.0):
        failures.append("Nav2 path action is not available.")
    else:
        node.send_path(line_path(0.0, 22.0))    # the lane runs on into the wall on purpose
        phase = "outbound"
        max_x = node.x
        hold_started = hold_x = None
        hold_moved = hold_cmd = hold_speed = 0.0
        retreat_asked = 0.0
        deadline = time.time() + 240.0
        while time.time() < deadline and phase != "done":
            node.publish_fix()
            executor.spin_once(timeout_sec=0.05)
            if node.x is None or node.mode is None or node.speed is None or node.cmd is None:
                continue
            now = time.time()
            max_x = max(max_x, node.x)
            _status, _sigma, _gap, place = gnss_for_pose(node.x, node.y)

            if phase == "outbound":
                if place == "open grass" and node.mode == "NOMINAL" and abs(node.speed - 0.50) < 0.05 and node.cmd > 0.35:
                    if not seen[place]:
                        print(f"open grass: mode NOMINAL, speed limit {node.speed:.2f}, Nav2 command {node.cmd:.2f}")
                    seen[place] = True
                elif (place == "under a tree" and node.mode == "DEGRADED" and 0.20 <= node.speed < 0.50
                      and 0.20 <= node.cmd <= 0.45):
                    if not seen[place]:
                        print(f"under a tree: mode DEGRADED, speed limit {node.speed:.2f}, Nav2 command {node.cmd:.2f}")
                    seen[place] = True
                if node.mode == "HOLD" and node.hold():
                    phase, hold_started = "hold", now
                    node.events.clear()

            elif phase == "hold":
                # Give Nav2 one second to act on the cancel, then the mower must not move at all.
                if hold_x is None and now - hold_started >= 1.0:
                    hold_x = node.x
                if hold_x is not None:
                    hold_moved = max(hold_moved, abs(node.x - hold_x))
                    hold_cmd = max(hold_cmd, abs(node.cmd))
                    hold_speed = max(hold_speed, node.vx)
                if "skip" in node.events:
                    waited = now - hold_started
                    print(f"next to a wall: mode HOLD, path cancelled, Nav2 command {hold_cmd:.2f}, "
                          f"mower moved {100 * hold_moved:.1f} cm in {waited:.0f} s")
                    seen["next to a wall"] = (hold_x is not None and place == "next to a wall" and hold_cmd < 1e-3
                                              and hold_speed < 0.01 and hold_moved < 0.02)
                    print("hold timed out: strip by the wall given up, supervisor told to retreat")
                    seen["hold timed out"] = True
                    node.command("retreat")
                    phase, retreat_asked = "retreat-wait", now

            elif phase == "retreat-wait":
                if node.mode == "RETREAT":
                    node.send_path(line_path(node.x, RETREAT_TO_X))
                    phase = "retreat"
                elif now - retreat_asked > 1.0:
                    node.command("retreat")
                    retreat_asked = now

            elif phase == "retreat":
                if (node.mode == "RETREAT" and abs(node.speed - 0.30) < 0.02 and 0.20 <= node.cmd <= 0.32
                        and hold_x is not None and node.x < hold_x - 0.05):
                    if not seen["retreat"]:
                        print(f"retreat: mode RETREAT, speed limit {node.speed:.2f}, Nav2 command {node.cmd:.2f}, "
                              "driving away from the wall")
                    seen["retreat"] = True
                if seen["retreat"] and place == "open grass" and node.mode == "NOMINAL":
                    print(f"back in open grass: mode NOMINAL at x = {node.x:.1f} m")
                    seen["back in open grass"] = True
                    phase = "done"

        node.hold()                     # the check is over: stop the retreat where it is
        for _ in range(10):
            executor.spin_once(timeout_sec=0.05)
        gap = WALL_FACE_X - (max_x + BODY_RADIUS)
        print(f"closest the mower body came to the wall: {gap:.2f} m")
        if gap <= 0.0:
            failures.append("the mower touched the wall")
        for name, ok in seen.items():
            if not ok:
                failures.append(f"did not see: {name}")

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
