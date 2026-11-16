"""ROS 2 node: a GPS message and a gap to the nearest hazard in, a speed limit out.

Uses the same supervisor as the Python yard. The GPS here is whoever publishes it:
the check script, or a recording played later. This node does not invent a yard.

Topics are the same as the C++ node's (ros2/mower_supervisor_cpp/src/node.cpp):
/gnss/fix, optional /gnss/rtk_quality (NMEA GGA quality: 4 fixed, 5 float), /hazard_clearance
and /mower/command in; /mower/speed_limit, /mower/mode and /mower/event out.
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, String, UInt8

# Yard code. In the image this is /work. Locally it is the mower repo root.
_ROOT = Path(os.environ.get("MOWER_ROOT", Path(__file__).resolve().parents[3]))
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from mower.replay_nmea import quality_code  # noqa: E402
from mower.show_cases import quality_from_status  # noqa: E402
from mower.supervisor import Mode, Supervisor  # noqa: E402


class MowerSupervisor(Node):
    def __init__(self):
        super().__init__("mower_supervisor")
        self.supervisor = Supervisor()
        self.clearance_m = 10.0
        self.gga_quality = None
        self.quality_time = None
        self.quality_timeout = float(self.declare_parameter("rtk_quality_timeout", 2.0).value)
        self.create_subscription(Float64, "/hazard_clearance", self._on_clearance, 10)
        self.create_subscription(UInt8, "/gnss/rtk_quality", self._on_quality, 10)
        self.create_subscription(NavSatFix, "/gnss/fix", self._on_fix, 10)
        self.create_subscription(String, "/mower/command", self._on_command, 10)
        self.speed_pub = self.create_publisher(Float64, "/mower/speed_limit", 10)
        self.mode_pub = self.create_publisher(String, "/mower/mode", 10)
        self.event_pub = self.create_publisher(String, "/mower/event", 10)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_clearance(self, msg):
        self.clearance_m = float(msg.data)

    def _on_quality(self, msg):
        self.gga_quality, self.quality_time = int(msg.data), self._now()

    def _on_command(self, msg):
        if msg.data == "retreat":
            self.supervisor.force(self._now(), Mode.RETREAT)
            self._publish_state()

    def _on_fix(self, msg):
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        if t <= 0.0:                      # a driver that leaves the stamp empty
            t = self._now()
        # The larger of the east and north variances: a fix that is good along a wall and poor
        # across it must count as poor.
        cov = msg.position_covariance
        variance = max(cov[0], cov[4]) if msg.position_covariance_type else 0.0
        sigma = math.sqrt(max(float(variance), 0.0))
        # NavSatStatus cannot tell RTK fixed from RTK float, so the receiver's own code wins.
        fresh = self.quality_time is not None and self._now() - self.quality_time <= self.quality_timeout
        quality = quality_code(self.gga_quality) if fresh else quality_from_status(int(msg.status.status))
        self.supervisor.update(t, quality, sigma, self.clearance_m)
        self._publish_state()
        if self.supervisor.hold_expired:
            event = String()
            event.data = "skip"
            self.event_pub.publish(event)

    def _publish_state(self):
        speed = Float64()
        speed.data = float(self.supervisor.speed_limit)
        self.speed_pub.publish(speed)
        name = String()
        name.data = Mode(self.supervisor.mode).name
        self.mode_pub.publish(name)


def main():
    rclpy.init()
    node = MowerSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
