#!/usr/bin/env python3
"""Deterministic waypoint driver (guide section 4.4).

Follows the route in sim/routes/<sequence>.yaml using the simulator's ground-truth
pose as feedback: rotate in place towards the next waypoint, drive straight at a
fixed speed, repeat. Exits with code 0 when the route is finished.
"""
import math
import sys

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

SETTLE_S = 2.0        # standstill at start and end (sim time)
YAW_TOL = 0.01        # rad, turn-in-place tolerance
POS_TOL = 0.05        # m
K_YAW_TURN = 2.5
K_YAW_DRIVE = 1.5


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class RouteDriver(Node):
    def __init__(self, route):
        super().__init__("route_driver")
        self.v, self.w = route["linear_speed"], route["angular_speed"]
        self.goals = [("goto", wp) for wp in route["waypoints"]]
        if "final_yaw" in route:
            self.goals.append(("yaw", route["final_yaw"]))
        self.idx, self.phase = 0, "settle"
        self.t0 = self.t_end = None
        self.done = False
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/ms/ground_truth", self.on_pose, 50)

    def on_pose(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        cmd = Twist()
        if self.t0 is None:
            self.t0 = t
        if self.phase == "settle":
            if t - self.t0 >= SETTLE_S:
                self.phase = "turn"
        elif self.idx >= len(self.goals):
            if self.t_end is None:
                self.t_end = t
                self.get_logger().info(f"route finished at sim time {t:.2f} s, pose ({p.x:.3f}, {p.y:.3f}, {yaw:.3f})")
            self.done = t - self.t_end >= SETTLE_S
        else:
            kind, goal = self.goals[self.idx]
            if kind == "yaw":
                err = wrap(goal - yaw)
                if abs(err) < YAW_TOL:
                    self.idx += 1
                else:
                    cmd.angular.z = max(-self.w, min(self.w, K_YAW_TURN * err))
            else:
                dx, dy = goal[0] - p.x, goal[1] - p.y
                dist, err = math.hypot(dx, dy), wrap(math.atan2(dy, dx) - yaw)
                if dist < POS_TOL or (self.phase == "drive" and abs(err) > math.pi / 2):  # reached or passed
                    self.idx, self.phase = self.idx + 1, "turn"
                elif self.phase == "turn":
                    if abs(err) < YAW_TOL:
                        self.phase = "drive"
                    else:
                        cmd.angular.z = max(-self.w, min(self.w, K_YAW_TURN * err))
                else:
                    cmd.linear.x = min(self.v, max(0.05, 1.5 * dist))  # slow down close to the waypoint
                    cmd.angular.z = max(-self.w, min(self.w, K_YAW_DRIVE * err))
        self.pub.publish(cmd)


def main():
    route = yaml.safe_load(open(sys.argv[1]))
    rclpy.init()
    node = RouteDriver(route)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
