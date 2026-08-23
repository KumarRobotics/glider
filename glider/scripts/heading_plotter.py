#!/usr/bin/env python3
"""
heading_plotter_node.py

ROS2 node that subscribes to three heading sources, normalizes them all to
0-360 degrees, and live-plots them for comparison:

  /mavros/global_position/compass_hdg   (std_msgs/Float64)   -> already 0-360 deg
  /ublox_raw/navheading                 (sensor_msgs/Imu)    -> yaw from quaternion, -pi..pi -> 0-360 deg
  /gekf/state                           (gps_common/GPSFix)  -> heading in radians, -pi..pi -> 0-360 deg

NOTE on /gekf/state:
  The standard gps_common/GPSFix message doesn't have an obvious single
  "heading in radians" field (its `track` field is normally already in
  degrees per the GPSFix spec). Since you said your /gekf/state heading is
  in radians (-pi..pi), this script reads it from `msg.track` and treats it
  as radians. If your GEKF publisher actually puts heading in a different
  field (e.g. `dip`, `err_track`, or a custom message), change the field
  name in `gekf_cb()` below accordingly.

Usage:
  ros2 run <your_package> heading_plotter_node.py
  (or just: python3 heading_plotter_node.py, as long as ROS2 env is sourced)
"""

import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from gps_msgs.msg import GPSFix

import matplotlib.pyplot as plt
import matplotlib.animation as animation


def quaternion_to_yaw(q):
    """Extract yaw (rotation about Z) from a geometry_msgs/Quaternion, -pi..pi."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def rad_to_deg_0_360(rad):
    """Convert radians (-pi..pi or any range) to degrees wrapped to 0-360."""
    return math.degrees(rad) % 360.0


class HeadingPlotter(Node):
    def __init__(self, history_len=1000, window_seconds=60.0):
        super().__init__('heading_plotter')

        self.window_seconds = window_seconds
        self.t0 = time.time()

        self.compass_t = deque(maxlen=history_len)
        self.compass_h = deque(maxlen=history_len)

        self.nav_t = deque(maxlen=history_len)
        self.nav_h = deque(maxlen=history_len)

        self.gekf_t = deque(maxlen=history_len)
        self.gekf_h = deque(maxlen=history_len)

        # mavros and many sensor drivers publish BEST_EFFORT, not the default
        # RELIABLE. A mismatched QoS silently drops the connection (you'll
        # see a "incompatible QoS" warning and never receive any messages),
        # so match best-effort/volatile here explicitly.
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            Float64, '/mavros/global_position/compass_hdg', self.compass_cb, best_effort_qos)
        self.create_subscription(
            Imu, '/ublox_raw/navheading', self.navheading_cb, best_effort_qos)
        self.create_subscription(
            GPSFix, '/gekf/dgps', self.gekf_cb, best_effort_qos)

        self.get_logger().info('heading_plotter node started, subscribing to 3 topics.')

    def compass_cb(self, msg: Float64):
        heading = msg.data 
        self.compass_t.append(time.time() - self.t0)
        self.compass_h.append(heading)

    def navheading_cb(self, msg: Imu):
        yaw_rad = quaternion_to_yaw(msg.orientation)  # -pi..pi
        heading = rad_to_deg_0_360(-yaw_rad)
        self.nav_t.append(time.time() - self.t0)
        self.nav_h.append(heading)

    def gekf_cb(self, msg: GPSFix):
        # See NOTE at top of file if your heading field isn't `track`.
        yaw_rad = msg.track  # assumed radians, -pi..pi
        heading = rad_to_deg_0_360(yaw_rad)
        self.gekf_t.append(time.time() - self.t0)
        self.gekf_h.append(heading)


def main(args=None):
    rclpy.init(args=args)
    node = HeadingPlotter()

    fig, ax = plt.subplots(figsize=(10, 6))
    line_compass, = ax.plot([], [], '.-', ms=3, label='Compass Hdg (mavros)', color='tab:blue')
    line_nav, = ax.plot([], [], '.-', ms=3, label='Nav Heading (ublox IMU yaw)', color='tab:orange')
    line_gekf, = ax.plot([], [], '.-', ms=3, label='GEKF State', color='tab:green')

    ax.set_ylim(0, 360)
    ax.set_yticks(range(0, 361, 45))
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Heading (deg)')
    ax.set_title('Heading Comparison')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.4)

    def update(_frame):
        # Pump ROS callbacks without blocking the plot loop.
        rclpy.spin_once(node, timeout_sec=0.0)

        line_compass.set_data(node.compass_t, node.compass_h)
        line_nav.set_data(node.nav_t, node.nav_h)
        line_gekf.set_data(node.gekf_t, node.gekf_h)

        all_t = list(node.compass_t) + list(node.nav_t) + list(node.gekf_t)
        if all_t:
            t_max = max(all_t)
            t_min = max(0.0, t_max - node.window_seconds)
            ax.set_xlim(t_min, t_max + 1.0)

        return line_compass, line_nav, line_gekf

    ani = animation.FuncAnimation(
        fig, update, interval=100, blit=False, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
