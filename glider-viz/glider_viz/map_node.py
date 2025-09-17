"""
    Jason Hughes
    Septemeber 2025

    ros node
"""

import utm
import rclpy

from rclpy.node import Node
from nav_msgs.msg import Odometry

from scipy.spatial.transform import Rotation as R

from glider_viz.app import MapApp


class MapNode(Node):

    def __init__(self) -> None:
        super(MapNode, self).__init__('glider_map_viz')

        self.odom_sub_ = self.create_subscription(Odometry, "/odom", self.odom_callback, 1)

        self.declare_parameter("path", ".")
        path = self.get_parameter('path').get_parameter_value().string_value
        self.declare_parameter("app.ip_address", "127.0.0.1")
        ip = self.get_parameter('app.ip_address').get_parameter_value().string_value
        self.declare_parameter("app.port", 8080)
        port = self.get_parameter('app.port').get_parameter_value().integer_value
        self.declare_parameter("map.lat", 0.0)
        lat = self.get_parameter('map.lat').get_parameter_value().double_value
        self.declare_parameter("map.lon", 0.0)
        lon = self.get_parameter('map.lon').get_parameter_value().double_value
        self.declare_parameter("map.zone_number", 18)
        self.zone_number = self.get_parameter('map.zone_number').get_parameter_value().integer_value
        self.declare_parameter("map.zone_id", "S")
        self.zone_id = self.get_parameter('map.zone_id').get_parameter_value().string_value       

        self.map_ = MapApp(path, ip, port, (lat,lon))
        self.map_.run_in_thread()

    def odom_callback(self, msg : Odometry) -> None:
        lat, lon = utm.to_latlon(msg.pose.pose.position.x, msg.pose.pose.position.y, self.zone_number, self.zone_id)

        orientation_q = msg.pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        
        rotation = R.from_quat(quaternion)
        euler_angles = rotation.as_euler('xyz', degrees=True)
        
        self.map_.update(lat, lon, euler_angles[2])
