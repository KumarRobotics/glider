"""
    Jason Hughes
    September 2025

    launch the node
"""

import rclpy
from rclpy.executors import SingleThreadedExecutor
from glider_viz.map_node import MapNode


def main(args=None) -> None:
    rclpy.init(args=args)

    node = MapNode()
    executor = SingleThreadedExecutor()

    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
