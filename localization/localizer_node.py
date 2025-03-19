""" 
    Jason Hughes
    January 2025

    Script to start localization node
"""

import rclpy
import threading

from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from localization.factor_graph_node import FactorGraphNode

def main(args=None) -> None:
    rclpy.init(args=args)
    
    config = {}
    node = FactorGraphNode(config)
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
