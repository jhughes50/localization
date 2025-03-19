"""
    Jason Hughes
    February 2025

    start the node
"""

import rospy 
from localizer.factor_node import FactorGraphNode

if __name__ == "__main__":
    rospy.init_node("factor_localizer")
    node = FactorGraphNode()

    node.spin()
    
