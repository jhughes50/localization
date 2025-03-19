"""
    Jason Hughes
    January 2025

    Launch the factor graph node
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    share_dir = get_package_share_directory("localization")

    return LaunchDescription([Node(package="localization",
                                   executable="localizer",
                                   name="localizer",
                                   output="screen",
                                   emulate_tty=True,
                                   #arguments=['--ros-args', '--log-level', 'DEBUG'],
                                   remappings=[("/gps", "/fmu/out/vehicle_gps_position"),
                                               ("/imu", "/fmu/out/sensor_combined"),
                                               ("/attitude", "/fmu/out/vehicle_attitude")],
                                   parameters=[{"sim": True}])])

