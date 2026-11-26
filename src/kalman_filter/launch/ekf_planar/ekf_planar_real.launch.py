"""Planar EKF on the real 7-sonar robot. Everything (topics, sonar layout,
noise, start pose, TF) comes from kalman_filter/config/ekf_planar/ekf_real_7sonar.yaml:

  ros2 launch kalman_filter ekf_planar_real.launch.py

Start the sonar and IMU drivers separately. The EKF
publishes /ekf_planar/odom, /ekf_planar/sonar_diag and TF world -> base_link.

Every launch is recorded: bag_recorder records the EKF output,
IMU, sonars and TF from launch into kf_runs/ekf_real_<date>/. Ctrl+C closes
the bag, so each launch is one bag.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("kalman_filter"),
        "config",
        "ekf_planar",
        "ekf_real_7sonar.yaml",
    )

    ekf_planar_node = Node(
        package="kalman_filter",
        executable="ekf_planar_node",
        name="ekf_planar",
        output="screen",
        parameters=[params_file, {"use_sim_time": False}],
    )

    recorder = Node(
        package="kalman_filter",
        executable="bag_recorder",
        name="bag_recorder",
        output="screen",
        parameters=[
            {
                "prefix": "ekf_real",
                "topics": [
                    "/ekf_planar/odom",
                    "/ekf_planar/sonar_diag",
                    "/imu/data",
                    "/tf",
                    "/tf_static",
                ],
                "regex": "/sonar/.*",
            }
        ],
    )

    return LaunchDescription([ekf_planar_node, recorder])
