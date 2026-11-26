"""Planar EKF on the Gazebo sim (7-sonar robot), with uwr_gazebo sim.launch.py config:=gazebo_7sonar:

  ros2 launch kalman_filter ekf_planar_sim.launch.py

Every launch is recorded: bag_recorder records the EKF output, ground truth,
IMU, sonars, TF and clock from launch into kf_runs/ekf_sim_<date>/. Ctrl+C
closes the bag, so each launch is one bag (runs restarted with reset-run share it).

Settings come from two files:
  kalman_filter/config/ekf_planar/ekf_sim_7sonar.yaml   the EKF: sonar topics and layout, sonar model
  uwr_gazebo/config/gazebo_7sonar.yaml                  the sim: start pose (spawn) and sonar_max_range,
                                                        so they are set in one place for both
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("kalman_filter"),
        "config",
        "ekf_planar",
        "ekf_sim_7sonar.yaml",
    )
    with open(
        os.path.join(
            get_package_share_directory("uwr_gazebo"), "config", "gazebo_7sonar.yaml"
        )
    ) as f:
        sim = yaml.safe_load(f)

    # start pose and sonar range from the sim config (spawn is in the pool frame)
    from_sim = {
        "init_x": float(sim["spawn"]["x"]),
        "init_y": float(sim["spawn"]["y"]),
        "init_psi": float(sim["spawn"]["yaw"]),
        "max_range": float(sim["sonar_max_range"]),
        "use_sim_time": True,
    }

    ekf_planar_node = Node(
        package="kalman_filter",
        executable="ekf_planar_node",
        name="ekf_planar",
        output="screen",
        parameters=[params_file, from_sim],
    )
    recorder = Node(
        package="kalman_filter",
        executable="bag_recorder",
        name="bag_recorder",
        output="screen",
        parameters=[
            {
                "prefix": "ekf_sim",
                "topics": [
                    "/ekf_planar/odom",
                    "/ekf_planar/sonar_diag",
                    "/pose_topic_gt",
                    "/imu/data",
                    "/tf",
                    "/tf_static",
                    "/clock",
                ],
                "regex": "/sonar/.*",
                "use_sim_time": True,
            }
        ],
    )
    return LaunchDescription([ekf_planar_node, recorder])
