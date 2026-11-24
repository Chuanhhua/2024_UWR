"""Planar EKF on the Gazebo sim. Pick the robot by config name (without .yaml):

  ros2 launch kalman_filter ekf_planar_sim.launch.py config:=sim_7sonar   # with uwr_gazebo sim.launch.py config:=sim_7sonar
  ros2 launch kalman_filter ekf_planar_sim.launch.py config:=sim_3sonar   # with uwr_gazebo sim.launch.py config:=sim_3sonar

Default: sim_7sonar. Settings come from two files with that name:
  kalman_filter/config/ekf_planar/<config>.yaml   the EKF: sonar topics and layout, sonar model
  uwr_gazebo/config/<config>.yaml                 the sim: start pose (spawn) and sonar_max_range,
                                                  so they are set in one place for both
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_ekf(context):
    # The sim config is read here in Python (a missing name fails with "No such file").
    name = LaunchConfiguration("config").perform(context)
    params_file = os.path.join(
        get_package_share_directory("kalman_filter"), "config", "ekf_planar", name + ".yaml"
    )
    with open(
        os.path.join(get_package_share_directory("uwr_gazebo"), "config", name + ".yaml")
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
    return [ekf_planar_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value="sim_7sonar",
                description="config name (without .yaml) in kalman_filter/config/ekf_planar/ "
                "and uwr_gazebo/config/, e.g. sim_7sonar or sim_3sonar",
            ),
            OpaqueFunction(function=make_ekf),
        ]
    )
