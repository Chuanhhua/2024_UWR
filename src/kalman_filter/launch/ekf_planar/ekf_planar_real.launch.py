"""Planar EKF on the real robot. Everything (topics, sonar layout, noise, start
pose, TF) comes from a param file in kalman_filter/config/ekf_planar/, picked by
name (without .yaml):

  ros2 launch kalman_filter ekf_planar_real.launch.py config:=real_7sonar

Default: real_7sonar. Start the sonar and IMU drivers separately. The EKF
publishes /ekf_planar/odom, /ekf_planar/sonar_diag and TF world -> base_link.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [
            FindPackageShare("kalman_filter"),
            "config",
            "ekf_planar",
            [LaunchConfiguration("config"), ".yaml"],
        ]
    )

    ekf_planar_node = Node(
        package="kalman_filter",
        executable="ekf_planar_node",
        name="ekf_planar",
        output="screen",
        parameters=[params_file, {"use_sim_time": False}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value="real_7sonar",
                description="param file name in kalman_filter/config/ekf_planar/ "
                "(without .yaml), e.g. real_7sonar",
            ),
            ekf_planar_node,
        ]
    )
