"""Planar EKF configured for the Gazebo tank (uwr_gazebo sim.launch.py).

Tank: 5 m x 10 m, centred on the world origin, so the pool frame origin
(left wall, near end) is at world (-2.5, -5). The robot spawns at the world
origin facing +y: pool (2.5, 5), heading 90 deg.

Sonars used: surge (field 1, forward) and sway (field 2, left). Both sit at the
body origin in the plane. Heave (field 3) points down and is not planar.
"""
import math

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kalman_filter',
            executable='ekf_planar_node',
            name='ekf_planar',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'imu_topic': '/imu/data',
                'sonar_topic': '/sbs',

                # sonars: one entry each
                'sonar_fields': [1, 2],
                'sonar_ox': [0.0, 0.0],
                'sonar_oy': [0.0, 0.0],
                'sonar_alpha': [0.0, math.pi / 2],

                # tank
                'pool_width': 5.0,
                'pool_length': 10.0,
                'pool_origin_x': -2.5,
                'pool_origin_y': -5.0,
                'output_z': 0.0,

                # sonar model (matches ping1d.xacro: 10 deg half angle, 0.3..30 m, 1 cm noise)
                'beam_half_angle': 0.1745,
                'min_range': 0.3,
                'max_range': 30.0,
                'sonar_sigma': 0.02,

                # spawn pose in the pool frame
                'init_x': 2.5,
                'init_y': 5.0,
                'init_psi': math.pi / 2,
            }],
        ),
    ])
