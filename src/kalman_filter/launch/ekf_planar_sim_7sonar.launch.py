"""Planar EKF for the 7-sonar sim (uwr_gazebo sim_7sonar.launch.py).

Pass the same spawn pose as the sim (world frame), so the EKF starts there:
  ros2 launch kalman_filter ekf_planar_sim_7sonar.launch.py x:=-2.0 y:=0.0 yaw:=0.785

Tank: 5 m x 10 m centred on the world origin, so pool = world + (2.5, 5).
Sonar layout matches uwr_7sonar.urdf.xacro and the unit tests: 0.15 m from
the centre, front/back and four corners at +-45 / +-135 deg. The downward
sonar is not planar and is left out.
"""
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

POOL_W, POOL_L = 5.0, 10.0
H = 0.15

# name, x, y, yaw (body frame), in EKF sonar order
SONARS = [
    ('sonar_front', H, 0.0, 0.0),
    ('sonar_back', -H, 0.0, math.pi),
    ('sonar_front_left', H, H, math.pi / 4),
    ('sonar_front_right', H, -H, -math.pi / 4),
    ('sonar_back_left', -H, H, 3 * math.pi / 4),
    ('sonar_back_right', -H, -H, -3 * math.pi / 4),
]


def generate_launch_description():
    x, y, yaw = (LaunchConfiguration(n) for n in ('x', 'y', 'yaw'))
    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='1.5708'),
        Node(
            package='kalman_filter',
            executable='ekf_planar_node',
            name='ekf_planar',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'imu_topic': '/imu/data',

                'range_topics': [f'/sonar/{s[0]}/range' for s in SONARS],
                'sonar_ox': [s[1] for s in SONARS],
                'sonar_oy': [s[2] for s in SONARS],
                'sonar_alpha': [s[3] for s in SONARS],

                'pool_width': POOL_W,
                'pool_length': POOL_L,
                'pool_origin_x': -POOL_W / 2,
                'pool_origin_y': -POOL_L / 2,
                'output_z': 0.0,

                # sonar model (matches ping1d.xacro: 10 deg half angle, 0.3..30 m, 1 cm noise)
                'beam_half_angle': 0.1745,
                'min_range': 0.3,
                'max_range': 30.0,
                'sonar_sigma': 0.01,

                # spawn pose, world -> pool frame
                'init_x': PythonExpression(['float(', x, ') + ', str(POOL_W / 2)]),
                'init_y': PythonExpression(['float(', y, ') + ', str(POOL_L / 2)]),
                'init_psi': PythonExpression(['float(', yaw, ')']),
            }],
        ),
    ])
