"""Gazebo pool (15 x 25 m) with a UWR robot. Which robot, its sonar range and
converter, sensor noise, where it spawns and the ROS-Gazebo bridge topics come
from a config in uwr_gazebo/config/, picked by name (without .yaml):

  ros2 launch uwr_gazebo sim.launch.py config:=sim_7sonar   # 7-sonar robot, zig-zag start
  ros2 launch uwr_gazebo sim.launch.py config:=sim_3sonar   # 3-sonar robot, pool centre

  gui:=false   run Gazebo without its window (the sonars still work), for
               batch runs or a slow machine; Foxglove still shows everything

Default: sim_7sonar. Start the matching filter with the same name; it reads
its start pose and sonar range from the same config:
  ros2 launch kalman_filter ekf_planar_sim.launch.py config:=sim_7sonar

Starts: Gazebo with the pool, robot_state_publisher, the robot spawn, the
ROS-Gazebo bridge, the sonar converter and foxglove_bridge (ws://localhost:8765).
"""
import math
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



def make_sim(context):
    # The config decides which files and programs to start, so it is read here
    # in Python (a missing name fails with "No such file").
    gazebo_share = get_package_share_directory('uwr_gazebo')
    name = LaunchConfiguration('config').perform(context)
    with open(os.path.join(gazebo_share, 'config', name + '.yaml')) as f:
        cfg = yaml.safe_load(f)

    xacro_file = os.path.join(get_package_share_directory('uwr_description'), 'urdf', cfg['robot'])
    noise = cfg['noise']
    xacro_cmd = (f"xacro {xacro_file} sonar_max_range:={cfg['sonar_max_range']}"
                 f" sonar_noise_std:={noise['sonar_std']}"
                 f" imu_gyro_std:={noise['imu_gyro_std']} imu_accel_std:={noise['imu_accel_std']}")
    robot_description = ParameterValue(Command(xacro_cmd), value_type=str)

    spawn = cfg['spawn']

    # Gazebo itself, via ros_gz_sim's ready-made launch file; -s = server only (no window)
    gui = LaunchConfiguration('gui').perform(context).lower() in ('true', '1')
    gz_flags = '-r ' if gui else '-r -s '
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_flags + os.path.join(gazebo_share, 'worlds', 'pool.sdf')}.items())

    # publishes the URDF on /robot_description (and TF)
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    # reads /robot_description and creates the robot inside Gazebo
    spawner = Node(package='ros_gz_sim', executable='create', output='screen',
                   arguments=['-topic', 'robot_description', '-name', 'uwr', '-z', '0',
                              '-x', str(spawn['x']), '-y', str(spawn['y']), '-Y', str(spawn['yaw'])])

    # the bridge between ROS 2 and Gazebo; parameter_bridge only takes a file,
    # so the config's bridge list is written to a temporary one
    # (one fixed file per config, overwritten each launch, so they don't pile up)
    bridge_file = os.path.join(tempfile.gettempdir(), f'uwr_bridge_{name}.yaml')
    with open(bridge_file, 'w') as f:
        yaml.safe_dump(cfg['bridge'], f, sort_keys=False)
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  parameters=[{'config_file': bridge_file, 'use_sim_time': True}])

    # turns the simulated sonar scans into what the filter reads
    converter = Node(package='uwr_gazebo', executable=cfg['sonar_converter'], output='screen',
                     parameters=[{'use_sim_time': True}])

    # live connection for Foxglove: ws://localhost:8765
    foxglove = Node(package='foxglove_bridge', executable='foxglove_bridge', output='screen',
                    parameters=[{'use_sim_time': True}])

    print(f"[sim.launch.py] {name}: {cfg['robot']} at pool ({spawn['x']}, {spawn['y']}), "
          f"heading {math.degrees(spawn['yaw']):.0f} deg, sonars {cfg['sonar_max_range']} m, "
          f"{len(cfg['bridge'])} bridged topics")
    return [gazebo, rsp, spawner, bridge, converter, foxglove]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='false: run Gazebo without its window (much less CPU)'),
        DeclareLaunchArgument('config', default_value='sim_7sonar',
                              description='sim config name in uwr_gazebo/config/ '
                                          '(without .yaml), e.g. sim_7sonar or sim_3sonar'),
        OpaqueFunction(function=make_sim),
    ])
