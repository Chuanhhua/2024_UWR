"""Gazebo tank with the 7-sonar robot (uwr_7sonar.urdf.xacro).

Same as sim.launch.py, except: the robot description, a bridge for all seven
sonars, range_adapter (one sensor_msgs/Range per sonar) instead of
sonar_adapter, and a configurable spawn pose.

  ros2 launch uwr_gazebo sim_7sonar.launch.py x:=-2.0 y:=0.0 yaw:=0.785
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = PathJoinSubstitution([FindPackageShare('uwr_gazebo'), 'worlds', 'tank.sdf'])
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('uwr_description'), 'urdf', 'uwr_7sonar.urdf.xacro'])
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    bridge_config = PathJoinSubstitution(
        [FindPackageShare('uwr_gazebo'), 'config', 'bridge_7sonar.yaml'])

    # spawn pose in the world frame (tank centre = origin, +y down the length)
    args = [
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='1.5708'),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': ['-r ', world]}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'uwr', '-z', '0',
                            '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                            '-Y', LaunchConfiguration('yaw')])

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  parameters=[{'config_file': bridge_config, 'use_sim_time': True}])

    adapter = Node(package='uwr_gazebo', executable='range_adapter', output='screen',
                   parameters=[{'use_sim_time': True}])

    foxglove = Node(package='foxglove_bridge', executable='foxglove_bridge', output='screen',
                    parameters=[{'use_sim_time': True}])

    return LaunchDescription(args + [gazebo, rsp, spawn, bridge, adapter, foxglove])
