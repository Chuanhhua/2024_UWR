from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('uwr_description'), 'urdf', 'uwr_3sonar.urdf.xacro'])
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('uwr_description'), 'rviz', 'uwr.rviz'])

    # run xacro at launch time and pass the resulting URDF text as a parameter
    # The model needs its sensor args, which the sim takes from uwr_gazebo's config.
    # RViz only draws the robot, so any values do here.
    viewer_args = ' sonar_max_range:=30.0 sonar_noise_std:=0.0 imu_gyro_std:=0.0 imu_accel_std:=0.0'
    robot_description = ParameterValue(Command(['xacro ', xacro_file, viewer_args]), value_type=str)

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config]),
    ])
