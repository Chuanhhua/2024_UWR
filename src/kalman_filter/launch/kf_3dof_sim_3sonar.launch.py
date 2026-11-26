"""The old 3-DOF Kalman filter (kalman_filter kf_3dof) on the 3-sonar sim.

Starts only the filter side; run the sim first (uwr_gazebo sim.launch.py config:=gazebo_3sonar):
  - kf_3dof (kalman_filter launch_3dof.py) on sim time, reading /imu/data and /sbs,
    writing its CSV to csv_path
  - kf_to_pool: turns its wall distances into a pool pose on /kf/pool_pose (+ TF)

  ros2 launch kalman_filter kf_3dof_sim_3sonar.launch.py csv_path:=$PIXI_PROJECT_ROOT/kf_runs/sim_
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
  
    kf_to_pool_node = Node(package='kalman_filter', executable='kf_to_pool', output='screen')
    
    return LaunchDescription([
        # applies to every node launched after it, including ones in included files
        SetParameter(name='use_sim_time', value=True),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('kalman_filter'), 'launch', 'launch_3dof.py']))),
        kf_to_pool_node
    ])
