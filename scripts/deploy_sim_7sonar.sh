#!/usr/bin/env bash
# zellij session for the 7-sonar sim with the planar EKF and the zig-zag run.
# Types (without pressing Enter):
#   sim      the Gazebo pool sim: 7-sonar robot (5 m sonars) at the zig-zag start
#   kf       the planar EKF, starting at the same pose
#   debug 1  the zig-zag driver; its legs, speed and disturbances are set in
#            src/uwr_gazebo/config/gazebo_7sonar.yaml (zigzag_driver:)
#   debug 2  pixi run reset-run: robot back to its spawn pose and EKF reset, for a
#            new run without restarting anything (then rerun the driver)
# Start sim, then kf, then the driver.
# Run from a normal terminal (not pixi):
#   scripts/deploy_sim_7sonar.sh [session-name]   (default: uwr-sim-7sonar)
# Ctrl+q inside the session stops everything cleanly.
SESSION="${1:-uwr-sim-7sonar}"
declare -A PREFILL=(
    [sim]="ros2 launch uwr_gazebo sim.launch.py config:=gazebo_7sonar"
    [kf]="ros2 launch kalman_filter ekf_planar_sim.launch.py"
    ["debug 1"]="ros2 run uwr_gazebo zigzag_driver --ros-args -p use_sim_time:=true"
    ["debug 2"]="pixi run reset-run"
)
source "$(dirname "${BASH_SOURCE[0]}")/sim_session.sh"
