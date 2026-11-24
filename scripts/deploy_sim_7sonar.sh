#!/usr/bin/env bash
# zellij session for the 7-sonar sim with the planar EKF and the zig-zag run.
# Types (without pressing Enter):
#   sim      the Gazebo pool sim: 7-sonar robot (5 m sonars) at the zig-zag start
#   kf       the planar EKF, starting at the same pose
#   debug 1  the 6-leg zig-zag driver; add disturbances to its --ros-args, e.g.
#            -p sway_noise:=0.01 (random sideways slip, m/s)
#            -p current_y:=0.02  (current up the pool, m/s)
# Start sim, then kf, then the driver.
# Run from a normal terminal (not pixi):
#   scripts/deploy_sim_7sonar.sh [session-name]   (default: uwr-sim-7sonar)
# Ctrl+q inside the session stops everything cleanly.
SESSION="${1:-uwr-sim-7sonar}"
declare -A PREFILL=(
    [sim]="ros2 launch uwr_gazebo sim.launch.py config:=sim_7sonar"
    [kf]="ros2 launch kalman_filter ekf_planar_sim.launch.py config:=sim_7sonar"
    ["debug 1"]="ros2 run uwr_gazebo zigzag_driver --ros-args -p use_sim_time:=true"
)
source "$(dirname "${BASH_SOURCE[0]}")/sim_session.sh"
