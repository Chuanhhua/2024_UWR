#!/usr/bin/env bash
# zellij session for the real 7-sonar robot with the planar EKF.
# Types (without pressing Enter):
#   drivers  a reminder: start the sonar and IMU drivers here (not written yet)
#   kf       the planar EKF with config/ekf_planar/real_7sonar.yaml
#            (publishes /ekf_planar/odom and TF world -> base_link)
#   record   a bag of the EKF output, IMU, sonars and TF into kf_runs/real_*
# Start the drivers, then kf, then record.
# Run from a normal terminal (not pixi):
#   scripts/deploy_real_7sonar.sh [session-name]   (default: uwr-real-7sonar)
# Ctrl+q inside the session stops everything cleanly.
SESSION="${1:-uwr-real-7sonar}"
LAYOUT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/zellij/real.kdl"
declare -A PREFILL=(
    [drivers]="# TODO: start the sonar and IMU drivers here"
    [kf]="ros2 launch kalman_filter ekf_planar_real.launch.py config:=real_7sonar"
    [record]='ros2 bag record -s mcap -o $PIXI_PROJECT_ROOT/kf_runs/real_$(date +%Y_%m_%d_%H_%M_%S) --topics /ekf_planar/odom /ekf_planar/sonar_diag /imu/data /tf --regex "/sonar/.*"'
)
source "$(dirname "${BASH_SOURCE[0]}")/sim_session.sh"
