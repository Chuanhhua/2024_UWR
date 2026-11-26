#!/usr/bin/env bash
# zellij session for the real 7-sonar robot with the planar EKF.
# Types (without pressing Enter):
#   drivers  a reminder: start the sonar and IMU drivers here (not written yet)
#   kf       the planar EKF with config/ekf_planar/ekf_real_7sonar.yaml
#            (publishes /ekf_planar/odom and TF world -> base_link) and records
#            a bag of the EKF output, IMU, sonars and TF into kf_runs/ekf_real_*
#            (Ctrl+C closes the bag, one bag per launch)
# Start the drivers, then kf.
# Run from a normal terminal (not pixi):
#   scripts/deploy_real_7sonar.sh [session-name]   (default: uwr-real-7sonar)
# Ctrl+q inside the session stops everything cleanly.
SESSION="${1:-uwr-real-7sonar}"
LAYOUT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/zellij/real.kdl"
declare -A PREFILL=(
    [drivers]="# TODO: start the sonar and IMU drivers here"
    [kf]="ros2 launch kalman_filter ekf_planar_real.launch.py"
)
source "$(dirname "${BASH_SOURCE[0]}")/sim_session.sh"
