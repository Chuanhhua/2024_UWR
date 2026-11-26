#!/usr/bin/env bash
# zellij session for the 3-sonar sim (surge / sway / heave) with kf_3dof.
# Types (without pressing Enter):
#   sim  the Gazebo pool sim: robot, bridge, scan_to_three_sonar, foxglove bridge
#   kf   kf_3dof on sim time plus kf_to_pool; bag goes to kf_runs/sim_*
# Run from a normal terminal (not pixi):
#   scripts/deploy_sim_3sonar.sh [session-name]   (default: uwr-sim-3sonar)
# Ctrl+q inside the session stops everything cleanly.
SESSION="${1:-uwr-sim-3sonar}"
declare -A PREFILL=(
    [sim]="ros2 launch uwr_gazebo sim.launch.py config:=gazebo_3sonar"
    [kf]='ros2 launch kalman_filter kf_3dof_sim_3sonar.launch.py csv_path:=$PIXI_PROJECT_ROOT/kf_runs/sim_'
)
source "$(dirname "${BASH_SOURCE[0]}")/sim_session.sh"
