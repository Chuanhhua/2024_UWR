#!/usr/bin/env python3
"""Start a new sim run without restarting anything: put the robot back at its
spawn pose and reset the planar EKF to its start pose.

  pixi run reset-run               # config sim_7sonar
  pixi run reset-run sim_3sonar

Then start the driver again (in its pane: Up arrow, Enter). Gazebo, the
bridge, the EKF node and Foxglove keep running; Foxglove's trails restart
by themselves when the pose jumps.

Steps: remove all forces on the robot (wait for it to coast to a stop),
teleport it to `spawn` from uwr_gazebo config/<config>.yaml, then call the
EKF's ~/reset service (skipped with a note if the EKF isn't running).
Stop the zig-zag driver first if it is still mid-run, or it keeps pushing.
"""
import math
import os
import subprocess
import sys
import time

import yaml
from ament_index_python.packages import get_package_share_directory

WORLD = 'pool'
ROBOT = 'uwr'


def run(cmd, timeout=10):
    """Run a command; return (ok, combined output)."""
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.returncode == 0, (r.stdout + r.stderr).strip()
    except subprocess.TimeoutExpired:
        return False, f'timed out after {timeout} s'


def main():
    config = sys.argv[1] if len(sys.argv) > 1 else 'sim_7sonar'
    with open(os.path.join(get_package_share_directory('uwr_gazebo'), 'config', config + '.yaml')) as f:
        spawn = yaml.safe_load(f)['spawn']
    x, y, yaw = float(spawn['x']), float(spawn['y']), float(spawn['yaw'])

    # 1. no forces left from the driver, then let the water drag stop the robot
    ok, out = run(['gz', 'topic', '-t', f'/world/{WORLD}/wrench/clear', '-m', 'gz.msgs.Entity',
                   '-p', f'name: "{ROBOT}::base_link", type: LINK'])
    if not ok:
        sys.exit(f'could not clear the forces (is the sim running?): {out}')
    time.sleep(1.5)

    # 2. teleport to the spawn pose (pool frame = world frame)
    pose = (f'name: "{ROBOT}", position: {{x: {x}, y: {y}, z: 0.0}}, '
            f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
    ok, out = run(['gz', 'service', '-s', f'/world/{WORLD}/set_pose', '--reqtype', 'gz.msgs.Pose',
                   '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', pose])
    if not ok or 'true' not in out:
        sys.exit(f'could not move the robot: {out}')
    print(f'robot back at pool ({x}, {y}), heading {math.degrees(yaw):.0f} deg')

    # 3. EKF back to its start pose (the IMU keeps it predicting from there)
    time.sleep(0.5)
    ok, out = run(['ros2', 'service', 'call', '/ekf_planar/reset', 'std_srvs/srv/Trigger'], timeout=15)
    if ok and 'success=True' in out:
        print('EKF reset to its start pose')
    else:
        print('EKF not reset (not running?): start it as usual')


if __name__ == '__main__':
    main()
