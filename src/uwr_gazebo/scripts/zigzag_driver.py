#!/usr/bin/env python3
"""Drive the 7-sonar robot through the zig-zag-with-yaw run along the left wall.

Pushes the robot with real forces (ApplyLinkWrench, bridged by the bridge list
in uwr_gazebo config/gazebo_7sonar.yaml), so the simulated IMU sees the true accelerations. It
follows the ground truth (/pose_topic_gt), so it tests the EKF, not a
controller.

Pool frame: x across the pool from the left wall, y along it from the bottom
wall, heading psi from +x (90 deg = straight up the pool).

  leg "out":  heading 45 deg,  for leg_time  (away from the left wall)
  yaw +90 deg (towards the wall)
  leg "in":   heading 135 deg, for leg_time  (back towards the wall)
  yaw -90 deg
  ... repeat for `legs` legs, then stop.

Legs are timed, not driven to a waypoint, like the real robot driving on its
IMU heading without knowing its position: each leg ramps up at accel, cruises
at speed and ramps down to stop exactly at leg_time, all through the water. So
the current changes where a leg ends, not how long it takes. The heading is
held, and turns end, on the true heading (the real robot's IMU gives it that).
The only use of the true position is a safety stop: the run ends if the robot
gets within wall_margin of a wall.

Gazebo's persistent wrenches add up, so each tick sends only the change in
wrench; the stack is cleared while the robot is stopped between phases.

Speed and yaw rate follow ramped profiles, and every leg ends in a smooth stop.
Start the robot near the left wall facing 45 deg (the sim config's spawn); in
still water a leg covers speed * (leg_time - speed / accel) (default 2.25 m,
1.6 m across and up the pool), so the default 6 legs go ~9.5 m up the pool.

Water current: the water moves at a velocity in the pool frame, so the drag
force is drag * (current - robot velocity). The driver controls the robot's
speed THROUGH THE WATER, like a real robot with no idea of the current, so the
current carries it sideways and along. Sideways slip is what the EKF's
no-sideslip assumption gets wrong. The current is random with zero mean, so it
goes both ways: each pool axis wanders on its own (an Ornstein-Uhlenbeck
process: std current_noise, correlation time current_noise_tau, much slower
than the sway noise).

Sway noise (sway_noise, sway_noise_tau): a random sideways water velocity in
the robot's frame, added to the current. It wanders smoothly (an
Ornstein-Uhlenbeck process: std sway_noise, correlation time sway_noise_tau),
like turbulence or a tether tugging, and the driver does not correct it.

Settings: the zigzag_driver: section of uwr_gazebo config/gazebo_7sonar.yaml
(legs, leg_time, speed, the disturbances and their seed). The driver has no
ROS parameters of its own (only use_sim_time); edit the yaml to change a run.
That section also holds the motion profile, the safety stop, the pool size and
the robot model for the feed-forward (keep those matched to the world and URDF).
"""
import math
import os
import random
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity, EntityWrench
import yaml
from ament_index_python.packages import get_package_share_directory

ENTITY = Entity(name='uwr::base_link', type=Entity.LINK)
CONFIG = 'gazebo_7sonar'          # uwr_gazebo/config/<CONFIG>.yaml, section zigzag_driver:


def wrap(a):
    """Wrap an angle to [-pi, pi]."""
    return math.remainder(a, 2 * math.pi)


def yaw_of(q):
    """Yaw [rad] of a quaternion."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def ramp(prev, target_mag, remaining, accel, dt, sign):
    """Next setpoint: ramp up by accel*dt, capped at target_mag, slowing to 0 at remaining = 0."""
    mag = min(target_mag, abs(prev) + accel * dt, math.sqrt(2 * accel * max(remaining, 0.0)))
    return sign * mag


class ZigZag(Node):
    """Drives the zig-zag run with forces on the simulated robot."""

    def __init__(self):
        """Read the zigzag_driver: settings and set up the publishers, truth input and timer."""
        super().__init__('zigzag_driver')
        with open(os.path.join(get_package_share_directory('uwr_gazebo'), 'config', CONFIG + '.yaml')) as f:
            cfg = yaml.safe_load(f)['zigzag_driver']

        # --- settings, from the yaml's zigzag_driver: section ---
        self.n_legs = int(cfg['legs'])
        self.leg_time = float(cfg['leg_time'])
        self.speed = float(cfg['speed'])
        self.accel = float(cfg['accel'])
        self.yaw_rate = math.radians(float(cfg['yaw_rate']))
        self.yaw_accel = math.radians(float(cfg['yaw_accel']))
        self.wall_margin = float(cfg['wall_margin'])
        self.pool = (float(cfg['pool_width']), float(cfg['pool_length']))
        self.mass, self.drag, self.yaw_drag = float(cfg['mass']), float(cfg['drag']), float(cfg['yaw_drag'])
        self.kv, self.kw = float(cfg['kv']), float(cfg['kw'])
        self.cur_std, self.cur_tau = float(cfg['current_noise']), float(cfg['current_noise_tau'])
        self.sway_std, self.sway_tau = float(cfg['sway_noise']), float(cfg['sway_noise_tau'])
        self.rng = random.Random(int(cfg['seed']))   # for both noises, seeded from the yaml

        # --- run state, not from the yaml: starts here and changes during the run ---
        self.cur_noise = [0.0, 0.0]       # random current, pool frame
        self.sway = 0.0                   # sideways noise velocity, body frame (+ = left)
        self.current = (0.0, 0.0)         # current + sway noise, pool frame
        self.pose = None                  # (t, x, y, psi) in the pool frame
        self.prev = None
        self.vel = (0.0, 0.0, 0.0)        # body (u, v, r), from differencing the truth
        self.phase = 'wait'
        self.heading = math.radians(45)
        self.v = self.w = 0.0             # speed / yaw-rate setpoints
        self.legs = 0
        self.leg_start = None             # time the current leg started [s]
        self.last = None
        self.applied = (0.0, 0.0, 0.0)    # world (fx, fy, tz) currently applied

        # --- ROS interfaces ---
        self.pub = self.create_publisher(EntityWrench, '/world/pool/wrench/persistent', 10)
        self.clear = self.create_publisher(Entity, '/world/pool/wrench/clear', 10)
        self.create_subscription(Odometry, '/pose_topic_gt', self.on_gt, 10)
        self.create_timer(0.02, self.step)

    def on_gt(self, m):
        """Store the true pose and a smoothed body velocity from the ground truth."""
        q = m.pose.pose
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        cur = (t, q.position.x, q.position.y, yaw_of(q.orientation))   # world = pool frame
        if self.prev is not None and cur[0] > self.prev[0]:
            dt = cur[0] - self.prev[0]
            vx, vy = (cur[1] - self.prev[1]) / dt, (cur[2] - self.prev[2]) / dt
            c, s = math.cos(cur[3]), math.sin(cur[3])
            new = (c * vx + s * vy, -s * vx + c * vy, wrap(cur[3] - self.prev[3]) / dt)
            a = 0.5  # light smoothing
            self.vel = tuple(a * n + (1 - a) * o for n, o in zip(new, self.vel))
        self.prev = cur
        self.pose = cur

    def push(self, fu, fv, tz):
        """Apply a body-frame thrust (fu forward, fv left), yaw torque and the current's push."""
        # Persistent wrenches add up in Gazebo, and a clear sent in the same
        # step as a new wrench wipes the new one too. So only the change from
        # what is already applied is sent; reset_wrench() empties the stack
        # while the robot is stopped between phases.
        psi = self.pose[3]
        c, s = math.cos(psi), math.sin(psi)
        want = (c * fu - s * fv + self.drag * self.current[0],
                s * fu + c * fv + self.drag * self.current[1], tz)
        d = tuple(w - a for w, a in zip(want, self.applied))
        if max(abs(v) for v in d) < 1e-3:
            return
        m = EntityWrench(entity=ENTITY)
        m.wrench.force = Vector3(x=d[0], y=d[1], z=0.0)
        m.wrench.torque = Vector3(x=0.0, y=0.0, z=d[2])
        self.pub.publish(m)
        self.applied = want

    def reset_wrench(self):
        """Remove every persistent wrench; the next push() sends the full wrench."""
        self.clear.publish(ENTITY)
        self.applied = (0.0, 0.0, 0.0)

    def control(self, u_d, a_d, v_d, r_d):
        """Thrust to hit body velocity (u_d, v_d) through the water and yaw rate r_d."""
        u, v, r = self.vel
        psi = self.pose[3]
        c, s = math.cos(psi), math.sin(psi)
        u -= c * self.current[0] + s * self.current[1]      # velocity relative to the water
        v -= -s * self.current[0] + c * self.current[1]
        fu = self.drag * u_d + self.mass * a_d + self.kv * (u_d - u)
        fv = self.drag * v_d + self.kv * (v_d - v)
        tz = self.yaw_drag * r_d + self.kw * (r_d - r)
        self.push(fu, fv, tz)

    def update_water(self, dt, psi):
        """Advance the random current and sway noise (zero-mean Ornstein-Uhlenbeck) and add them up."""
        if self.cur_std > 0.0:
            a = math.exp(-dt / self.cur_tau)
            for i in range(2):
                self.cur_noise[i] = (a * self.cur_noise[i]
                                     + self.cur_std * math.sqrt(1 - a * a) * self.rng.gauss(0.0, 1.0))
        if self.sway_std > 0.0:
            a = math.exp(-dt / self.sway_tau)
            self.sway = a * self.sway + self.sway_std * math.sqrt(1 - a * a) * self.rng.gauss(0.0, 1.0)
        self.current = (self.cur_noise[0] - math.sin(psi) * self.sway,
                        self.cur_noise[1] + math.cos(psi) * self.sway)

    def step(self):
        """One 50 Hz tick: update the water, check the walls and run the leg / turn phase."""
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.02 if self.last is None else min(max(now - self.last, 0.0), 0.1)
        self.last = now
        if self.pose is None or dt == 0.0:
            return
        _, x, y, psi = self.pose
        self.update_water(dt, psi)

        if self.phase == 'wait':
            self.get_logger().info(
                f'start at pool ({x:.2f}, {y:.2f}), heading {math.degrees(psi):.0f} deg; '
                f'{self.n_legs} legs of {self.leg_time} s at {self.speed} m/s; '
                f'current +- {self.cur_std} m/s (tau {self.cur_tau} s), sway noise {self.sway_std} m/s')
            self.heading = math.radians(45) if math.cos(psi) > 0 else math.radians(135)
            self.phase = 'leg'
            self.leg_start = now

        # safety stop: the only use of the true position
        gap = min(x, self.pool[0] - x, y, self.pool[1] - y)
        if gap < self.wall_margin:
            self.get_logger().error(f'{gap:.2f} m from a wall at pool ({x:.2f}, {y:.2f}), stopping')
            self.phase = 'done'
            return

        if self.phase == 'leg':
            out = math.cos(self.heading) > 0          # moving away from the left wall
            elapsed = now - self.leg_start
            # trapezoid in time: ramp up, cruise, ramp down to 0 at leg_time
            v_new = max(0.0, min(self.speed, self.v + self.accel * dt, self.accel * (self.leg_time - elapsed)))
            a_d = (v_new - self.v) / dt
            self.v = v_new
            r_d = max(-self.yaw_rate, min(self.yaw_rate, 1.5 * wrap(self.heading - psi)))  # hold the heading
            self.control(self.v, a_d, 0.0, r_d)
            if elapsed >= self.leg_time:
                self.legs += 1
                self.v = 0.0
                self.reset_wrench()
                self.get_logger().info(f'leg {self.legs} done at pool ({x:.2f}, {y:.2f})')
                if self.legs >= self.n_legs:
                    self.phase = 'done'
                    self.get_logger().info(f'{self.legs} legs done, stopping')
                    return
                self.target = self.heading + (math.radians(90) if out else -math.radians(90))
                self.phase = 'turn'

        elif self.phase == 'turn':
            err = wrap(self.target - psi)
            self.w = ramp(self.w, self.yaw_rate, abs(err), self.yaw_accel, dt, math.copysign(1.0, err))
            self.control(0.0, 0.0, 0.0, self.w)
            if abs(err) < math.radians(0.5):
                self.w = 0.0
                self.reset_wrench()
                self.heading = wrap(self.target)
                self.phase = 'leg'
                self.leg_start = now


def main():
    """Run the driver until the run is done or Ctrl+C, then remove all forces."""
    # Handle Ctrl+C ourselves: rclpy's own handler shuts ROS down first, and then
    # the forces can no longer be cleared, leaving the robot pushed.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ZigZag()
    try:
        while rclpy.ok() and node.phase != 'done':
            rclpy.spin_once(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('stopped early')
    finally:
        if rclpy.ok():
            for _ in range(5):       # remove all forces so the robot drifts to a stop
                node.clear.publish(ENTITY)
            time.sleep(0.2)          # let the clears go out before shutting down
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
