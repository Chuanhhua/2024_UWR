#!/usr/bin/env python3
"""Drive the 7-sonar robot through the zig-zag-with-yaw run along the left wall.

Pushes the robot with real forces (ApplyLinkWrench, bridged by the bridge list
in uwr_gazebo config/sim_7sonar.yaml), so the simulated IMU sees the true accelerations. It
follows the ground truth (/pose_topic_gt), so it tests the EKF, not a
controller.

Pool frame: x across the pool from the left wall, y along it from the bottom
wall, heading psi from +x (90 deg = straight up the pool).

  leg "out":  heading 45 deg,  x_in  -> x_out  (away from the left wall)
  yaw +90 deg (towards the wall)
  leg "in":   heading 135 deg, x_out -> x_in   (back towards the wall)
  yaw -90 deg
  ... repeat for `legs` legs, then stop.

Gazebo's persistent wrenches add up, so each tick sends only the change in
wrench; the stack is cleared while the robot is stopped between phases.

Speed and yaw rate follow ramped profiles, and every leg ends in a smooth stop.
Start the robot at (x_in, y_start) facing 45 deg; each leg moves it
x_out - x_in further up the pool (the default 6 legs: 9 m).

Water current (current_x, current_y): the water moves at that velocity, so
the drag force is drag * (current - robot velocity). The driver controls the
robot's speed THROUGH THE WATER, like a real robot with no idea of the
current, so the current carries it sideways and along. Sideways slip is what
the EKF's no-sideslip assumption gets wrong.

Sway noise (sway_noise, sway_noise_tau): a random sideways water velocity in
the robot's frame, added to the current. It wanders smoothly (an
Ornstein-Uhlenbeck process: std sway_noise, correlation time sway_noise_tau),
like turbulence or a tether tugging, and the driver does not correct it.

Parameters (pool frame, metres / seconds / radians):
  x_in, x_out               closest / farthest distance from the left wall
  legs                      number of legs to drive
  speed, accel              forward speed and acceleration along a leg
  yaw_rate, yaw_accel       turn rate and its ramp
  current_x, current_y      water current in the pool frame [m/s]
                            (+x away from the left wall, +y up the pool)
  sway_noise                std of the random sideways water velocity [m/s] (0 = off)
  sway_noise_tau            how long it takes to wander [s]
  seed                      random seed, so a noisy run can be repeated
  mass, drag, yaw_drag      robot model for the feed-forward (match uwr_7sonar.urdf.xacro)
"""
import math
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity, EntityWrench

ENTITY = Entity(name='uwr::base_link', type=Entity.LINK)


def wrap(a):
    return math.remainder(a, 2 * math.pi)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def ramp(prev, target_mag, remaining, accel, dt, sign):
    """Next setpoint: ramp up by accel*dt, never above target_mag, and slow down
    so it reaches zero at remaining = 0 (v^2 = 2 a d)."""
    mag = min(target_mag, abs(prev) + accel * dt, math.sqrt(2 * accel * max(remaining, 0.0)))
    return sign * mag


class ZigZag(Node):
    def __init__(self):
        super().__init__('zigzag_driver')
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.x_in, self.x_out = p('x_in', 1.0), p('x_out', 2.5)
        self.n_legs = p('legs', 6)
        self.speed, self.accel = p('speed', 0.4), p('accel', 0.2)
        self.rate, self.yaw_accel = p('yaw_rate', math.radians(15)), p('yaw_accel', math.radians(30))
        self.current0 = (p('current_x', 0.0), p('current_y', 0.0))
        self.current = self.current0          # current + sway noise, pool frame
        self.sway_std, self.sway_tau = p('sway_noise', 0.0), p('sway_noise_tau', 2.0)
        self.sway = 0.0                       # sideways noise velocity, body frame (+ = left)
        self.rng = random.Random(p('seed', 0))
        self.mass, self.drag, self.yaw_drag = p('mass', 2.9), p('drag', 10.0), p('yaw_drag', 1.0)
        self.kv, self.kw = 20.0, 2.0      # velocity / yaw-rate feedback gains
        self.verbose = p('verbose', False)  # log setpoints, measured velocity and force at 1 Hz
        self.last_print = -1.0

        self.pose = None                  # (t, x, y, psi) in the pool frame
        self.prev = None
        self.vel = (0.0, 0.0, 0.0)        # body (u, v, r), from differencing the truth
        self.phase = 'wait'
        self.heading = math.radians(45)
        self.v = self.w = 0.0             # speed / yaw-rate setpoints
        self.legs = 0
        self.last = None
        self.applied = (0.0, 0.0, 0.0)    # world (fx, fy, tz) currently applied

        self.pub = self.create_publisher(EntityWrench, '/world/pool/wrench/persistent', 10)
        self.clear = self.create_publisher(Entity, '/world/pool/wrench/clear', 10)
        self.create_subscription(Odometry, '/pose_topic_gt', self.on_gt, 10)
        self.create_timer(0.02, self.step)

    def on_gt(self, m):
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
        """Apply a body-frame thrust (fu forward, fv left) and yaw torque, plus
        the push of the water current (drag * current, in the pool frame).

        Persistent wrenches add up in Gazebo, and a clear sent in the same
        step as a new wrench wipes the new one too. So only the change from
        what is already applied is sent; reset_wrench() empties the stack
        while the robot is stopped between phases."""
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
        if self.verbose and self.last - self.last_print >= 1.0:
            self.last_print = self.last
            print(f't={self.last:7.2f} {self.phase}: u_d={u_d:.3f} u={u:.3f} v={v:.3f} '
                  f'r_d={r_d:.3f} r={r:.3f} fu={fu:.2f} fv={fv:.2f} tz={tz:.3f}', flush=True)

    def update_water(self, dt, psi):
        """Advance the sway noise (Ornstein-Uhlenbeck) and combine it with the current."""
        if self.sway_std > 0.0:
            a = math.exp(-dt / self.sway_tau)
            self.sway = a * self.sway + self.sway_std * math.sqrt(1 - a * a) * self.rng.gauss(0.0, 1.0)
        self.current = (self.current0[0] - math.sin(psi) * self.sway,
                        self.current0[1] + math.cos(psi) * self.sway)

    def step(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.02 if self.last is None else min(max(now - self.last, 0.0), 0.1)
        self.last = now
        if self.pose is None or dt == 0.0:
            return
        _, x, y, psi = self.pose
        self.update_water(dt, psi)

        if self.phase == 'wait':
            self.get_logger().info(f'start at pool ({x:.2f}, {y:.2f}), heading {math.degrees(psi):.0f} deg')
            self.heading = math.radians(45) if math.cos(psi) > 0 else math.radians(135)
            self.phase = 'leg'

        if self.phase == 'leg':
            out = math.cos(self.heading) > 0          # moving away from the left wall
            remaining = ((self.x_out - x) if out else (x - self.x_in)) / abs(math.cos(self.heading))
            v_new = ramp(self.v, self.speed, remaining, self.accel, dt, 1.0)
            a_d = (v_new - self.v) / dt
            self.v = v_new
            r_d = max(-self.rate, min(self.rate, 1.5 * wrap(self.heading - psi)))  # hold the heading
            self.control(self.v, a_d, 0.0, r_d)
            if remaining <= 0.005 or (self.v < 1e-3 and remaining < 0.05):
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
            self.w = ramp(self.w, self.rate, abs(err), self.yaw_accel, dt, math.copysign(1.0, err))
            self.control(0.0, 0.0, 0.0, self.w)
            if abs(err) < math.radians(0.5):
                self.w = 0.0
                self.reset_wrench()
                self.heading = wrap(self.target)
                self.phase = 'leg'


def main():
    rclpy.init()
    node = ZigZag()
    try:
        while rclpy.ok() and node.phase != 'done':
            rclpy.spin_once(node)
        if rclpy.ok():
            for _ in range(5):       # remove the force so the robot drifts to a stop
                node.clear.publish(ENTITY)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
