#!/usr/bin/env python3
"""Turn each simulated sonar LaserScan into a sensor_msgs/Range.

/sonar/<name> (LaserScan, a small grid of rays over the beam cone)
  -> /sonar/<name>/range (Range, the nearest echo, like a real Ping1D)

When nothing is hit, range is +inf (the Range convention for "no detection").

Bad readings, like a real sonar's (rates from the noise: section of the sim
config, passed in by sim.launch.py; each reading independently):
  sonar_dropout   probability a reading is lost: no echo, published as +inf
  sonar_outlier   probability a reading is a spurious echo: a random range,
                  uniform between the sonar's min and max range
  sonar_seed      random seed, so a run's bad readings can be repeated
"""
import math
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range

DEFAULT_SONARS = ['sonar_front', 'sonar_back', 'sonar_front_left', 'sonar_front_right',
                  'sonar_back_left', 'sonar_back_right', 'sonar_down']


class ScanToRange(Node):
    """Publishes one Range per simulated sonar, with dropouts and outliers mixed in."""

    def __init__(self):
        """Read the sonar list and bad-reading rates and connect every sonar."""
        super().__init__('scan_to_range')
        names = self.declare_parameter('sonars', DEFAULT_SONARS).value
        self.dropout = self.declare_parameter('sonar_dropout', 0.0).value
        self.outlier = self.declare_parameter('sonar_outlier', 0.0).value
        self.rng = random.Random(self.declare_parameter('sonar_seed', 0).value)
        self.get_logger().info(f'sonar dropout {self.dropout:.3f}, outlier {self.outlier:.3f}')
        self.pubs = {}
        for name in names:
            self.pubs[name] = self.create_publisher(Range, f'/sonar/{name}/range', 10)
            self.create_subscription(LaserScan, f'/sonar/{name}',
                                     lambda msg, name=name: self.on_scan(name, msg), 10)

    def on_scan(self, name, scan):
        """Publish the nearest echo of one scan, or a dropout / outlier in its place."""
        hits = [r for r in scan.ranges                                   # the rays across the beam cone
                if math.isfinite(r) and scan.range_min <= r <= scan.range_max]   # keep only rays that hit something in range
        out = Range()
        out.header = scan.header                                         # same time stamp and sonar frame as the scan
        out.radiation_type = Range.ULTRASOUND
        out.field_of_view = scan.angle_max - scan.angle_min              # full cone width (the scan spans both sides of the beam axis)
        out.min_range = scan.range_min
        out.max_range = scan.range_max
        out.range = min(hits) if hits else math.inf                      # true reading: the nearest echo, or +inf if no ray hit
        u = self.rng.random()                                            # one random number in [0, 1) per reading decides its fate:
        if u < self.dropout:                                             #   u in [0, dropout): dropout
            out.range = math.inf                                         #     no echo; the EKF skips it
        elif u < self.dropout + self.outlier:                            #   u in [dropout, dropout + outlier): outlier
            out.range = self.rng.uniform(scan.range_min, scan.range_max) #     true range replaced by a random one anywhere in min..max
                                                                         #   otherwise the true reading is kept
        self.pubs[name].publish(out)


def main():
    """Run the converter until Ctrl+C or launch shutdown."""
    rclpy.init()
    try:
        rclpy.spin(ScanToRange())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                        # Ctrl+C / launch shutdown
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
