#!/usr/bin/env python3
"""Turn each simulated sonar LaserScan into a sensor_msgs/Range.

/sonar/<name> (LaserScan, a small grid of rays over the beam cone)
  -> /sonar/<name>/range (Range, the nearest echo, like a real Ping1D)

When nothing is hit, range is +inf (the Range convention for "no detection").
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range

DEFAULT_SONARS = ['sonar_front', 'sonar_back', 'sonar_front_left', 'sonar_front_right',
                  'sonar_back_left', 'sonar_back_right', 'sonar_down']


class RangeAdapter(Node):
    def __init__(self):
        super().__init__('range_adapter')
        names = self.declare_parameter('sonars', DEFAULT_SONARS).value
        self.pubs = {}
        for name in names:
            self.pubs[name] = self.create_publisher(Range, f'/sonar/{name}/range', 10)
            self.create_subscription(LaserScan, f'/sonar/{name}',
                                     lambda msg, name=name: self.on_scan(name, msg), 10)

    def on_scan(self, name, scan):
        hits = [r for r in scan.ranges
                if math.isfinite(r) and scan.range_min <= r <= scan.range_max]
        out = Range()
        out.header = scan.header
        out.radiation_type = Range.ULTRASOUND
        # the scan spans the cone in both directions; its half width is the beam half angle
        out.field_of_view = scan.angle_max - scan.angle_min
        out.min_range = scan.range_min
        out.max_range = scan.range_max
        out.range = min(hits) if hits else math.inf
        self.pubs[name].publish(out)


def main():
    rclpy.init()
    rclpy.spin(RangeAdapter())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
