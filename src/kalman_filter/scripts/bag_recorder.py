#!/usr/bin/env python3
"""Record a bag from launch to shutdown. Shared by the launch files, so the
topic list for a setup lives in its launch file and the recording itself lives
here. One bag per launch; Ctrl+C on the launch closes it cleanly.

Runs `ros2 bag record` (mcap) into <out_dir>/<prefix>_<date>/.

Parameters:
  topics   topics to record (string array)
  regex    also record topics matching this (e.g. "/sonar/.*"); with neither, all topics
  prefix   bag name prefix (default "run")
  out_dir  folder for the bags (default $PIXI_PROJECT_ROOT/kf_runs, else ./kf_runs)
"""
import os
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter


class BagRecorder(Node):
    """Records one bag with `ros2 bag record` from startup until stop()."""

    def __init__(self):
        """Read the parameters and start `ros2 bag record` into a new bag folder."""
        super().__init__('bag_recorder')
        topics = self.declare_parameter('topics', Parameter.Type.STRING_ARRAY).value or []
        regex = self.declare_parameter('regex', '').value
        prefix = self.declare_parameter('prefix', 'run').value
        root = os.environ.get('PIXI_PROJECT_ROOT', os.getcwd())
        out_dir = self.declare_parameter('out_dir', os.path.join(root, 'kf_runs')).value

        os.makedirs(out_dir, exist_ok=True)
        self.path = os.path.join(out_dir, time.strftime(f'{prefix}_%Y_%m_%d_%H_%M_%S'))
        cmd = ['ros2', 'bag', 'record', '-s', 'mcap', '-o', self.path]
        if topics:
            cmd += ['--topics', *topics]
        if regex:
            cmd += ['--regex', regex]
        if not topics and not regex:
            cmd += ['--all']
        # its own process group: Ctrl+C reaches only this node, which then closes the bag
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, start_new_session=True)
        threading.Thread(target=self._watch, daemon=True).start()
        self.get_logger().info(f'recording to {self.path}')

    def _watch(self):
        """Pass the recorder's errors on to this node's log."""
        for line in self.proc.stdout:
            if 'ERROR' in line:
                self.get_logger().error(f'bag: {line.strip()}')

    def stop(self):
        """Close the bag: SIGINT the recorder and wait for it (kill it after 10 s)."""
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.get_logger().info(f'bag saved: {self.path}')


def main():
    """Record until Ctrl+C or shutdown, then close the bag cleanly."""
    rclpy.init()
    node = BagRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl+C in a terminal arrives twice (terminal + launch): don't let the
        # second one interrupt closing the bag
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
