#!/usr/bin/env python3
import argparse
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from octa_ros.msg import Labviewdata


class LabviewPublisher(Node):
    def __init__(self, hz: float, toggle_sec: float):
        super().__init__("labview_publisher")
        self.pub = self.create_publisher(
            Labviewdata, "labview_data", QoSProfile(depth=10)
        )
        self.hz = max(1e-3, hz)
        self.toggle_sec = max(0.1, toggle_sec)
        self._scan3d = False
        self._next_toggle = time.monotonic() + self.toggle_sec
        period = 1.0 / self.hz
        self.timer = self.create_timer(period, self._on_timer)

    def _on_timer(self):
        now = time.monotonic()
        if now >= self._next_toggle:
            self._scan3d = not self._scan3d
            self._next_toggle = now + self.toggle_sec
        msg = Labviewdata()
        # Only scan_3d is relevant for Coordinator's scan3d service
        msg.scan_3d = self._scan3d
        self.pub.publish(msg)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument(
        "--toggle",
        type=float,
        default=1.0,
        help="Seconds between scan_3d true/false toggles",
    )
    args = parser.parse_args(argv)

    rclpy.init()
    node = LabviewPublisher(args.hz, args.toggle)

    # Handle SIGINT/SIGTERM gracefully
    def _shutdown(signum, frame):
        node.get_logger().info("Shutting down labview publisher")
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
