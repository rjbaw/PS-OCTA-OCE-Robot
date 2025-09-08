#!/usr/bin/env python3
import signal

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from octa_ros.srv import Scan3d


class Scan3dService(Node):
    def __init__(self):
        super().__init__("scan3d_server")
        qos = QoSProfile(depth=10)
        self.srv = self.create_service(
            Scan3d, "scan_3d", self._on_request, qos_profile=qos
        )

    def _on_request(self, request, response):
        response.success = True
        return response


def main():
    rclpy.init()
    node = Scan3dService()

    def _shutdown(signum, frame):
        node.get_logger().info("Shutting down scan3d server")
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
