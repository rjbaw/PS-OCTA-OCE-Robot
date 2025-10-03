#!/usr/bin/env python3
import argparse
import os
import signal
import socket
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from ament_index_python.packages import get_package_share_directory

def _ping(host: str, timeout_s: float = 1.0) -> bool:
    try:
        rc = subprocess.run(
            ["ping", "-c", "1", "-W", str(int(max(1, round(timeout_s)))), host],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        ).returncode
        if rc == 0:
            return True
    except Exception:
        pass
    try:
        # TCP connect to Universal Robots dashboard port (29999)
        with socket.create_connection((host, 29999), timeout=timeout_s):
            return True
    except Exception:
        return False


def _detect_host_ip(robot_ip: str) -> str:
    try:
        out = subprocess.check_output(["ip", "route", "get", robot_ip], text=True)
        parts = out.split()
        for i, p in enumerate(parts):
            if p == "src" and i + 1 < len(parts):
                return parts[i + 1]
    except Exception:
        pass
    return os.environ.get("HOST_IP", "192.168.0.2")


class DriverManager(Node):
    def __init__(self, robot_ip: str, host_ip: str, ur_type: str, headless: bool, ping_timeout: float):
        super().__init__("driver_manager")
        self.robot_ip = robot_ip
        self.host_ip = host_ip or _detect_host_ip(robot_ip)
        self.ur_type = ur_type
        self.headless = headless
        self.ping_timeout = ping_timeout

        self.state_run_false = False  
        self.state_last_msg_time = 0.0

        self.sub = self.create_subscription(Bool, "/run_state", self._on_run_state, 1)
        self.timer = self.create_timer(0.5, self._tick)

        self._proc: subprocess.Popen | None = None
        self._lock = threading.Lock()

        self.get_logger().info(
            f"DriverManager: robot_ip={self.robot_ip} host_ip={self.host_ip} ur_type={self.ur_type} headless={self.headless}"
        )

    def _ensure_started(self):
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                return
            # Build ros2 launch command
            args = [
                "ros2",
                "launch",
                "octa_ros",
                "launch.py",
                f"ur_type:={self.ur_type}",
                f"robot_ip:={self.robot_ip}",
                f"headless_mode:={'true' if self.headless else 'false'}",
                f"reverse_ip:={self.host_ip}",
            ]
            env = os.environ.copy()
            log_dir = env.get("RCUTILS_LOGGING_DIRECTORY", os.path.join(os.getcwd(), "logs"))
            os.makedirs(log_dir, exist_ok=True)
            env["RCUTILS_LOGGING_DIRECTORY"] = log_dir
            env["ROS_LOG_DIR"] = log_dir
            try:
                self._proc = subprocess.Popen(
                    args,
                    env=env,
                    stdout=None,
                    stderr=None,
                    preexec_fn=os.setsid,
                )
                self.get_logger().info("ros2 launch started (subprocess)")
            except Exception as e:
                self.get_logger().error(f"Failed to start ros2 launch: {e}")

    def _ensure_stopped(self):
        with self._lock:
            proc = self._proc
        if proc is None:
            return
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except Exception:
            pass
        try:
            proc.wait(timeout=5.0)
        except Exception:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except Exception:
                pass
        with self._lock:
            self._proc = None
        self.get_logger().info("ros2 launch stopped")

    def _on_run_state(self, msg: Bool):
        self.state_last_msg_time = time.monotonic()
        self.state_run_false = (msg.data is False)

    def _tick(self):
        online = _ping(self.robot_ip, self.ping_timeout)
        if not online:
            self._ensure_stopped()
            return

        if self.state_run_false:
            self._ensure_stopped()
        else:
            self._ensure_started()


def main():
    parser = argparse.ArgumentParser(description="ROS 2 Driver Manager")
    parser.add_argument("--robot-ip", default=os.environ.get("ROBOT_IP", "192.168.0.10"))
    parser.add_argument("--host-ip", default=os.environ.get("HOST_IP", ""))
    parser.add_argument("--ur-type", default=os.environ.get("UR_TYPE", "ur3e"))
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--ping-timeout", type=float, default=float(os.environ.get("PING_TIMEOUT", 3)))
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = DriverManager(args.robot_ip, args.host_ip, args.ur_type, args.headless, args.ping_timeout)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._ensure_stopped()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
