#!/usr/bin/env python3
import argparse
import os
import signal
import socket
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


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
    def __init__(
        self,
        robot_ip: str,
        host_ip: str,
        ur_type: str,
        headless: bool,
        ping_timeout: float,
        run_state_stale_sec: float,
        ur_health_startup_grace_sec: float,
        ur_health_stale_sec: float,
        ur_health_unhealthy_sec: float,
        restart_delay_sec: float,
        restart_cooldown_sec: float,
    ):
        super().__init__("driver_manager")
        self.robot_ip = robot_ip
        self.host_ip = host_ip or _detect_host_ip(robot_ip)
        self.ur_type = ur_type
        self.headless = headless
        self.ping_timeout = ping_timeout
        self.run_state_stale_sec = run_state_stale_sec
        self.ur_health_startup_grace_sec = ur_health_startup_grace_sec
        self.ur_health_stale_sec = ur_health_stale_sec
        self.ur_health_unhealthy_sec = ur_health_unhealthy_sec
        self.restart_delay_sec = restart_delay_sec
        self.restart_cooldown_sec = restart_cooldown_sec

        self.state_run_false = False
        self.state_last_msg_time = 0.0

        self.driver_healthy: bool | None = None
        self.health_last_msg_time = 0.0
        self.health_false_since: float | None = None

        self.sub = self.create_subscription(Bool, "/run_state", self._on_run_state, 1)
        self.health_sub = self.create_subscription(
            Bool, "/ur_driver_healthy", self._on_driver_health, 1
        )
        self.timer = self.create_timer(0.5, self._tick)

        self._proc: subprocess.Popen | None = None
        self._lock = threading.Lock()
        self._proc_start_time = 0.0
        self._next_start_time = 0.0
        self._last_restart_time = 0.0
        self._last_stop_reason: str | None = None

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
            log_dir = env.get(
                "RCUTILS_LOGGING_DIRECTORY", os.path.join(os.getcwd(), "logs")
            )
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
                now = time.monotonic()
                self._proc_start_time = now
                self._next_start_time = 0.0
                self._last_stop_reason = None
                self.driver_healthy = None
                self.health_last_msg_time = 0.0
                self.health_false_since = None
                self.get_logger().info("ros2 launch started (subprocess)")
            except Exception as e:
                self.get_logger().error(f"Failed to start ros2 launch: {e}")

    def _ensure_stopped(self, reason: str | None = None):
        if reason is not None and reason != self._last_stop_reason:
            self.get_logger().warn(f"UR driver stop condition: {reason}")
            self._last_stop_reason = reason
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
            try:
                proc.wait(timeout=5.0)
            except Exception:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except Exception:
                    pass
        with self._lock:
            self._proc = None
        self.get_logger().info("ros2 launch stopped")

    def _on_run_state(self, msg: Bool):
        self.state_last_msg_time = time.monotonic()
        self.state_run_false = msg.data is False

    def _on_driver_health(self, msg: Bool):
        now = time.monotonic()
        self.health_last_msg_time = now
        self.driver_healthy = bool(msg.data)
        if self.driver_healthy:
            self.health_false_since = None
        else:
            if self.health_false_since is None:
                self.health_false_since = now

    def _request_restart(self, reason: str):
        now = time.monotonic()
        if self.restart_cooldown_sec > 0.0:
            if now - self._last_restart_time < self.restart_cooldown_sec:
                return
        self._last_restart_time = now
        self._next_start_time = now + max(0.0, self.restart_delay_sec)
        self.get_logger().warn(f"Restarting UR driver: {reason}")
        self._ensure_stopped()

    def _tick(self):
        now = time.monotonic()
        online = _ping(self.robot_ip, self.ping_timeout)
        if not online:
            self._ensure_stopped(reason="robot not reachable (ping/dashboard)")
            return

        run_state_recent = (
            self.state_last_msg_time > 0.0
            and now - self.state_last_msg_time <= self.run_state_stale_sec
        )
        if run_state_recent and self.state_run_false:
            self._ensure_stopped(reason="/run_state is false")
            return

        if self._next_start_time > 0.0 and now < self._next_start_time:
            return

        with self._lock:
            proc = self._proc
        if proc is None or proc.poll() is not None:
            self._ensure_started()
            return

        if self._proc_start_time > 0.0 and (
            now - self._proc_start_time < self.ur_health_startup_grace_sec
        ):
            return

        health_recent = (
            self.health_last_msg_time > 0.0
            and now - self.health_last_msg_time <= self.ur_health_stale_sec
        )
        if not health_recent:
            self._request_restart("ur health topic stale")
            return

        if self.driver_healthy is False and self.health_false_since is not None:
            if now - self.health_false_since >= self.ur_health_unhealthy_sec:
                self._request_restart("/ur_driver_healthy=false")


def main():
    parser = argparse.ArgumentParser(description="ROS 2 Driver Manager")
    parser.add_argument(
        "--robot-ip", default=os.environ.get("ROBOT_IP", "192.168.0.10")
    )
    parser.add_argument("--host-ip", default=os.environ.get("HOST_IP", ""))
    parser.add_argument("--ur-type", default=os.environ.get("UR_TYPE", "ur3e"))
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument(
        "--ping-timeout", type=float, default=float(os.environ.get("PING_TIMEOUT", 3))
    )
    parser.add_argument(
        "--run-state-stale-sec",
        type=float,
        default=float(os.environ.get("RUN_STATE_STALE_SEC", 10)),
        help="Seconds after which /run_state is treated as absent (driver allowed to run).",
    )
    parser.add_argument(
        "--ur-health-startup-grace-sec",
        type=float,
        default=float(os.environ.get("UR_HEALTH_STARTUP_GRACE_SEC", 30)),
        help="Seconds after stack start before enforcing /ur_driver_healthy.",
    )
    parser.add_argument(
        "--ur-health-stale-sec",
        type=float,
        default=float(os.environ.get("UR_HEALTH_STALE_SEC", 20)),
        help="Seconds without /ur_driver_healthy updates before restarting stack.",
    )
    parser.add_argument(
        "--ur-health-unhealthy-sec",
        type=float,
        default=float(os.environ.get("UR_HEALTH_UNHEALTHY_SEC", 20)),
        help="Seconds /ur_driver_healthy must remain false before restarting stack.",
    )
    parser.add_argument(
        "--restart-delay-sec",
        type=float,
        default=float(os.environ.get("RESTART_DELAY_SEC", 5)),
        help="Seconds to wait after stopping before starting again.",
    )
    parser.add_argument(
        "--restart-cooldown-sec",
        type=float,
        default=float(os.environ.get("RESTART_COOLDOWN_SEC", 30)),
        help="Minimum seconds between restarts (prevents thrashing).",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = DriverManager(
        args.robot_ip,
        args.host_ip,
        args.ur_type,
        args.headless,
        args.ping_timeout,
        args.run_state_stale_sec,
        args.ur_health_startup_grace_sec,
        args.ur_health_stale_sec,
        args.ur_health_unhealthy_sec,
        args.restart_delay_sec,
        args.restart_cooldown_sec,
    )
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
