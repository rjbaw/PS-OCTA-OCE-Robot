#!/usr/bin/env python3
import argparse, rclpy, sys, time
from rclpy.node import Node
from std_msgs.msg import Bool


class RunStateListener(Node):
    def __init__(self, timeout, writer):
        super().__init__("run_state_listener")
        self.timeout_sec = timeout
        self.writer = writer
        self.last_msg_time = time.monotonic()
        self.create_subscription(Bool, "/run_state", self._cb, 1)
        self.create_timer(0.5, self._watchdog)

    def _cb(self, msg: Bool):
        self.last_msg_time = time.monotonic()
        self._emit("true" if msg.data else "false")

    def _watchdog(self):
        if time.monotonic() - self.last_msg_time > self.timeout_sec:

            if self.last_msg_time != -1:
                self._emit("unavailable")
                self.last_msg_time = -1

    def _emit(self, text: str):
        self.writer.write(text + "\n")
        self.writer.flush()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--unavailable-timeout",
        type=float,
        default=5.0,
        help='Seconds without a message before emitting "unavailable"',
    )
    parser.add_argument("--log-file", help="If given, tee output to this file")
    args, ros_args = parser.parse_known_args()

    writer = sys.stdout
    if args.log_file:
        log_fh = open(args.log_file, "w", buffering=1)

        class Tee:
            def __init__(self, a, b):
                self.a, self.b = a, b

            def write(self, data):
                self.a.write(data)
                self.b.write(data)

            def flush(self):
                self.a.flush()
                self.b.flush()

        writer = Tee(sys.stdout, log_fh)

    rclpy.init(args=ros_args)
    node = RunStateListener(args.unavailable_timeout, writer)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
