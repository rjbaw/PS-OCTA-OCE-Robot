import importlib.util
import os
import sys
import tempfile
import threading
import types
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
DRIVER_MANAGER = ROOT / "utils" / "driver_manager.py"


def load_driver_manager():
    """Load the module without requiring a ROS installation on the test host."""

    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = object
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Bool = object
    stubs = {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
    }
    spec = importlib.util.spec_from_file_location(
        "driver_manager_for_test", DRIVER_MANAGER
    )
    module = importlib.util.module_from_spec(spec)
    with mock.patch.dict(sys.modules, stubs):
        spec.loader.exec_module(module)
    return module


class DriverManagerChildExitTest(unittest.TestCase):
    def test_unexpected_child_exit_logs_provenance_before_restart(self):
        module = load_driver_manager()
        logger = mock.Mock()
        old_process = mock.Mock(pid=4242)
        old_process.poll.return_value = -6
        new_process = mock.Mock(pid=4343)
        manager = SimpleNamespace(
            _lock=threading.Lock(),
            _proc=old_process,
            _proc_start_time=100.0,
            _next_start_time=10.0,
            _last_stop_reason="old reason",
            robot_ip="192.0.2.10",
            host_ip="192.0.2.20",
            ur_type="ur3e",
            headless=True,
            launch_rviz=True,
            driver_healthy=False,
            health_last_msg_time=12.0,
            health_false_since=12.0,
            get_logger=lambda: logger,
        )

        with tempfile.TemporaryDirectory() as log_dir:
            with (
                mock.patch.object(module.time, "monotonic", return_value=130.25),
                mock.patch.object(
                    module.subprocess, "Popen", return_value=new_process
                ) as popen_mock,
                mock.patch.dict(
                    os.environ, {"RCUTILS_LOGGING_DIRECTORY": log_dir}, clear=False
                ),
            ):
                module.DriverManager._ensure_started(manager)

        logger.error.assert_called_once_with(
            "ros2 launch exited unexpectedly: "
            "pid=4242 return_code=-6 runtime_sec=30.250"
        )
        logger.info.assert_called_once_with(
            "ros2 launch started (subprocess): pid=4343"
        )
        self.assertIs(manager._proc, new_process)
        self.assertEqual(manager._proc_start_time, 130.25)
        self.assertEqual(manager._next_start_time, 0.0)
        self.assertIsNone(manager._last_stop_reason)
        popen_args = popen_mock.call_args.args[0]
        self.assertIn("headless_mode:=true", popen_args)
        self.assertIn("launch_rviz:=true", popen_args)


class DriverManagerArgumentsTest(unittest.TestCase):
    def test_launch_rviz_environment_is_independent_of_headless_mode(self):
        for value, expected in (("false", False), ("true", True)):
            with self.subTest(LAUNCH_RVIZ=value):
                module = load_driver_manager()
                module.rclpy.init = mock.Mock()
                module.rclpy.spin = mock.Mock()
                module.rclpy.shutdown = mock.Mock()

                with (
                    mock.patch.dict(os.environ, {"LAUNCH_RVIZ": value}),
                    mock.patch.object(sys, "argv", ["driver_manager.py"]),
                    mock.patch.object(module, "DriverManager") as manager_type,
                ):
                    module.main()

                constructor_args = manager_type.call_args.args
                self.assertTrue(constructor_args[3])
                self.assertIs(constructor_args[4], expected)


if __name__ == "__main__":
    unittest.main()
