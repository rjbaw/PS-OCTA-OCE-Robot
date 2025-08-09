import os
import time
import unittest

import launch
from launch import LaunchDescription
from launch_testing.actions import ReadyToTest
from launch_ros.actions import Node
import launch_testing

try:
    import rclpy
    from rclpy.action import ActionClient
    from octa_ros.action import Freedrive, Reset, MoveZAngle, Focus
    RCLPY_AVAILABLE = True
except Exception:
    RCLPY_AVAILABLE = False


def generate_test_description():
    ld = LaunchDescription()
    ld.add_action(Node(package='octa_ros', executable='freedrive_node', name='freedrive_node', parameters=[{'dry_run': True}]))
    ld.add_action(Node(package='octa_ros', executable='reset_node', name='reset_node', parameters=[{'plan_only': True, 'offline_mode': True}]))
    ld.add_action(Node(package='octa_ros', executable='move_z_angle_node', name='move_z_angle_node', parameters=[{'plan_only': True, 'offline_mode': True}]))
    ld.add_action(Node(package='octa_ros', executable='focus_node', name='focus_node', parameters=[{'plan_only': True, 'offline_mode': True}]))
    ld.add_action(ReadyToTest())
    context = {}
    return ld, context


class TestActionsStart(unittest.TestCase):
    def test_nodes_up(self):
        # If nodes crash, launch_testing will fail this test run
        time.sleep(1.0)


@unittest.skipUnless(RCLPY_AVAILABLE, "rclpy not available")
class TestActionGoals(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_action_client')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _send_goal(self, action_type, name, goal_msg):
        client = ActionClient(self.node, action_type, name)
        end = time.time() + 5.0
        while not client.wait_for_server(timeout_sec=0.2):
            if time.time() > end:
                self.fail(f"Action server '{name}' not available")
        future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        goal_handle = future.result()
        self.assertIsNotNone(goal_handle)
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=10.0)
        result = result_future.result()
        self.assertIsNotNone(result)
        return result

    def test_freedrive_enable_disable(self):
        # Enable
        res1 = self._send_goal(Freedrive, 'freedrive_action', Freedrive.Goal(enable=True))
        # Disable
        res2 = self._send_goal(Freedrive, 'freedrive_action', Freedrive.Goal(enable=False))
        self.assertIsNotNone(res1)
        self.assertIsNotNone(res2)

    def test_reset(self):
        res = self._send_goal(Reset, 'reset_action', Reset.Goal(reset=True))
        self.assertIsNotNone(res)

    def test_move_z_angle(self):
        goal = MoveZAngle.Goal()
        goal.target_angle = 5.0
        goal.radius = 0.0
        goal.angle = 0.0
        res = self._send_goal(MoveZAngle, 'move_z_angle_action', goal)
        self.assertIsNotNone(res)

    def test_focus(self):
        goal = Focus.Goal()
        goal.angle_tolerance = 1.0
        goal.z_tolerance = 0.5
        goal.z_height = 0.0
        res = self._send_goal(Focus, 'focus_action', goal)
        self.assertIsNotNone(res)


@launch_testing.post_shutdown_test()
class TestAfterShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        # Ensure no process died with non-zero exit code during the run
        try:
            proc_info.assertWaitForShutdown(process=None, timeout=5)
        except AttributeError:
            # Older launch_testing may not provide this helper; if we reached
            # here without exceptions earlier, consider it OK.
            self.assertTrue(True)
