import unittest

import launch
from launch_ros.actions import Node
from launch.testing.legacy import LaunchTestService


class TestCILaunch(unittest.TestCase):
    def test_launch_nodes(self):
        # Only verify the nodes start; offline/plan-only avoids hardware deps
        ld = launch.LaunchDescription(
            [
                Node(
                    package="octa_ros",
                    executable="freedrive_node",
                    parameters=[{"dry_run": True}],
                ),
                Node(
                    package="octa_ros",
                    executable="reset_node",
                    parameters=[{"plan_only": True, "offline_mode": True}],
                ),
                Node(
                    package="octa_ros",
                    executable="move_z_angle_node",
                    parameters=[{"plan_only": True, "offline_mode": True}],
                ),
                Node(
                    package="octa_ros",
                    executable="focus_node",
                    parameters=[{"plan_only": True, "offline_mode": True}],
                ),
            ]
        )

        ls = LaunchTestService()
        ls.include_launch_description(ld)
        ls.run(timeout=10)
