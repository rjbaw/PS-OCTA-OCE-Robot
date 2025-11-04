from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_mock = LaunchConfiguration("use_mock_hardware", default="true")

    common_parameters = []

    nodes = [
        Node(
            package="octa_ros",
            executable="freedrive_node",
            name="freedrive_node",
            output="screen",
            parameters=common_parameters + [{"dry_run": use_mock}],
        ),
        Node(
            package="octa_ros",
            executable="reset_node",
            name="reset_node",
            output="screen",
            parameters=common_parameters
            + [
                {"plan_only": use_mock},
                {"offline_mode": use_mock},
            ],
        ),
        Node(
            package="octa_ros",
            executable="move_node",
            name="move_node",
            output="screen",
            parameters=common_parameters
            + [
                {"plan_only": use_mock},
                {"offline_mode": use_mock},
            ],
        ),
        Node(
            package="octa_ros",
            executable="focus_node",
            name="focus_node",
            output="screen",
            parameters=common_parameters
            + [
                {"plan_only": use_mock},
                {"offline_mode": use_mock},
            ],
        ),
    ]

    return LaunchDescription(nodes)
