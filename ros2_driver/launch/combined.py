import os
import time
import yaml
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    IncludeLaunchDescription, #
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    AndSubstitution,
    NotSubstitution,
)

from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory

from launch.launch_description_sources import AnyLaunchDescriptionSource

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None

# Spawn controllers
def controller_spawner(controllers, controller_spawner_timeout, active=True):
    inactive_flags = ["--inactive"] if not active else []
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            controller_spawner_timeout,
        ]
        + inactive_flags
        + controllers,
    )



def generate_launch_description():

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    # General arguments
    controllers_file = LaunchConfiguration("controllers_file")
    description_launchfile = LaunchConfiguration("description_launchfile")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    headless_mode = LaunchConfiguration("headless_mode")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    tool_device_name = LaunchConfiguration("tool_device_name")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")

    launch_rviz = LaunchConfiguration("launch_rviz")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    launch_servo = LaunchConfiguration("launch_servo")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")

    # safety_limits = LaunchConfiguration("safety_limits")
    # safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    # safety_k_position = LaunchConfiguration("safety_k_position")
    # description_file = LaunchConfiguration("description_file")
    # tf_prefix = LaunchConfiguration("tf_prefix")
    # robot_ip = LaunchConfiguration("robot_ip")
    # joint_limit = LaunchConfiguration("joint_limit")
    # kinematics_params = LaunchConfiguration("kinematics_params")
    # physical_params = LaunchConfiguration("physical_params")
    # visual_params = LaunchConfiguration("visual_params")
    # robot_description = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         description_file,
    #         " ",
    #         "robot_ip:=",
    #         robot_ip,
    #         " ",
    #         "joint_limit_params:=",
    #         joint_limit,
    #         " ",
    #         "kinematics_params:=",
    #         kinematics_params,
    #         " ",
    #         "physical_params:=",
    #         physical_params,
    #         " ",
    #         "visual_params:=",
    #         visual_params,
    #         " ",
    #         "safety_pos_margin:=",
    #         safety_pos_margin,
    #         " ",
    #         "safety_k_position:=",
    #         safety_k_position,
    #         " ",
    #         "name:=",
    #         "ur",
    #         " ",
    #         "ur_type:=",
    #         ur_type,
    #         " ",
    #         "prefix:=",
    #         "''",
    #         " ",
    #         "tf_prefix:=",
    #         tf_prefix,
    #     ]
    # )
    # robot_description_semantic = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
    #         " ",
    #         "name:=",
    #         "ur",
    #         # Also ur_type parameter could be used but then the planning group names in yaml
    #         # configs has to be updated!
    #         " ",
    #         "prefix:=",
    #         "''",
    #         " ",
    #     ]
    # )
    # robot_description = {"robot_description": robot_description}
    # robot_description_semantic = {
    #     "robot_description_semantic": robot_description_semantic
    # }
    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    # )
    # robot_state_publisher_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="both",
    #     parameters=[robot_description],
    # )
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    # )
    # nodes_to_start = [
    #     joint_state_publisher_node,
    #     robot_state_publisher_node,
    #     rviz_node,
    # ]
    # return LaunchDescription(declared_arguments + nodes_to_start)

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            LaunchConfiguration("update_rate_config_file"),
            ParameterFile(controllers_file, allow_substs=True),
            # We use the tf_prefix as substitution in there, so that's why we keep it as an
            # argument for this launchfile
        ],
        output="screen",
    )

    dashboard_client_node = Node(
        package="ur_robot_driver",
        condition=IfCondition(
            AndSubstitution(launch_dashboard_client, NotSubstitution(use_mock_hardware))
        ),
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": robot_ip}],
    )

    tool_communication_node = Node(
        package="ur_robot_driver",
        condition=IfCondition(use_tool_communication),
        executable="tool_communication.py",
        name="ur_tool_comm",
        output="screen",
        parameters=[
            {
                "robot_ip": robot_ip,
                "tcp_port": tool_tcp_port,
                "device_name": tool_device_name,
            }
        ],
    )

    urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        parameters=[{"robot_ip": robot_ip}],
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
    )

    controller_stopper_node = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(use_mock_hardware),
        parameters=[
            {"headless_mode": headless_mode},
            {"joint_controller_active": activate_joint_controller},
            {
                "consistent_controllers": [
                    "io_and_status_controller",
                    "force_torque_sensor_broadcaster",
                    "joint_state_broadcaster",
                    "speed_scaling_state_broadcaster",
                ]
            },
        ],
    )

    # rviz_node = Node(
    #     package="rviz2",
    #     condition=IfCondition(launch_rviz),
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    # )

    controllers_active = [
        "joint_state_broadcaster",
        "io_and_status_controller",
        "speed_scaling_state_broadcaster",
        "force_torque_sensor_broadcaster",
    ]
    controllers_inactive = ["forward_position_controller"]

    controller_spawners = [controller_spawner(controllers_active,
                                              controller_spawner_timeout)] + [
        controller_spawner(controllers_inactive,
                           controller_spawner_timeout,
                           active=False)
    ]

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            initial_joint_controller,
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            controller_spawner_timeout,
        ],
        condition=IfCondition(activate_joint_controller),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            initial_joint_controller,
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            controller_spawner_timeout,
            "--inactive",
        ],
        condition=UnlessCondition(activate_joint_controller),
    )

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description(Path("urdf") / "ur.urdf.xacro")
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": ur_type})
        .robot_description_kinematics(Path("config") / "kinematics.yaml")
        .joint_limits(Path("config") / "joint_limits.yaml")
        # .planning_pipelines(
        #     pipelines=["ompl", "pilz_industrial_motion_planner"],
        #     default_planning_pipeline="pilz_industrial_motion_planner",
        # )
        .to_moveit_configs()
    )

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }
    
    ld = LaunchDescription()
    ld.add_entity(
        LaunchDescription(
            [
                DeclareLaunchArgument(
                    "ur_type",
                    description="Type/series of used UR robot.",
                    choices=[
                        "ur3",
                        "ur3e",
                        "ur5",
                        "ur5e",
                        "ur10",
                        "ur10e",
                        "ur16e",
                        "ur20",
                        "ur30",
                    ],
                ),
                DeclareLaunchArgument(
                    "robot_ip", description="IP address by which the robot can be reached."
                ),
                DeclareLaunchArgument(
                    "safety_limits",
                    default_value="true",
                    description="Enables the safety limits controller if true.",
                ),
                DeclareLaunchArgument(
                    "safety_pos_margin",
                    default_value="0.15",
                    description="The margin to lower and upper limits in the safety controller.",
                ),
                DeclareLaunchArgument(
                    "safety_k_position",
                    default_value="20",
                    description="k-position factor in the safety controller.",
                ),
                DeclareLaunchArgument(
                    "controllers_file",
                    default_value=PathJoinSubstitution(
                        [FindPackageShare("ur_robot_driver"), "config", "ur_controllers.yaml"]
                    ),
                    description="YAML file with the controllers configuration.",
                ),
                DeclareLaunchArgument(
                    "description_launchfile",
                    default_value=PathJoinSubstitution(
                        [FindPackageShare("ur_robot_driver"), "launch", "ur_rsp.launch.py"]
                    ),
                    description="Launchfile (absolute path), providing the description. "
                    "The launchfile has to start a robot_state_publisher node that "
                    "publishes the description topic.",
                ),
                DeclareLaunchArgument(
                    "tf_prefix",
                    default_value="",
                    description="tf_prefix of the joint names, useful for "
                    "multi-robot setup. If changed, also joint names in the controllers' configuration "
                    "have to be updated.",
                ),
                DeclareLaunchArgument(
                    "use_mock_hardware",
                    default_value="false",
                    description="Start robot with mock hardware mirroring command to its states.",
                ),
                DeclareLaunchArgument(
                    "mock_sensor_commands",
                    default_value="false",
                    description="Enable mock command interfaces for sensors used for simple simulations. "
                    "Used only if 'use_mock_hardware' parameter is true.",
                ),
                DeclareLaunchArgument(
                    "headless_mode",
                    default_value="false",
                    description="Enable headless mode for robot control",
                ),
                DeclareLaunchArgument(
                    "controller_spawner_timeout",
                    default_value="10",
                    description="Timeout used when spawning controllers.",
                ),
                DeclareLaunchArgument(
                    "initial_joint_controller",
                    default_value="scaled_joint_trajectory_controller",
                    description="Initially loaded robot controller.",
                ),
                DeclareLaunchArgument(
                    "activate_joint_controller",
                    default_value="true",
                    description="Activate loaded joint controller.",
                ),
                DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"),
                DeclareLaunchArgument(
                    "rviz_config_file",
                    default_value=PathJoinSubstitution(
                        [FindPackageShare("ur_description"), "rviz", "view_robot.rviz"]
                    ),
                    description="RViz config file (absolute path),to use when launching rviz.",
                ),
                DeclareLaunchArgument(
                    "launch_dashboard_client",
                    default_value="true",
                    description="Launch Dashboard Client?",
                ),
                DeclareLaunchArgument(
                    "use_tool_communication",
                    default_value="false",
                    description="Only available for e series!",
                ),
                DeclareLaunchArgument(
                    "tool_parity",
                    default_value="0",
                    description="Parity configuration for serial communication. Only effective, if "
                    "use_tool_communication is set to True.",
                ),
                DeclareLaunchArgument(
                    "tool_baud_rate",
                    default_value="115200",
                    description="Baud rate configuration for serial communication. Only effective, if "
                    "use_tool_communication is set to True.",
                ),
                DeclareLaunchArgument(
                    "tool_stop_bits",
                    default_value="1",
                    description="Stop bits configuration for serial communication. Only effective, if "
                    "use_tool_communication is set to True.",
                ),
                DeclareLaunchArgument(
                    "tool_rx_idle_chars",
                    default_value="1.5",
                    description="RX idle chars configuration for serial communication. Only effective, "
                    "if use_tool_communication is set to True.",
                ),
                DeclareLaunchArgument(
                    "tool_tx_idle_chars",
                    default_value="3.5",
                    description="TX idle chars configuration for serial communication. Only effective, "
                    "if use_tool_communication is set to True.",
                ),
                DeclareLaunchArgument(
                    "tool_device_name",
                    default_value="/tmp/ttyUR",
                    description="File descriptor that will be generated for the tool communication device. "
                    "The user has be be allowed to write to this location. "
                    "Only effective, if use_tool_communication is set to True.",
                ),
                DeclareLaunchArgument(
                    "tool_tcp_port",
                    default_value="54321",
                    description="Remote port that will be used for bridging the tool's serial device. "
                    "Only effective, if use_tool_communication is set to True.",
                ),
                DeclareLaunchArgument(
                    "tool_voltage",
                    default_value="0",  # 0 being a conservative value that won't destroy anything
                    description="Tool voltage that will be setup.",
                ),
                DeclareLaunchArgument(
                    "reverse_ip",
                    default_value="0.0.0.0",
                    description="IP that will be used for the robot controller to communicate back to the driver.",
                ),
                DeclareLaunchArgument(
                    "script_command_port",
                    default_value="50004",
                    description="Port that will be opened to forward URScript commands to the robot.",
                ),
                DeclareLaunchArgument(
                    "reverse_port",
                    default_value="50001",
                    description="Port that will be opened to send cyclic instructions from the driver to the robot controller.",
                ),
                DeclareLaunchArgument(
                    "script_sender_port",
                    default_value="50002",
                    description="The driver will offer an interface to query the external_control URScript on this port.",
                ),
                DeclareLaunchArgument(
                    "trajectory_port",
                    default_value="50003",
                    description="Port that will be opened for trajectory control.",
                ),
                DeclareLaunchArgument(
                    name="update_rate_config_file",
                    default_value=[
                        PathJoinSubstitution(
                            [
                                FindPackageShare("ur_robot_driver"),
                                "config",
                            ]
                        ),
                        "/",
                        LaunchConfiguration("ur_type"),
                        "_update_rate.yaml",
                    ],
                ),
                DeclareLaunchArgument(
                    "warehouse_sqlite_path",
                    default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
                    description="Path where the warehouse database should be stored",
                ),
                DeclareLaunchArgument(
                    "launch_servo", default_value="false", description="Launch Servo?"
                ),
                DeclareLaunchArgument(
                    "use_sim_time",
                    default_value="false",
                    description="Using or not time from simulation",
                ),
                DeclareLaunchArgument(
                    "publish_robot_description_semantic",
                    default_value="true",
                    description="MoveGroup publishes robot description semantic",
                ),
                DeclareLaunchArgument(
                    "joint_limit",
                    default_value=PathJoinSubstitution(
                        [FindPackageShare("ur_description"),
                        "config", "ur3e", "joint_limits.yaml"]
                    ),
                    description="joint",
                ),
                DeclareLaunchArgument(
                    "kinematics_params",
                    default_value=PathJoinSubstitution(
                        [FindPackageShare("ur_description"),
                         "config", "ur3e", "default_kinematics.yaml"]
                    ),
                    description="joint",
                ),
                DeclareLaunchArgument(
                    "physical_params",
                    default_value=PathJoinSubstitution(
                        [FindPackageShare("ur_description"),
                         "config", "ur3e", "physical_parameters.yaml"]
                    ),
                    description="joint",
                ),
                DeclareLaunchArgument(
                    "visual_params",
                    default_value=PathJoinSubstitution(
                        [FindPackageShare("ur_description"),
                         "config", "ur3e", "visual_parameters.yaml"]
                    ),
                    description="joint",
                ),
                DeclareLaunchArgument(
                    "tf_prefix",
                    default_value="''",
                    description="Prefix of the joint names, useful for "
                    "multi-robot setup. If changed than also joint names in the controllers' configuration "
                    "have to be updated.",
                ),
                DeclareLaunchArgument(
                    "safety_limits",
                    default_value="true",
                    description="Enables the safety limits controller if true.",
                ),
                DeclareLaunchArgument(
                    "safety_pos_margin",
                    default_value="0.15",
                    description="The margin to lower and upper limits in the safety controller.",
                ),
                DeclareLaunchArgument(
                    "safety_k_position",
                    default_value="20",
                    description="k-position factor in the safety controller.",
                ),
                DeclareLaunchArgument(
                    "description_file",
                    default_value=PathJoinSubstitution(
                        [FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]
                    ),
                    description="URDF/XACRO description file (absolute path) with the robot.",
                ),
            ]
        )
    )

    rsp = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(description_launchfile),
        launch_arguments={
            "robot_ip": robot_ip,
            "ur_type": ur_type,
        }.items(),
    )

    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time,
                "publish_robot_description_semantic": publish_robot_description_semantic,
            },
        ],
    )

    servo_yaml = load_yaml("ur_moveit_config", "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node",
        parameters=[
            moveit_config.to_dict(),
            servo_params,
        ],
        output="screen",
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
        ],
    )

    robot_state_node = Node(
        package="octa_ros",
        executable="joint_state_listener",
        name="joint_state_listener",
        output="screen",
    )
    nodes_to_start = [
        control_node,
        dashboard_client_node,
        tool_communication_node,
        controller_stopper_node,
        urscript_interface,
        rsp,
        #rviz_node,
        initial_joint_controller_spawner_stopped,
        initial_joint_controller_spawner_started,
    ] + controller_spawners

    #ld.add_action(LaunchDescription(nodes_to_start))

    # for node in nodes_to_start:
    #     ld.add_action(node)
    # ld.add_action(LaunchDescription(nodes_to_start))

    ld.add_action(wait_robot_description)

    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_robot_description,
                on_exit=[
                    move_group_node,
                    rviz_node,
                    servo_node,
                    robot_state_node,
                ],
            )
        ),
    )

    return ld
