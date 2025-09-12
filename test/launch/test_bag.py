import os
import time
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest
import launch_testing

try:
    import rclpy
    from rclpy.action import ActionClient
    from octa_ros.action import Focus as FocusAction

    RCLPY_AVAILABLE = True
except Exception:
    RCLPY_AVAILABLE = False


def generate_test_description():
    set_debug_env = SetEnvironmentVariable(name="OCTA_SAVE_DEBUG", value="1")
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    try:
        import shutil

        shutil.rmtree(os.path.join(repo_root, "result"), ignore_errors=True)
    except Exception:
        pass
    set_ros_log_dir = SetEnvironmentVariable(
        name="ROS_LOG_DIR", value=os.path.join(repo_root, "log")
    )

    pkg_share = FindPackageShare("octa_ros")

    labview_pub_script = os.path.join(repo_root, "test", "launch", "publish_labview.py")
    labview_pub = ExecuteProcess(
        cmd=["python3", "-u", labview_pub_script, "--hz", "10", "--toggle", "1.0"],
        output="screen",
    )
    scan3d_server_script = os.path.join(repo_root, "test", "launch", "scan3d_server.py")
    scan3d_server = ExecuteProcess(
        cmd=["python3", "-u", scan3d_server_script],
        output="screen",
    )

    ci_launch_path = os.path.join(repo_root, "test", "launch", "ci.launch.py")
    ros2_launch = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            ci_launch_path,
            "ur_type:=ur3e",
            "launch_rviz:=false",
            "launch_dashboard_client:=false",
            "use_mock_hardware:=true",
        ],
        output="screen",
        cwd=repo_root,
        additional_env={"ROS_LOG_DIR": os.path.join(repo_root, "log")},
    )

    bag_dir = PathJoinSubstitution([pkg_share, "bags", "fullscan"])
    qos_yaml = PathJoinSubstitution([pkg_share, "config", "bag_qos.yaml"])

    def retry(cmd):
        return [
            "bash",
            "-lc",
            "for i in $(seq 1 60); do "
            + " ".join(cmd)
            + " && exit 0; sleep 1; done; exit 2",
        ]

    set_exec_params = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=retry(
                    ["ros2", "param", "set", "/focus_node", "plan_only", "false"]
                ),
                output="screen",
            ),
            ExecuteProcess(
                cmd=retry(
                    ["ros2", "param", "set", "/focus_node", "offline_mode", "false"]
                ),
                output="screen",
            ),
            ExecuteProcess(
                cmd=retry(
                    [
                        "ros2",
                        "param",
                        "set",
                        "/focus_node",
                        "focus_step_timeout_sec",
                        "60.0",
                    ]
                ),
                output="screen",
            ),
            ExecuteProcess(
                cmd=retry(
                    [
                        "ros2",
                        "param",
                        "set",
                        "/focus_node",
                        "gating_interval_sec",
                        "0.0",
                    ]
                ),
                output="screen",
            ),
            ExecuteProcess(
                cmd=retry(
                    [
                        "ros2",
                        "param",
                        "set",
                        "/focus_node",
                        "scan3d_service_wait_ms",
                        "5000",
                    ]
                ),
                output="screen",
            ),
            ExecuteProcess(
                cmd=retry(
                    [
                        "ros2",
                        "param",
                        "set",
                        "/focus_node",
                        "scan3d_response_timeout_ms",
                        "10000",
                    ]
                ),
                output="screen",
            ),
            ExecuteProcess(
                cmd=retry(
                    ["ros2", "param", "set", "/focus_node", "image_width", "500"]
                ),
                output="screen",
            ),
            ExecuteProcess(
                cmd=retry(
                    ["ros2", "param", "set", "/focus_node", "image_height", "512"]
                ),
                output="screen",
            ),
        ],
    )
    bag_player = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            bag_dir,
            "--qos-profile-overrides-path",
            qos_yaml,
            "--rate",
            "10.0",
            "--loop",
        ],
        output="screen",
    )
    delayed_bag = TimerAction(period=5.0, actions=[bag_player])

    ld = LaunchDescription()
    ld.add_action(set_debug_env)
    ld.add_action(set_ros_log_dir)
    ld.add_action(labview_pub)
    ld.add_action(scan3d_server)
    ld.add_action(ros2_launch)
    ld.add_action(set_exec_params)
    ld.add_action(delayed_bag)
    ld.add_action(ReadyToTest())

    baseline_results = []
    try:
        result_root = os.path.join(repo_root, "result")
        baseline_results = [
            d
            for d in os.listdir(result_root)
            if os.path.isdir(os.path.join(result_root, d))
        ]
    except FileNotFoundError:
        baseline_results = []

    context = {"repo_root": repo_root, "baseline_results": baseline_results}
    return ld, context


@unittest.skipUnless(RCLPY_AVAILABLE, "rclpy not available")
class TestFocusAction(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_fullstack_focus_client")
        cls.client = ActionClient(cls.node, FocusAction, "focus_action")

    @classmethod
    def tearDownClass(cls):
        cls.client.destroy()
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_focus_goal_and_images(self):
        time.sleep(10.0)
        end = time.time() + 180.0
        while not self.client.wait_for_server(timeout_sec=0.5):
            if time.time() > end:
                self.fail("Focus action server not available")
        goal = FocusAction.Goal()
        goal.angle_tolerance = 1.0
        goal.z_tolerance = 0.5
        goal.z_height = 0.0
        self.client.send_goal_async(goal)

        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
        result_root = os.path.join(repo_root, "result")
        deadline = time.time() + 900.0
        found = False
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if os.path.isdir(result_root):
                sessions = [
                    d
                    for d in os.listdir(result_root)
                    if os.path.isdir(os.path.join(result_root, d))
                ]
                if sessions:
                    latest = os.path.join(
                        result_root,
                        max(
                            sessions,
                            key=lambda d: os.path.getmtime(
                                os.path.join(result_root, d)
                            ),
                        ),
                    )
                    import glob

                    det = glob.glob(os.path.join(latest, "detected_image*.jpg"))
                    raw = glob.glob(os.path.join(latest, "raw_image*.jpg"))
                    if len(det) >= 6 and len(raw) >= 6:
                        found = True
                        break
            time.sleep(0.5)
        self.assertTrue(
            found,
            "Result images not found within timeout (need >=6 raw and >=6 detected)",
        )


@launch_testing.post_shutdown_test()
class TestAfterShutdown(unittest.TestCase):
    def test_outputs_present(self, proc_info, proc_output):
        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
        result_root = os.path.join(repo_root, "result")
        self.assertTrue(
            os.path.isdir(result_root), f"result root missing: {result_root}"
        )

        try:
            sessions = [
                d
                for d in os.listdir(result_root)
                if os.path.isdir(os.path.join(result_root, d))
            ]
        except FileNotFoundError:
            sessions = []
        self.assertTrue(bool(sessions), "No result session directory found")
        latest = os.path.join(
            result_root,
            max(sessions, key=lambda d: os.path.getmtime(os.path.join(result_root, d))),
        )
        import glob

        detected = glob.glob(os.path.join(latest, "detected_image*.jpg"))
        raw = glob.glob(os.path.join(latest, "raw_image*.jpg"))
        self.assertTrue(
            len(detected) >= 6, "Latest session missing >=6 detected images"
        )
        self.assertTrue(len(raw) >= 6, "Latest session missing >=6 raw images")

        logs_dir = os.path.join(repo_root, "result", "logs")
        if os.path.isdir(logs_dir):
            import shutil

            try:
                shutil.rmtree(logs_dir)
            except Exception:
                pass
