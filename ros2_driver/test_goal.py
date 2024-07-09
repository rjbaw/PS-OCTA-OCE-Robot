import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from rclpy.node import Node

class JointTrajectoryActionClient(Node):

    def __init__(self):
        super().__init__('joint_trajectory_action_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self._urscript_publisher = self.create_publisher(String, '/urscript_interface/script_command', 10)
        
    def send_goal(self):
        # Create and send the FollowJointTrajectory goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = ['joint1', 'joint2']  # Replace with your joint names

        point = JointTrajectoryPoint()
        point.positions = [1.0, 0.5]  # Replace with desired joint positions
        point.time_from_start.sec = 2  # Replace with desired duration
        goal_msg.trajectory.points.append(point)

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.error_code}')

        # Send URScript command when the action is completed
        self.send_urscript_command()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback}')

    def send_urscript_command(self):
        script_command = String()
        script_command.data = (
            'def my_prog():\n'
            '  set_digital_out(1, True)\n'
            '  movej(p[0.5, 0.5, 0.5, 0, 0, 3.14], a=1.2, v=0.25, r=0)\n'
            '  textmsg("motion finished")\n'
            'end'
        )
        self._urscript_publisher.publish(script_command)
        self.get_logger().info('URScript command sent')

def main(args=None):
    rclpy.init(args=args)
    action_client = JointTrajectoryActionClient()
    action_client.send_goal()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()

