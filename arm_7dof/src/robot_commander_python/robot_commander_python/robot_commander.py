import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from robot_commander_python.modern_robotics_tools import *
from robot_commander_python.forward_kinematics import *

class RobotJointCommander(Node):
    def __init__(self):
        super().__init__('robot_joint_commander')

        from rclpy.parameter import Parameter
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.joint_names = [
            'joint1', 'joint3', 'joint5', 'joint6', 'joint7', 'joint8', 'joint9'
        ]

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/position_trajectory_controller/follow_joint_trajectory')

        self.get_logger().info('Robot Joint Commander Node has started.')

    def joint_state_callback(self, msg):
        """Callback to read current joint positions."""
        # Note: msg.name and msg.position may include joints in different orders
        # or include fixed joints if using a broadcaster.
        positions = dict(zip(msg.name, msg.position))
        
        # Log the position of a specific joint, e.g., joint1
        if 'joint1' in positions:
            self.get_logger().info(f"Current Joint1 Position: {positions['joint1']:.4f}", throttle_duration_sec=1.0)

    def send_goal(self, target_positions):
        """Sends a position goal to the trajectory controller."""
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.time_from_start.sec = 2  # Reach target in 2 seconds

        goal_msg.trajectory.points = [point]

        self.get_logger().info(f'Sending goal: {target_positions}')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotJointCommander()

    # Example: Command all joints to move to 0.5 radians
    # Order matches: joint1, joint3, joint5, joint6, joint7, joint8, joint9
    node.send_goal([3.5, -1.5, 0.5, 0.5, 0.5, 0.5, 0.5])

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()