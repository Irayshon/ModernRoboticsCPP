import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

class QuadrupedCommander(Node):
    def __init__(self):
        super().__init__('quadruped_commander')
        
        # 1. Quadruped Joint Names (Must match your YAML/URDF exactly)
        self.joint_names = [
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint'
        ]
        self.num_joints = len(self.joint_names)
        
        # State Storage
        self.current_pos = np.zeros(self.num_joints)
        self.state_received = False
        self.command_sent = False

        # 2. ROS Communication
        # Subscribing to Gazebo's feedback
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        # Action Client for the Trajectory Controller
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/joint_trajectory_controller/follow_joint_trajectory')

        # 3. Timers
        self.logic_timer = self.create_timer(0.5, self.logic_loop)

        self.get_logger().info('Quadruped Commander Initialized. Waiting for JointStates...')

    def joint_state_callback(self, msg):
        """Maps incoming Gazebo data to quadruped joint order."""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_pos[i] = msg.position[idx]
        
        self.state_received = True

    def logic_loop(self):
        """Waits for sim to start and server to be ready, then sends zeros."""
        if self.command_sent:
            return

        if not self.state_received:
            return

        if not self._action_client.server_is_ready():
            self.get_logger().info('Waiting for Joint Trajectory Action Server...')
            return

        self.get_logger().info('Systems ready. Sending Zero-Position Command...')
        self.send_zeros()
        self.command_sent = True

    def send_zeros(self):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        # Create a point with all zeros
        point = JointTrajectoryPoint()
        
        # EDIT THIS ARRAY LATER: [FR_h, FR_t, FR_c, FL_h, FL_t, FL_c, RR_h, RR_t, RR_c, RL_h, RL_t, RL_c]
        target_positions = [
            0.0, 0.0, -1.7,  # FR (Front Right)
            0.0, 0.0, -1.7,  # FL (Front Left)
            0.0, 0.0, -1.7,  # RR (Rear Right)
            0.0, 0.0, -1.7   # RL (Rear Left)
        ]
                
        point.positions = target_positions
        point.time_from_start.sec = 3  # Take 3 seconds to move to zero pose
        
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info(f'Sending zeros to all {self.num_joints} joints.')
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = QuadrupedCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()