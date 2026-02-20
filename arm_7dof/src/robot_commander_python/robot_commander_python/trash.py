import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

# Modern Robotics tools
from robot_commander_python.modern_robotics_tools import *
from robot_commander_python.forward_kinematics import *
from robot_commander_python.inverse_kinematics import *

class RobotLiveCommander(Node):
    def __init__(self, M_home):
        super().__init__('robot_live_commander')
        self.M_home = M_home
        
        # 1. Joint Configuration
        self.joint_names = ['joint1', 'joint3', 'joint5', 'joint6', 'joint7', 'joint8', 'joint9']
        self.num_joints = len(self.joint_names)
        
        # State Storage
        self.current_pos = np.zeros(self.num_joints)
        self.current_vel = np.zeros(self.num_joints)
        self.current_eff = np.zeros(self.num_joints)
        
        self.state_received = False
        self.ik_executed = False

        # 2. ROS Communication
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/position_trajectory_controller/follow_joint_trajectory')

        # 3. Timers
        # Check every 0.5s if we are ready to send the command
        self.logic_timer = self.create_timer(0.5, self.logic_loop)
        # Log data every 1s
        self.log_timer = self.create_timer(1.0, self.status_logger)

        self.get_logger().info('Node Initialized. Monitoring Position, Velocity, and Effort...')

    def joint_state_callback(self, msg):
        """Continuously maps incoming data to our joint order."""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_pos[i] = msg.position[idx]
                if len(msg.velocity) > idx:
                    self.current_vel[i] = msg.velocity[idx]
                if len(msg.effort) > idx:
                    self.current_eff[i] = msg.effort[idx]
        
        self.state_received = True

    def logic_loop(self):
        """Main control logic: waits for data and server, then triggers IK."""
        if self.ik_executed:
            return

        if not self.state_received:
            self.get_logger().info('Waiting for initial JointState...')
            return

        # Check if Action Server is ready
        if not self._action_client.server_is_ready():
            self.get_logger().info('Waiting for Action Server...')
            return

        # If we have data and server is ready, run IK
        self.get_logger().info('All systems ready. Computing Inverse Kinematics...')
        self.run_ik_and_send()
        self.ik_executed = True

    def run_ik_and_send(self):
        # Screw Axis Definition
        w_vals = np.array([[0,0,1], [1,0,0], [1,0,0], [0,0,1], [1,0,0], [0,0,1], [1,0,0]])
        q_vals = np.array([[0,0,1], [2,0,5], [0,0,9], [0,0,13], [0,0,14], [0,0,16], [0,0,17]])
        Slist = ConstructScrewlist(w_vals, q_vals)

        # Target Pose
        Tx = np.array([[1, 0, 0, 5],
                       [0, 1, 0, 5],
                       [0, 0, 1, 10],
                       [0, 0, 0, 1]])
        
        # Solve IK using CURRENT positions as the starting point
        self.get_logger().info('Solving IK...')
        joints_target = IK_MPC(self.current_pos, Tx, N=10, R=np.eye(7), Q=np.eye(6), 
                              screw_list=Slist, frame="space", M=self.M_home, 
                              tol_omg=1e-8, max_iters=500)

        self.send_trajectory(joints_target)

    def send_trajectory(self, target_joints):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = target_joints.tolist()
        point.time_from_start.sec = 6 # Smooth move
        
        goal_msg.trajectory.points = [point]
        self.get_logger().info(f'Sending goal: {np.round(target_joints, 3)}')
        self._action_client.send_goal_async(goal_msg)

    def status_logger(self):
        """Periodic terminal output of robot health."""
        if self.state_received:
            print("\n" + "="*30)
            print(f"POS: {np.round(self.current_pos, 2)}")
            print(f"VEL: {np.round(self.current_vel, 2)}")
            print(f"EFF: {np.round(self.current_eff, 2)}")
            print("="*30)

def main(args=None):
    rclpy.init(args=args)
    M = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 19], [0, 0, 0, 1]])
    node = RobotLiveCommander(M)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()