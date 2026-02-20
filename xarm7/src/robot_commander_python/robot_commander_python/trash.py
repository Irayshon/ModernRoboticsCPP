import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

from robot_commander_python.modern_robotics_tools import *
from robot_commander_python.forward_kinematics import *
from robot_commander_python.inverse_kinematics import *

class RobotGoalCommander(Node):
    def __init__(self):
        super().__init__('robot_goal_commander')
        
        # 1. Joint Configuration - Names must match robot_controller.yaml
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        self.num_joints = len(self.joint_names)

        self.current_pos = np.zeros(self.num_joints)
        self.current_vel = np.zeros(self.num_joints)
        self.current_eff = np.zeros(self.num_joints)
        
        
        # State Storage
        self.state_received = False

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

        # 3. Timer: Toggle between goals every 5 seconds
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.toggle = True

        self.get_logger().info('Commander Initialized. Alternating between predefined joint lists...')

    def joint_state_callback(self, msg):
        self.state_received = True

    def timer_callback(self):
        if not self.state_received:
            self.get_logger().info('Waiting for JointState data...')
            return

        if not self._action_client.server_is_ready():
            self.get_logger().info('Waiting for Action Server...')
            return
        
        # Base frame to end-effector home position
        M = np.array([[1, 0, 0, 0.206369], 
                      [0, -1, 0, 0], 
                      [0, 0, -1, 0.120954], 
                      [0, 0, 0, 1]])

        w_vals = np.array([[0,0,1], [0,1,0], [0,0,1], [0,-1,0], [0,0,-1], [0,1,0], [0,0,-1]])
        q_vals = np.array([[0,0,0.2668], [0,0,0.2668], [0,0,0.55969], [0.052665,0,0.55969], [0.130364,0,0.21711], [0.130364,0,0.21711], [0.206369,0,0.21711]])
        Slist = ConstructScrewlist(w_vals, q_vals)

        jointx = [-1.57, 1.57, -1.57, 1.57, -1.57, 1.57, -1.57]
        joints = [0.45, -1.2, 1, 0.5, 0.2, 1.2, 1.3]

        # Forward Kinematics
        tfk = FKin(M, Slist, joints)
        tfk = np.round(tfk, 2)
        
        # FIX: Convert numpy array to string for the logger
        self.get_logger().info(f"Forward Kinematics Matrix:\n{tfk}")

        Tx = tfk
        
        self.get_logger().info('Solving IK...')
        joints_target = IK_MPC(self.current_pos, Tx, N=10, R=np.eye(7)*0.01, Q = np.eye(6) * 100, 
                              screw_list=Slist, frame="space", M=M, 
                              tol_omg=1e-8, max_iters=500)
        
        tik = FKin(M, Slist, joints_target)
        tik = np.round(tik, 2)
        
        # FIX: Convert numpy array to string for the logger
        self.get_logger().info(f"inverse Kinematics joints:\n{joints_target}")
        self.get_logger().info(f"inverse Kinematics Matrix:\n{tik}")
        

        self.send_trajectory(joints)
        
     

    def send_trajectory(self, target_joints):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = target_joints
        point.time_from_start.sec = 4 # Duration of movement
        
        goal_msg.trajectory.points = [point]
        self.get_logger().info(f'Sending Goal List: {target_joints}')
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotGoalCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()