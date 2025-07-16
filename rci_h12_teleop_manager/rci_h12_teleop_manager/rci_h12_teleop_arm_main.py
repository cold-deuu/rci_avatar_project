import rclpy
from rclpy.node import Node

# from rclpy.qos import QoSProfile
# from rclpy.qos import DurabilityPolicy
# from rclpy.qos import ReliabilityPolicy

# from control_msgs.msg import DynamicJointState
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
#######################################
from geometry_msgs.msg import Pose
#######################################

import time
import numpy as np

from rci_h12_teleop_manager.rci_h12_teleop_manager.src.robot.h12_wrapper import H12Wrapper
from rci_h12_teleop_manager.src.solver.qp_solver import RCI_QP_Solver

# from rb5_action_manager.src.server.kimm_joint_posture_server import JointPostureServer
# from rb5_action_manager.src.server.kimm_server import SE3Server


from rclpy.time import Time
from rclpy.duration import Duration

import pinocchio as pin
from pinocchio import RobotWrapper
from pinocchio.utils import *
import matplotlib.pyplot as plt

from copy import deepcopy
class H12Simulator(Node):
    def __init__(self):
        super().__init__("rb5_gazebo_simulator")

        # Pinocchio RobotWrapper
        self.robot = H12Wrapper()
        self.robot2 = H12Wrapper()

        cal_timer_period = 0.01 
        pub_timer_period = 0.01  
        self.cal_timer = self.create_timer(cal_timer_period, self.cal_timer_callback)
        self.pub_timer = self.create_timer(pub_timer_period, self.pub_timer_callback)

        # arm publisher
        self.arm_msg = Float64MultiArray()
        # self.arm_publisher = self.create_publisher(Float64MultiArray, '/mujoco_ctrl', 10)
        self.arm_publisher = self.create_publisher(Float64MultiArray, '/rci_h12_manager/unity_ctrl', 10)
        
        # Head Subscriber
        self.head_pose_sub = self.create_subscription(Pose, '/rci_h12_manager/head_pose', self.head_pose_callback, 10)

        # Target Subscriber
        self.lwrist_target_sub = self.create_subscription(Pose, '/rci_h12_manager/l_wrist_pose', self.lwrist_pose_callback, 10)
        self.rwrist_target_sub = self.create_subscription(Pose, '/rci_h12_manager/r_wrist_pose', self.rwrist_pose_callback, 10)
        
        # Joint State of Unity H1_2
        self.subscription = self.create_subscription(JointState, '/rci_h12_manager/joint_states', self.joint_state_callback, 10)

        self.rot_diff_init_l = np.zeros((3,3))
        self.rot_diff_init_r = np.zeros((3,3))


        self.rot_diff_init_l[0,0] = 1.0
        self.rot_diff_init_l[2,1] = 1.0
        self.rot_diff_init_l[1,2] = -1.0

        self.rot_diff_init_r[0,0] = 1.0
        self.rot_diff_init_r[2,1] = -1.0
        self.rot_diff_init_r[1,2] = 1.0

        self.initPostureFlag = False

        self.controlFlag = False
        self.initControl = True
        self.iter = 0

        # Modify to AR Head Pose
        self.current_head_se3 = pin.SE3()
        self.current_head_se3.translation = np.array([0.04874, 0.0, 0.67980])
        self.current_head_se3.rotation = np.eye((3))

        self.l_oMi_init = pin.SE3()
        self.r_oMi_init = pin.SE3()

        self.goal_l = pin.SE3()
        self.goal_r = pin.SE3()

        self.qdes_l : np.array
        self.qdes_r : np.array

        self.init_qdes = np.zeros((14))
        self.waitingFlag = True


        # RoS WAITING
        self.l_wirst_ready = False
        self.r_wirst_ready = False
        self.head_ready = False

        #QP Solver
        self.solver_l = RCI_QP_Solver(7) # Left
        self.solver_r = RCI_QP_Solver(7) # Right

        self.iter = 0

    def cal_timer_callback(self):
        qdes = np.zeros((14))
        if self.controlFlag:
            if not self.initPostureFlag:
                qdes = self.init_qdes.copy()
                self.get_logger().info(f"Init robot.state.l_oMi: {self.robot.state.l_oMi}")
                self.get_logger().info(f"Init robot.state.r_oMi: {self.robot.state.r_oMi}")
                self.get_logger().info(f"Initializing")
                if np.linalg.norm(qdes - self.robot.state.q) < 3e-2:
                    self.initPostureFlag = True

            else:
                if self.l_wirst_ready and self.r_wirst_ready and self.head_ready:
                    self.qdes_l = self.robot.state.q[:7].copy()
                    self.qdes_r = self.robot.state.q[7:].copy()
                    
                    # Compute Target SE3 w.r.t head
                    oMeef_l = self.current_head_se3 * self.goal_l
                    oMeef_r = self.current_head_se3 * self.goal_r

                    # Adjust Rotation btw AR Hand and Robot Hand
                    oMeef_l.rotation = oMeef_l.rotation @ self.rot_diff_init_l
                    oMeef_r.rotation = oMeef_r.rotation @ self.rot_diff_init_r

                    l_dMi = self.robot.state.l_oMi.inverse() * oMeef_l
                    r_dMi = self.robot.state.r_oMi.inverse() * oMeef_r

                    x_err_l =pin.log(l_dMi)
                    x_err_r =pin.log(r_dMi)

                    manip_left = np.sqrt(np.linalg.det(self.robot.state.left_J @ self.robot.state.left_J.T))
                    manip_right = np.sqrt(np.linalg.det(self.robot.state.right_J @ self.robot.state.right_J.T))
                    dev_manip_l , dev_manip_r = self.dev_manipulability(manip_left, manip_right, self.robot.state.q)

                    CostP = np.eye((7))
                    
                    costQ_l = -dev_manip_l.copy()
                    costQ_r = -dev_manip_r.copy()

                    self.solver_l.AddCost(CostP, costQ_l)
                    self.solver_l.AddEqTask(self.robot.state.left_J, 100 * x_err_l)
                    
                    self.solver_r.AddCost(CostP, costQ_r)
                    self.solver_r.AddEqTask(self.robot.state.right_J, 100 * x_err_r)      
                    
                    
                    qdot_l = self.solver_l.solveQP()
                    qdot_r = self.solver_r.solveQP()

                    self.qdes_l += qdot_l * 0.01
                    self.qdes_r += qdot_r * 0.01


                    qdes = np.append(self.qdes_l, self.qdes_r)
                 


                else:
                    # qtmp = None
                    # if self.waitingFlag:
                    #     qtmp = self.robot.state.q.copy()
                    #     self.waitingFlag = False
                    self.get_logger().info(f"Waiting Teleoperation")
                    qdes = self.robot.state.q.copy()

        # self.get_logger().info(f"Rot : {self.goal_r.rotation}")
        # self.get_logger().info(f"controlFlag2 : {self.controlFlag}")
        # if self.iter<200:
        #     qdes[0] = 0
        #     qdes[1] = 0
        #     qdes[2] = 0
        #     qdes[3] = 0
        #     qdes[4] = 0
        #     qdes[5] = 0
        #     qdes[6] = 0
        
        # for i in range(14):
        #     qdes[i] = 0.01 * i
        qdes = np.zeros((14))
        self.iter += 1
        self.arm_msg.data = qdes.tolist()
        


    def pub_timer_callback(self):
        self.arm_publisher.publish(self.arm_msg)
        return


    def joint_state_callback(self, msg):
        self.robot.state.q = np.array(msg.position)
        self.robot.state.v = np.array(msg.velocity)
        self.controlFlag = True
        self.robot.computeAllTerms()

    def head_pose_callback(self, msg : Pose):
        pos = np.array([msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

        unity_head_se3 = pin.XYZQUATToSE3(pos)

        # self.current_head_se3 = deepcopy(unity_head_se3)
        # self.current_head_se3.rotation = np.eye((3))
        # self.current_head_se3.translation[2] -= 0.8


        self.head_ready = True

    def lwrist_pose_callback(self, msg: Pose):
        pos = np.array([msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        
        self.goal_l = pin.XYZQUATToSE3(pos)
        self.get_logger().warn(f"Target Left : {self.goal_l}")

        self.l_wirst_ready = True

    def rwrist_pose_callback(self, msg: Pose):
        pos = np.array([msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

        self.goal_r = pin.XYZQUATToSE3(pos)
        # self.get_logger().warn(f"Target Right : {self.goal_r}")

        self.r_wirst_ready = True


    def dev_manipulability(self, manip_left, manip_right, q):
        eps = 1e-5
        v_tmp = np.zeros((14))
        del_manip_left = np.zeros((7))
        del_manip_right = np.zeros((7))
        for i in range(0, 7):
            q_current = q.copy()
            q_current[i] += eps
            self.robot2.state.q = q_current.copy()
            self.robot2.state.v = v_tmp.copy()
            self.robot2.computeAllTerms()
            J_l_delta = self.robot2.state.left_J.copy()

            manip_l = np.sqrt(np.linalg.det(J_l_delta @ J_l_delta.T))
            del_manip_l = manip_l - manip_left
            del_manip_left[i] = del_manip_l
        
        for i in range(0, 7):
            q_current = q.copy()
            q_current[i+7] += eps
            self.robot2.state.q = q_current.copy()
            self.robot2.state.v = v_tmp.copy()
            self.robot2.computeAllTerms()
            J_r_delta = self.robot2.state.right_J.copy()

            manip_r = np.sqrt(np.linalg.det(J_r_delta @ J_r_delta.T))
            del_manip_r = manip_r - manip_right
            del_manip_right[i] = del_manip_r

        return del_manip_left, del_manip_right
        
def main():
    rclpy.init()
    node = H12Simulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

