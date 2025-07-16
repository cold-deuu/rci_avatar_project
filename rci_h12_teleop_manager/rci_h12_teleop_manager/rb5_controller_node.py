# ROS2
import rclpy
from rclpy.node import Node
# from rclpy.time import Time
# from rclpy.duration import Duration

# ROS2 - MSGS
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

# Python
import time
import numpy as np
from copy import deepcopy

# CUSTOM LIBRARY
from rci_h12_teleop_manager.src.robot.h12_wrapper import H12Wrapper
from rci_h12_teleop_manager.src.solver.qp_solver import RCI_QP_Solver

# PINOCCHIO
import pinocchio as pin
from pinocchio import RobotWrapper
from pinocchio.utils import *





class H12Simulator(Node):
    def __init__(self):
        super().__init__("h12_controller")

        self.robot = H12Wrapper()
        self.robot2 = H12Wrapper()

        cal_timer_period = 0.01  # seconds
        pub_timer_period = 0.01  # seconds
        self.cal_timer = self.create_timer(cal_timer_period, self.cal_timer_callback)
        self.pub_timer = self.create_timer(pub_timer_period, self.pub_timer_callback)

        # arm publisher
        self.arm_msg = Float64MultiArray()
        self.arm_publisher = self.create_publisher(Float64MultiArray, '/mujoco_ctrl', 10)
        # self.subscription = self.create_subscription(JointState, '/rci_h12_manager/joint_states', self.joint_state_callback, 10)
        self.subscription = self.create_subscription(JointState, '/mujoco/joint_states', self.joint_state_callback, 10)

        self.controlFlag = False
        self.initControl = True
        self.iter = 0

        self.l_oMi_init = pin.SE3()
        self.r_oMi_init = pin.SE3()

        self.goal_l = pin.SE3()
        self.goal_r = pin.SE3()

        self.qdes_l : np.array
        self.qdes_r : np.array


        # TEST
        self.initPostureFlag = False
        self.init_qdes = np.zeros((14))
        self.solver_l = RCI_QP_Solver(7) # Left
        self.solver_r = RCI_QP_Solver(7) # Right


    def cal_timer_callback(self):
        qdes = np.zeros((14))
        if self.controlFlag:
            if not self.initPostureFlag:
                qdes = self.init_qdes.copy()
                self.get_logger().warn(f"qerror :{np.linalg.norm(qdes - self.robot.state.q) }")
                if np.linalg.norm(qdes - self.robot.state.q) < 3e-2:
                    self.initPostureFlag = True
            else:
                self.get_logger().info("Flag")
                if self.initControl:
                    self.l_oMi_init = deepcopy(self.robot.state.l_oMi)
                    self.r_oMi_init = deepcopy(self.robot.state.r_oMi)
                    self.goal_l = deepcopy(self.l_oMi_init)
                    self.goal_r = deepcopy(self.r_oMi_init)

                    self.qdes_l = self.robot.state.q[:7].copy()
                    self.qdes_r = self.robot.state.q[7:].copy()

                    self.initControl = False
                else:
                    if self.iter<200:
                        self.goal_l.translation[2] += 0.0005
                        self.goal_r.translation[2] += 0.0005

                    l_dMi = self.robot.state.l_oMi.inverse() * self.goal_l
                    r_dMi = self.robot.state.r_oMi.inverse() * self.goal_r
                    x_err_l =pin.log(l_dMi)
                    x_err_r =pin.log(r_dMi)


                    # manip_left = np.sqrt(np.linalg.det(self.robot.state.left_J @ self.robot.state.left_J.T))
                    # manip_right = np.sqrt(np.linalg.det(self.robot.state.right_J @ self.robot.state.right_J.T))
                    # dev_manip_l , dev_manip_r = self.dev_manipulability(manip_left, manip_right, self.robot.state.q)

                    # CostP = np.eye((7))

                    # costQ_l = -dev_manip_l.copy()
                    # costQ_r = -dev_manip_r.copy()
                    # costQ_r = np.zeros((7))
                    # costQ_l = np.zeros((7))

                    # self.solver_l.AddCost(CostP, costQ_l)
                    # self.solver_l.AddEqTask(self.robot.state.left_J, 10 * x_err_l)
                    
                    # self.solver_r.AddCost(CostP, costQ_r)
                    # self.solver_r.AddEqTask(self.robot.state.right_J, 10 * x_err_r)      
                    
                    
                    # qdot_l = self.solver_l.solveQP()
                    # qdot_r = self.solver_r.solveQP()

                    qdot_l = np.linalg.pinv(self.robot.state.left_J) @ (10 * x_err_l)
                    qdot_r = np.linalg.pinv(self.robot.state.right_J) @ (10 * x_err_r)
                    
                    l_target_swivel = -1.0
                    null_l = (np.eye((7)) - np.linalg.pinv(self.robot.state.left_J) @ self.robot.state.left_J)
                    qdot_null_l = np.zeros((7))
                    xdot_new_l = np.array([0, -1.0 * (l_target_swivel-self.robot.l_swivel), 0])
                    qdot_null_l[1:5] = np.linalg.pinv(self.robot.state.left_J[:3, :4]) @ xdot_new_l
                    qdot_l += null_l @ qdot_null_l

                    r_target_swivel = 1.0
                    null_r = (np.eye((7)) - np.linalg.pinv(self.robot.state.right_J) @ self.robot.state.right_J)
                    qdot_null_r = np.zeros((7))
                    xdot_new_r = np.array([0, -1.0 * (r_target_swivel-self.robot.r_swivel), 0])
                    qdot_null_r[1:5] = np.linalg.pinv(self.robot.state.right_J[:3, :4]) @ xdot_new_r
                    qdot_r += null_r @ qdot_null_r

                    self.qdes_l += qdot_l * 0.01
                    self.qdes_r += qdot_r * 0.01

                qdes = np.append(self.qdes_l, self.qdes_r)



        # qdes = np.zeros((14))
        # qdes[6] = 1.57
        # qdes[13] = 1.57
        # self.get_logger().info(f"Pose l : {self.robot.state.l_oMi}")
        # self.get_logger().info(f"Pose r : {self.robot.state.r_oMi}")
        self.iter += 1
        self.arm_msg.data = qdes.tolist()

    def pub_timer_callback(self):
        self.arm_publisher.publish(self.arm_msg)
        return


    def joint_state_callback(self, msg):
        self.controlFlag = True
        self.robot.state.q = np.array(msg.position)
        self.robot.state.v = np.array(msg.velocity)
        self.robot.computeAllTerms()

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
            # self.get_logger().info(f"J_l {J_l_delta}")

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
            # self.get_logger().info(f"J_r {J_r_delta}")

            manip_r = np.sqrt(np.linalg.det(J_r_delta @ J_r_delta.T))
            del_manip_r = manip_r - manip_right
            del_manip_right[i] = del_manip_r
        # self.get_logger().info(f"{del_manip_left}, {del_manip_right}")

        return del_manip_left, del_manip_right
        
# ISS
def main():
    rclpy.init()
    node = H12Simulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

