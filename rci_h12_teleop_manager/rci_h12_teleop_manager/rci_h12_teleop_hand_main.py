from rci_h12_teleop_manager.src.dex_retargeting.HandRetargeting import HandRetargeting
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import numpy as np


import pinocchio as pin
from pinocchio.utils import *

from copy import deepcopy

Inspire_Num_Motors = 6


class Inspire_Controller(Node):
    def __init__(self):
        super().__init__("teleop_hand")
        self.hand_retargeting = HandRetargeting()
        self.left_landmark_subscriber = self.create_subscription(PoseArray, '/rci_hand_manager/left_landmarks', self.left_landmark_callback, 10)
        self.right_landmark_subscriber = self.create_subscription(PoseArray, '/rci_hand_manager/right_landmarks', self.right_landmark_callback, 10)
        
        self.l_hand_joint_publisher = self.create_publisher(JointState, '/rci_h12_manager/l_hand_joint_pose', 10)
        self.r_hand_joint_publisher = self.create_publisher(JointState, '/rci_h12_manager/r_hand_joint_pose', 10)

        self.lhand_landmark_list = np.empty((0, 3)) # None
        self.rhand_landmark_list = np.empty((0, 3)) # None

        self.cal_timer = self.create_timer(0.01, self.cal_timer_callback)

        self.lwrist_pose : pin.SE3
        self.rwrist_pose : pin.SE3

        self.l_land_flag = False
        self.r_land_flag = False


        self.l_unity_to_wrist_rot = np.zeros((3,3))
        self.l_unity_to_wrist_rot[0,0] = 1.0
        self.l_unity_to_wrist_rot[1,2] = 1.0
        self.l_unity_to_wrist_rot[2,1] = -1.0

        self.r_unity_to_wrist_rot = np.zeros((3,3))
        self.r_unity_to_wrist_rot[0,0] = 1.0
        self.r_unity_to_wrist_rot[1,2] = -1.0
        self.r_unity_to_wrist_rot[2,1] = 1.0

        # base_joint 의 origin 적용
        self.world_to_robot = np.zeros((3,3))
        self.world_to_robot[0,0] = -1.0
        self.world_to_robot[1,2] = -1.0
        self.world_to_robot[2,1] = -1.0



        self.l_urdf_to_wrist_rot = self.r_urdf_to_wrist_rot = np.eye((3))

        self.l_urdf_to_unity_rot = self.l_urdf_to_wrist_rot @  self.l_unity_to_wrist_rot.T
        self.r_urdf_to_unity_rot = self.r_urdf_to_wrist_rot @  self.r_unity_to_wrist_rot.T

        '''
        urdf 와 unity 의 baseline 이 동일하다고 가정,
        wrist 와 landmark 간의 상대 위치만 파악하면 되기에
        굳이 translation 을 생각할 필요가 없다다
        '''
        self.l_urdf_to_unity_se3 = pin.SE3(1)
        self.r_urdf_to_unity_se3 = pin.SE3(1)
        self.l_urdf_to_unity_se3.rotation = self.l_urdf_to_unity_rot.copy()
        self.r_urdf_to_unity_se3.rotation = self.r_urdf_to_unity_rot.copy()

    def left_landmark_callback(self, msg):
        landmarks = []  # 리스트로 시작
        self.l_wrist_to_lm_trans = []
        self.l_robot_wrist_to_lm_trans = []

        for pose in msg.poses:
            pose_array = [pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            landmarks.append(pose_array)

        # landmarks: shape (25, 3)
        self.lhand_landmark_list = np.array(landmarks[2:])  # shape: (7, 25)
        self.lwrist_pose = pin.XYZQUATToSE3(np.array([msg.poses[0].position.x, msg.poses[0].position.y, msg.poses[0].position.z, msg.poses[0].orientation.x, msg.poses[0].orientation.y, msg.poses[0].orientation.z, msg.poses[0].orientation.w]))

        self.l_urdf_to_wrist = self.l_urdf_to_unity_se3 * self.lwrist_pose

        # wrist 기준 hand joint
        for i in range(len(self.lhand_landmark_list)):
            landmark_se3 = pin.XYZQUATToSE3(self.lhand_landmark_list[i])
            self.l_wrist_to_lm_trans.append(self.lwrist_pose.rotation.T @ (landmark_se3.translation - self.lwrist_pose.translation))
            self.l_robot_wrist_to_lm_trans.append(self.world_to_robot.T @ self.lwrist_pose.rotation @ (landmark_se3.translation - self.lwrist_pose.translation))
   
        self.l_land_flag = True

    def right_landmark_callback(self, msg):
        landmarks = []  # 리스트로 시작
        self.r_wrist_to_lm_trans = []
        self.r_robot_wrist_to_lm_trans = []

        for pose in msg.poses:
            pose_array = [pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            landmarks.append(pose_array)

        # landmarks: shape (25, 3)
        self.rhand_landmark_list = np.array(landmarks[2:])  # shape: (7, 25)
        self.rwrist_pose = pin.XYZQUATToSE3(np.array([msg.poses[0].position.x, msg.poses[0].position.y, msg.poses[0].position.z, msg.poses[0].orientation.x, msg.poses[0].orientation.y, msg.poses[0].orientation.z, msg.poses[0].orientation.w]))

        self.r_urdf_to_wrist = self.r_urdf_to_unity_se3 * self.rwrist_pose

        for i in range(len(self.rhand_landmark_list)):
            landmark_se3 = pin.XYZQUATToSE3(self.rhand_landmark_list[i])
            self.r_wrist_to_lm_trans.append(self.rwrist_pose.rotation.T @ (landmark_se3.translation - self.rwrist_pose.translation))
            self.r_robot_wrist_to_lm_trans.append(self.world_to_robot.T @ self.rwrist_pose.rotation @ (landmark_se3.translation - self.rwrist_pose.translation))
        
        self.r_land_flag = True
    
    def cal_timer_callback(self):
        
        left_q_target  = np.full(Inspire_Num_Motors, 1.0)
        right_q_target = np.full(Inspire_Num_Motors, 1.0)

        if self.l_land_flag and self.r_land_flag:            
            left_hand_mat = np.array([])
            right_hand_mat = np.array([])
            zero3 = np.zeros((3))
            left_hand_mat = np.append(left_hand_mat, zero3)
            right_hand_mat = np.append(right_hand_mat, zero3)
            for i in range(len(self.r_wrist_to_lm_trans)):
                left_hand_mat = np.append(left_hand_mat, self.l_robot_wrist_to_lm_trans[i])
                right_hand_mat = np.append(right_hand_mat, self.r_robot_wrist_to_lm_trans[i])
                # left_hand_mat = np.append(left_hand_mat, self.l_wrist_to_lm_trans[i])
                # right_hand_mat = np.append(right_hand_mat, self.r_wrist_to_lm_trans[i])
            left_hand_mat = left_hand_mat.reshape(26, 3)
            right_hand_mat = right_hand_mat.reshape(26, 3)

            ref_left_value = (left_hand_mat[self.hand_retargeting.l_task_idx, :] -left_hand_mat[self.hand_retargeting.l_origin_idx, :])
            ref_right_value = (right_hand_mat[self.hand_retargeting.r_task_idx, :] -right_hand_mat[self.hand_retargeting.r_origin_idx, :])
            
            left_joint_msgs =  JointState()
            right_joint_msgs =  JointState()

            left_q_target  = self.hand_retargeting.left_retargeting.retarget(ref_left_value)[self.hand_retargeting.left_dex_retargeting_to_hardware]
            right_q_target = self.hand_retargeting.right_retargeting.retarget(ref_right_value)[self.hand_retargeting.right_dex_retargeting_to_hardware]
            
            left_joint_msgs.position = left_q_target.tolist()
            right_joint_msgs.position = right_q_target.tolist()

            self.l_hand_joint_publisher.publish(left_joint_msgs)
            self.r_hand_joint_publisher.publish(right_joint_msgs)


def main():
    rclpy.init()
    node = Inspire_Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()