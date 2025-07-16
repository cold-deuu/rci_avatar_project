import rclpy
from rclpy.node import Node

# from control_msgs.msg import DynamicJointState
from geometry_msgs.msg import Pose, PoseArray

import time
import numpy as np

import pinocchio as pin
from pinocchio.utils import *


from copy import deepcopy
class UnityWristPublisher(Node):
    def __init__(self):
        super().__init__("unity_wrist_publisher")

        cal_timer_period = 0.01  # seconds
        self.cal_timer = self.create_timer(cal_timer_period, self.cal_timer_callback)

        # arm Subscriber
        self.l_wrist_msg = Pose()
        self.r_wrist_msg = Pose()
        self.head_msg = Pose()
        self.l_subscription = self.create_subscription(Pose, '/lwrist_pose', self.l_wrist_CB, 10)
        self.r_subscription = self.create_subscription(Pose, '/rwrist_pose', self.r_wrist_CB, 10)
        self.head_subscription = self.create_subscription(Pose, '/head_pose', self.head_CB, 10)

        # RoS WAITING
        self.l_wirst_ready = False
        self.r_wirst_ready = False
        self.head_ready = False


        # Publisher - wrist
        self.lwrist_publisher = self.create_publisher(Pose, '/rci_h12_manager/l_wrist_pose', 10)
        self.rwrist_publisher = self.create_publisher(Pose, '/rci_h12_manager/r_wrist_pose', 10)

        # Publisher - Head
        self.head_publisher = self.create_publisher(Pose, '/rci_h12_manager/head_pose', 10)

        self.ar_sensor_publisher = self.create_publisher(PoseArray, '/rci_h12_manager/ar_sensor_pose', 10)

    def cal_timer_callback(self):
        if self.l_wirst_ready and self.r_wirst_ready and self.head_ready:
            l_wrist_se3 = self.computePosetoSE3(self.l_wrist_msg)
            r_wrist_se3 = self.computePosetoSE3(self.r_wrist_msg)
            head_se3 = self.computePosetoSE3(self.head_msg)

            head_se3_inv = head_se3.inverse()
            head_to_lwrist = head_se3_inv * l_wrist_se3
            head_to_rwrist = head_se3_inv * r_wrist_se3
            lwrist_pose = self.computeSE3toPose(head_to_lwrist)
            rwrist_pose = self.computeSE3toPose(head_to_rwrist)
            head_pose = self.computeSE3toPose(head_se3)
            
            pose_array = PoseArray()
            pose_array.poses.resize(3)
            pose_array[0] = head_pose
            pose_array[1] = lwrist_pose
            pose_array[2] = rwrist_pose
            
            self.ar_sensor_publisher.publish(pose_array)

            # self.lwrist_publisher.publish(lwrist_pose)
            # self.rwrist_publisher.publish(rwrist_pose)
            # self.head_publisher.publish(head_pose)

    def l_wrist_CB(self, msg):
        self.l_wrist_msg = msg
        self.l_wirst_ready = True


    def r_wrist_CB(self, msg):
        self.r_wrist_msg = msg
        self.r_wirst_ready = True

    def head_CB(self, msg):
        self.head_msg = msg
        self.head_ready = True

    def computePosetoSE3(self, msg : Pose):
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z
        ox = msg.orientation.x
        oy = msg.orientation.y
        oz = msg.orientation.z
        ow = msg.orientation.w
    
        xyzquat = np.array([x, y, z, ox, oy, oz, ow])
        se3_pose = pin.XYZQUATToSE3(xyzquat)

        return se3_pose

    def computeSE3toPose(self, se3 : pin.SE3):
        pose = pin.SE3ToXYZQUAT(se3)

        msg = Pose()
        msg.position.x = pose[0]
        msg.position.y = pose[1]
        msg.position.z = pose[2]
        msg.orientation.x = pose[3]
        msg.orientation.y = pose[4]
        msg.orientation.z = pose[5]
        msg.orientation.w = pose[6]

        return msg

def main():
    rclpy.init()
    node = UnityWristPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

