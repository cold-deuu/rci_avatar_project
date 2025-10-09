import rclpy
from rclpy.node import Node

# from control_msgs.msg import DynamicJointState
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Float32, Float64MultiArray, Int16MultiArray

import time
import numpy as np

import pinocchio as pin
from pinocchio.utils import *


from copy import deepcopy
class UnityWristPublisher(Node):
    def __init__(self):
        super().__init__("unity_wrist_publisher")

        cal_timer_period = 1.0/30  # seconds
        self.cal_timer = self.create_timer(cal_timer_period, self.cal_timer_callback)

        # Publisher - hand
        self.lhand_publisher = self.create_publisher(Int16MultiArray, '/rci_h12_manager/l_hand_pose', 10)
        self.rhand_publisher = self.create_publisher(Int16MultiArray, '/rci_h12_manager/r_hand_pose', 10)

        self.subscription = self.create_subscription(Int16MultiArray, '/hand/scaled_joint', self.right_hand_cb, 10)

        self.iterate = 0

        self.right_hand_msg = Int16MultiArray()

        self.ok = False
    def cal_timer_callback(self):
        # 0번부터 새끼손가락, 약지, 중지, 검지, 엄지 위, 아래 순
    
        msg = Int16MultiArray()
        # if self.iterate % 6 ==0:
        #     msg.data = [1000,1000,1000,1000,1000,1000]
        # elif self.iterate % 6 ==1:
        #     msg.data = [0,0,1000,1000,200,1000]
        # elif self.iterate % 6 ==2:
        #     msg.data = [1000,1000,1000,1000,1000,1000]
        # elif self.iterate % 6 ==3:
        #     msg.data = [500,500,500,500,500,500]
        # elif self.iterate % 6 == 4:
        #     msg.data = [1000,1000,1000,1000,1000,1000]
        # elif self.iterate % 6 == 5:
        #     msg.data = [0,0,0,0,200,1000]

        msg = Int16MultiArray()
        msg.data = [1000, 1000, 1000, 1000, 1000, 1000]
        self.iterate +=1

        if self.ok:
            self.lhand_publisher.publish(msg)
            self.rhand_publisher.publish(self.right_hand_msg)

    def right_hand_cb(self, msg):
        self.right_hand_msg.data = msg.data
        self.ok = True



def main():
    rclpy.init()
    node = UnityWristPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

