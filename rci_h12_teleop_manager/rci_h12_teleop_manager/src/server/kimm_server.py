# ROS2
import rclpy
from rclpy.node import Node
from rclpy.task import Future

# ROS2 Msgs
from geometry_msgs.msg import Pose

# ROS2 Action
from rclpy.action import ActionServer, CancelResponse, GoalResponse

# Action Files
from action_interface.action import SE3

# Python Library
import numpy as np
from copy import deepcopy
from time import time


# RB5 Controller
from rb5_action_manager.src.controller.controller import rb5Controller

# Pinocchio
import pinocchio as pin
from pinocchio import RobotWrapper
from pinocchio.utils import *


class SE3Server:
    def __init__(self, node, controller : rb5Controller):
        self.node = node
        self.action_server = ActionServer(self.node,
                                          SE3,
                                          "rci_rb5_control/ik_controller",
                                          goal_callback = self.goalCB,
                                          execute_callback = self.executeCB,
                                          cancel_callback= self.preemptCB)
        # Action
        self.target : np.array
        self.duration : float

        # Controller
        self.controller = controller

        # Timer
        self.stime : time
        self.ctime : time

        # Server Running
        self.is_running = False
        
        # Init
        self.oMi_0 = deepcopy(self.controller.robot.state.oMi)
        self.initErr : pin.SE3
        print("init")



    def goalCB(self, goal_request):
        self.oMi_0 = deepcopy(self.controller.robot.state.oMi)

        # Target XYZ QUAT
        target = goal_request.target # Geometry Msgs
        target_array = np.array([target.position.x, target.position.y, target.position.z,
                                target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w])
        
        # QUAT To SE3
        targetSE3 = pin.XYZQUATToSE3(target_array)
        targetSE3.translation = targetSE3.translation + self.oMi_0.translation
        targetSE3.rotation = self.oMi_0.rotation @ targetSE3.rotation
        
        print(targetSE3)

        # Target Duration
        self.duration = goal_request.duration

        # Control Setting
        self.controller.initSE3(targetSE3, self.duration)

        # Timer
        self.stime = time()

        # For Feedback
        self.initErr = pin.log(self.oMi_0.inverse() * targetSE3)

        return GoalResponse.ACCEPT

    async def executeCB(self, goal_handle):
        self.result_future = Future()

        self.current_goal_handle = goal_handle
        self.feedback = SE3.Feedback()
        self.is_running = True


        res_msgs = SE3.Result()

        # print("sfdsfd")
        # goal_target = Pose()
        # goal_success = True
        # # Action
        # # Target Translation
        # goal_target.position.x = 0.0
        # goal_target.position.y = 0.0
        # goal_target.position.z = 0.0

        # # Target Orientation
        # goal_target.orientation.x = 0.0
        # goal_target.orientation.y = 0.0
        # goal_target.orientation.z = 0.0
        # goal_target.orientation.w = 1.0
        
        # goal_handle.succeed()

        # res_msgs.success = goal_success
        # res_msgs.final_position = goal_target
        
        # return res_msgs

        return await self.result_future
    
    def compute(self):
        if not self.is_running:
            return
        
        self.ctime = time()
        self.controller.controlSe3()

        oMi = deepcopy(self.controller.robot.state.oMi)
        currentErr = pin.log(self.oMi_0.inverse() * oMi)


        percent = 100.0 * np.linalg.norm(currentErr) / np.linalg.norm(self.initErr)
        percent = np.clip(percent, 0.0, 100.0)

        feedback = SE3.Feedback()
        feedback.progress = percent
        self.current_goal_handle.publish_feedback(feedback)
    
        if (self.ctime - self.stime)<self.duration + 1.0 and np.linalg.norm(currentErr - self.initErr)<0.001:
            self.current_goal_handle.succeed()
            goal_success = True
            self.is_running = False
            self.controller.controlFlag = False
            print("Success")

        elif (self.ctime - self.stime)>self.duration + 1.0:
            self.current_goal_handle.abort()
            goal_success = False
            self.goal_active = False
            self.controller.controlFlag = False
            print("Fail")

        if self.controller.controlFlag == False:
            res_msgs = SE3.Result()

            goal_target = Pose()
            xyzquat = pin.SE3ToXYZQUAT(oMi)


            # Action
            # Target Translation
            goal_target.position.x = xyzquat[0]
            goal_target.position.y = xyzquat[1]
            goal_target.position.z = xyzquat[2]

            # Target Orientation
            goal_target.orientation.x = xyzquat[3]
            goal_target.orientation.y = xyzquat[4]
            goal_target.orientation.z = xyzquat[5]
            goal_target.orientation.w = xyzquat[6]


            res_msgs.success = goal_success
            res_msgs.final_position = goal_target

            self.result_future.set_result(res_msgs)



    def preemptCB(self):
        pass
