# ROS2
import rclpy
from rclpy.node import Node
from rclpy.task import Future

# ROS2 Msgs
from geometry_msgs.msg import Pose

# ROS2 Action
from rclpy.action import ActionServer, CancelResponse, GoalResponse

# Action Files
from action_interface.action import JointPosture

# Python Library
import numpy as np
from time import time

# RB5 Controller
from rb5_action_manager.src.controller.controller import rb5Controller

# Pinocchio
import pinocchio as pin
from pinocchio import RobotWrapper
from pinocchio.utils import *

class JointPostureServer:
    def __init__(self, node, controller : rb5Controller):
        self.node = node
        self.result_future = None
        self.action_server = ActionServer(self.node,
                                          JointPosture,
                                          "rci_rb5_control/joint_controller",
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
        self.goal_active = False

        # Init
        self.qInit : np.array



    def goalCB(self, goal_request):
        target = goal_request.target # Geometry Msgs
        self.target = np.array(target)

        self.duration = goal_request.duration
        self.node.get_logger().info(f'Received Target = {self.target}')
        self.node.get_logger().info(f'Received Duration = {self.duration}')

        # Inital Info
        self.controller.initJointPosture(self.target, self.duration)
        self.qInit = np.copy(self.controller.robot.state.q)
        self.stime = time()

        return GoalResponse.ACCEPT

    async def executeCB(self, goal_handle):
        self.result_future = Future()

        self.current_goal_handle = goal_handle
        self.feedback = JointPosture.Feedback()
        self.goal_active = True

        return await self.result_future


    def step_execution(self):    
        if not self.goal_active:
            return

        self.ctime = time()
        self.controller.controlJointPosture()

        percent = 100.0 * np.linalg.norm(self.controller.robot.state.q - self.qInit) / np.linalg.norm(self.target - self.qInit)
        percent = np.clip(percent, 0.0, 100.0)

        feedback = JointPosture.Feedback()
        feedback.progress = percent
        self.current_goal_handle.publish_feedback(feedback)

        if (self.ctime - self.stime)<self.duration + 1.0 and np.linalg.norm(self.target - self.controller.robot.state.q)<0.001:
            self.current_goal_handle.succeed()
            goal_success = True
            self.goal_active = False
            self.controller.controlFlag = False
            print("Success")

        elif (self.ctime - self.stime)>self.duration + 1.0:
            self.current_goal_handle.abort()
            goal_success = False
            self.goal_active = False
            self.controller.controlFlag = False
            print("Fail")

        if self.controller.controlFlag == False:
            res_msgs = JointPosture.Result()

            oMi = self.controller.robot.state.oMi
            goal_target = Pose()

            # Action
            # Target Translation
            goal_target.position.x = oMi.translation[0]
            goal_target.position.y = oMi.translation[1]
            goal_target.position.z = oMi.translation[2]

            # Target Orientation
            goal_target.orientation.x = 0.0
            goal_target.orientation.y = 0.0
            goal_target.orientation.z = 0.0
            goal_target.orientation.w = 1.0


            res_msgs.success = goal_success
            res_msgs.final_position = goal_target

            self.result_future.set_result(res_msgs)




    def preemptCB(self):
        pass
