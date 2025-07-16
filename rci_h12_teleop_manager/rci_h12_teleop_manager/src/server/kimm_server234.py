import rclpy
from rclpy.node import Node
from rclpy.task import Future

from geometry_msgs.msg import Pose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from std_msgs.msg import Float64MultiArray

from action_interface.action import SE3

import numpy as np
from copy import deepcopy
from time import time

import pinocchio as pin
from pinocchio import RobotWrapper
from pinocchio.utils import *

from rb5_action_manager.src.controller.controller import rb5Controller


class SE3Server:
    def __init__(self, node, controller: rb5Controller):
        self.node = node
        self.controller = controller

        # [🟢] 설정: True이면 토픽만, False이면 Action만
        self.use_topic_input = True

        # Action server 등록
        self.action_server = ActionServer(
            self.node,
            SE3,
            "rci_rb5_control/ik_controller",
            goal_callback=self.goalCB,
            execute_callback=self.executeCB,
            cancel_callback=self.preemptCB
        )

        # 토픽 subscriber 등록
        self.matrix_sub = self.node.create_subscription(
            Float64MultiArray,
            '/ee_pose_matrix',
            self.ee_matrix_callback,
            10
        )

        # 내부 상태 변수
        self.target: np.array
        self.duration: float
        self.stime: float
        self.ctime: float
        self.initErr: pin.SE3
        self.oMi_0 = deepcopy(self.controller.robot.state.oMi)

        self.result_future = None
        self.current_goal_handle = None
        self.feedback = None
        self.is_running = False


    def goalCB(self, goal_request):
        if self.use_topic_input:
            self.node.get_logger().warn("Action goal ignored: using topic-based input.")
            return GoalResponse.REJECT

        self.oMi_0 = deepcopy(self.controller.robot.state.oMi)

        target = goal_request.target
        target_array = np.array([
            target.position.x, target.position.y, target.position.z,
            target.orientation.x, target.orientation.y,
            target.orientation.z, target.orientation.w
        ])

        targetSE3 = pin.XYZQUATToSE3(target_array)
        targetSE3.translation += self.oMi_0.translation
        targetSE3.rotation = self.oMi_0.rotation @ targetSE3.rotation

        self.duration = goal_request.duration
        self.controller.initSE3(targetSE3, self.duration)

        self.stime = time()
        self.initErr = pin.log(self.oMi_0.inverse() * targetSE3)
        self.is_running = True

        return GoalResponse.ACCEPT


    async def executeCB(self, goal_handle):
        self.result_future = Future()
        self.current_goal_handle = goal_handle
        self.feedback = SE3.Feedback()
        self.is_running = True
        return await self.result_future


    def ee_matrix_callback(self, msg: Float64MultiArray):
        if len(msg.data) != 17:
            self.node.get_logger().warn("Expected 17 floats (16 for matrix + 1 for duration)")
            return

        mat = np.array(msg.data[:16]).reshape((4, 4))
        duration = msg.data[16]

        T_target = pin.SE3(mat[:3, :3], mat[:3, 3])

        ...
        self.controller.initSE3(T_target, duration=duration)



    def compute(self):
        # if not self.is_running:
        #     return

        self.ctime = time()
        self.controller.controlSe3()

        oMi = deepcopy(self.controller.robot.state.oMi)
        currentErr = pin.log(self.oMi_0.inverse() * oMi)
        percent = 100.0 * np.linalg.norm(currentErr) / np.linalg.norm(self.initErr)
        percent = np.clip(percent, 0.0, 100.0)

        # Publish feedback if action is active
        if self.current_goal_handle is not None:
            feedback = SE3.Feedback()
            feedback.progress = percent
            self.current_goal_handle.publish_feedback(feedback)

        success = (self.ctime - self.stime) < self.duration + 1.0 and \
                  np.linalg.norm(currentErr - self.initErr) < 0.001

        timeout = (self.ctime - self.stime) > self.duration + 1.0

        if success or timeout:
            self.is_running = False
            self.controller.controlFlag = False

            if self.current_goal_handle is not None:
                res_msgs = SE3.Result()
                final_pose = Pose()
                xyzquat = pin.SE3ToXYZQUAT(oMi)
                final_pose.position.x, final_pose.position.y, final_pose.position.z = xyzquat[:3]
                final_pose.orientation.x, final_pose.orientation.y, final_pose.orientation.z, final_pose.orientation.w = xyzquat[3:]
                res_msgs.success = success
                res_msgs.final_position = final_pose

                if success:
                    self.current_goal_handle.succeed()
                    self.node.get_logger().info("SE3 goal succeeded.")
                else:
                    self.current_goal_handle.abort()
                    self.node.get_logger().warn("SE3 goal aborted (timeout).")

                self.result_future.set_result(res_msgs)
                self.current_goal_handle = None  # reset

    def preemptCB(self):
        pass
