import mujoco
import mujoco.viewer
import numpy as np
import time
from mujoco import viewer

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

import pinocchio as pin
from pinocchio.utils import *


class MujocoSim(Node):
    def __init__(self):
        super().__init__('mujoco_h12_simulator')
        pkg_path = get_package_share_directory("h1_2_description")
        xml_path = pkg_path + "/h1_2/h1_2_main.xml"
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.joint_indices = {name: i for i, name in enumerate(mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
                                                        for i in range(self.model.njnt))}

        print(f"len : {len(self.data.ctrl)}")
        self.num_joints = 14
        self.ctrl = np.zeros((self.num_joints))

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/mujoco_ctrl',
            self.mujoco_command_callback,
            10)

        self.publisher_ = self.create_publisher(JointState, '/mujoco/joint_states', 10)

        self.initSimulation = True
        self.controlFlag = False

        self.timer_period = 0.01  # 100Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # show joint idx
        for i in range(self.model.njnt):
            joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            qpos_adr = self.model.jnt_qposadr[i]
            qvel_adr = self.model.jnt_dofadr[i]
            print(f"[{i}] Joint: {joint_name}, qpos index: {qpos_adr}, qvel index: {qvel_adr}")

    def mujoco_command_callback(self, msg):
        self.controlFlag = True
        for i in range(self.num_joints):
            self.ctrl[i] = msg.data[i]

    
    def joint_states_publish(self, qpos, qvel):
        joint_states = JointState()
        left_qpos = qpos[7:14].copy()
        right_qpos = qpos[14:20].copy()
        arm_qpos = np.append(left_qpos, right_qpos)

        left_qvel = qvel[6:12].copy()
        right_qvel = qvel[13:19].copy()
        arm_qvel = np.append(left_qvel, right_qvel)

        joint_states.position = arm_qpos.tolist()
        joint_states.velocity = arm_qvel.tolist()
        self.publisher_.publish(joint_states)

    def timer_callback(self):
        self.joint_states_publish(self.data.qpos, self.data.qvel)

        if self.initSimulation:
            self.data.qpos = np.zeros((21))
            self.data.qpos[:7] = np.array([0,0,1.03, 0,0,0,1]) # first 7 : floating joint
            # mujoco.mj_forward(self.model, self.data)
            mujoco.mj_step(self.model, self.data)

            self.initSimulation = False
        if self.controlFlag:
            self.data.qpos[:7] = np.array([0,0,1.03, 0,0,0,1])
            # self.data.qpos[20:27] = self.ctrl[:7].copy()
            # self.data.qpos[39:46] = self.ctrl[7:].copy()
            # self.data.ctrl[:] = self.ctrl
            # mujoco.mj_forward(self.model, self.data)
            self.data.ctrl[:] = self.ctrl  # ex: torque, force, position targets
            mujoco.mj_step(self.model, self.data)

    def main(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while rclpy.ok() and viewer.is_running():
                rclpy.spin_once(self, timeout_sec=0.0)
                viewer.sync()

def main(args=None):
    rclpy.init(args=args)
    mjSim = MujocoSim()
    mjSim.main()
    mjSim.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
