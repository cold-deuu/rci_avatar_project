import numpy as np

import pinocchio as pin
from pinocchio import RobotWrapper
from pinocchio.utils import *

from rclpy.logging import get_logger

from copy import deepcopy

def pretty_matrix(matrix, fmt="8.4f"):
    """Prints a matrix in a pretty format."""
    matrix = np.array(matrix)  # torch.Tensor도 numpy 변환됨
    rows, cols = matrix.shape
    print("[")
    for i in range(rows):
        row_str = "  [ " + "  ".join(f"{val:{fmt}}" for val in matrix[i]) + " ]"
        print(row_str)
    print("]")


class state():
    def __init__(self):
        self.q: np.array
        self.v: np.array

        self.nq: np.array
        self.nv: np.array
        self.na: np.array

        self.left_id: np.array
        self.right_id: np.array
        self.left_J: np.array
        self.right_J: np.array

        self.l_oMi : pin.SE3
        self.r_oMi : pin.SE3



class H12Wrapper(RobotWrapper):
    def __init__(self):
        # For Logging
        self.logger = get_logger("Robot_Wrapper")

        package_name = "rbpodo_description"
        model_path = "/home/chan/avatar_ws/src/h1_2_description"
        urdf_filename = "h1_2.urdf"
        urdf_path = model_path + "/" + urdf_filename
        self.__robot = self.BuildFromURDF(
            urdf_path,
            package_dirs=[model_path]
        )        

        self.state = state()
        self.mixed_jointsToLockIDs = [
                                      "left_hip_yaw_joint",
                                      "left_hip_pitch_joint",
                                      "left_hip_roll_joint",
                                      "left_knee_joint",
                                      "left_ankle_pitch_joint",
                                      "left_ankle_roll_joint",
                                      "right_hip_yaw_joint",
                                      "right_hip_pitch_joint",
                                      "right_hip_roll_joint",
                                      "right_knee_joint",
                                      "right_ankle_pitch_joint",
                                      "right_ankle_roll_joint",
                                      "torso_joint",
                                      "L_index_proximal_joint",
                                      "L_index_intermediate_joint",
                                      "L_middle_proximal_joint",
                                      "L_middle_intermediate_joint",
                                      "L_pinky_proximal_joint",
                                      "L_pinky_intermediate_joint",
                                      "L_ring_proximal_joint",
                                      "L_ring_intermediate_joint",
                                      "L_thumb_proximal_yaw_joint",
                                      "L_thumb_proximal_pitch_joint",
                                      "L_thumb_intermediate_joint",
                                      "L_thumb_distal_joint",
                                      "R_index_proximal_joint",
                                      "R_index_intermediate_joint",
                                      "R_middle_proximal_joint",
                                      "R_middle_intermediate_joint",
                                      "R_pinky_proximal_joint",
                                      "R_pinky_intermediate_joint",
                                      "R_ring_proximal_joint",
                                      "R_ring_intermediate_joint",
                                      "R_thumb_proximal_yaw_joint",
                                      "R_thumb_proximal_pitch_joint",
                                      "R_thumb_intermediate_joint",
                                      "R_thumb_distal_joint"
                                    ]
        self.reduced_robot = self.__robot.buildReducedRobot(
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=np.array([0.0] * self.__robot.model.nq),
        )
        
        self.model = self.reduced_robot.model

        self.data, self.__collision_data, self.__visual_data = \
            pin.createDatas(self.reduced_robot.model, self.reduced_robot.collision_model, self.reduced_robot.visual_model)
    
        l_eef_joint = "left_wrist_yaw_joint"
        self.state.left_id = self.index(l_eef_joint)
        r_eef_joint = "right_wrist_yaw_joint"
        self.state.right_id = self.index(r_eef_joint)

        self.state.nq = self.reduced_robot.nq 
        self.state.nv = self.reduced_robot.nv

        self.state.q = zero(self.state.nq)
        self.state.v = zero(self.state.nv)

        self.state.l_oMi = pin.SE3()
        self.state.r_oMi = pin.SE3()

    def compute_swivel(self, base, shoulder, elbow, wrist):
        a = deepcopy(base)
        b = deepcopy(shoulder)
        c = deepcopy(elbow)
        d = deepcopy(wrist)

        tmp1 = np.sign(np.dot(np.cross(b-a,c-b),d-b))
        tmp2 = np.dot(np.cross(b-a, d-b),np.cross(c-b, d-c))/(np.linalg.norm(np.cross(b-a, d-b)) * np.linalg.norm(np.cross(c-b, d-c)))
        
        return tmp1 * np.arccos(tmp2)

    def computeAllTerms(self):
        pin.computeAllTerms(self.model, self.data, self.state.q, self.state.v)

        self.state.left_J = self.getJointJacobian(self.index("left_wrist_yaw_joint"))[:,:int(self.state.nq/2)]
        self.state.right_J = self.getJointJacobian(self.index("right_wrist_yaw_joint"))[:,int(self.state.nq/2):]

        self.state.l_oMi = self.data.oMi[self.index("left_wrist_yaw_joint")]
        self.state.r_oMi = self.data.oMi[self.index("right_wrist_yaw_joint")]
        
        lbase = self.data.oMi[self.index("left_shoulder_pitch_joint")]
        lshoulder = self.data.oMi[self.index("left_shoulder_roll_joint")]
        lelbow = self.data.oMi[self.index("left_elbow_joint")]
        lwrist = deepcopy(self.state.l_oMi)
        rbase = self.data.oMi[self.index("right_shoulder_pitch_joint")]
        rshoulder = self.data.oMi[self.index("right_shoulder_roll_joint")]
        relbow = self.data.oMi[self.index("right_elbow_joint")]
        rwrist = deepcopy(self.state.r_oMi)

        self.l_swivel = self.compute_swivel(lbase.translation, lshoulder.translation, lelbow.translation, lwrist.translation)
        self.r_swivel = self.compute_swivel(rbase.translation, rshoulder.translation, relbow.translation, rwrist.translation)

        # self.logger.info(f"Left Swivel : {self.l_swivel}")
        # self.logger.info(f"Right Swivel : {self.r_swivel}")
