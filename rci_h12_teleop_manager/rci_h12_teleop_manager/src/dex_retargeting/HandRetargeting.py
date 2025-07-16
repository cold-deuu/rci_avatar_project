from dex_retargeting.constants import (
    RobotName,
    RetargetingType,
    HandType,
    get_default_config_path,
)
from dex_retargeting.retargeting_config import RetargetingConfig

class HandRetargeting:
    def __init__(self):
        self.config_path_root = "/root/avatar_ws/src/h1_2_description/inspire_hand"
        self.l_config = self.config_path_root + "/inspire_hand_left.yml"
        self.r_config = self.config_path_root + "/inspire_hand_right.yml"
        
        # Default Urdf Path
        self.robot_dir = "/root/avatar_ws/src/h1_2_description"

        # 얘는 필요없을지도
        self.l_handtype = "Left"
        self.r_handtype = "Right"

        RetargetingConfig.set_default_urdf_dir(self.robot_dir)
        self.left_retargeting = RetargetingConfig.load_from_file(self.l_config).build()
        self.right_retargeting = RetargetingConfig.load_from_file(self.r_config).build()

        self.left_retargeting_joint_names = self.left_retargeting.joint_names
        self.right_retargeting_joint_names = self.right_retargeting.joint_names




        self.left_indices = self.left_retargeting.optimizer.target_link_human_indices
        self.right_indices = self.right_retargeting.optimizer.target_link_human_indices


        self.l_origin_idx = self.left_indices[0,:]
        self.l_task_idx = self.left_indices[1,:]
        
        self.r_origin_idx = self.right_indices[0,:]
        self.r_task_idx = self.right_indices[1,:]
        print(f"Joint Names : {self.r_origin_idx}")
        print(f"Joint Names : {self.r_task_idx}")



        self.inspire_api_joint_names  = [ 'pinky_proximal_joint', 'ring_proximal_joint', 'middle_proximal_joint',
                                            'index_proximal_joint', 'thumb_proximal_pitch_joint', 'thumb_proximal_yaw_joint' ]
        self.right_inspire_api_joint_names = [ 'R_pinky_proximal_joint', 'R_ring_proximal_joint', 'R_middle_proximal_joint',
                                            'R_index_proximal_joint', 'R_thumb_proximal_pitch_joint', 'R_thumb_proximal_yaw_joint' ]
        self.left_dex_retargeting_to_hardware = [ self.left_retargeting_joint_names.index(name) for name in self.inspire_api_joint_names]
        self.right_dex_retargeting_to_hardware = [ self.right_retargeting_joint_names.index(name) for name in self.right_inspire_api_joint_names]
