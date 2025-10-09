import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from action_interface.action import SE3, JointPosture
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QProgressBar,
                             QPushButton, QHBoxLayout, QGridLayout, QLabel,
                             QLineEdit, QCheckBox, QGroupBox)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QIcon

from ament_index_python.packages import get_package_share_directory

# Define Progress Bar Color
PROGRESS_BAR_STYLE_NORMAL = """
QProgressBar::chunk {
    background-color: #3add36; /* 녹색 */
    width: 20px;
}
QProgressBar {
    border: 2px solid grey;
    border-radius: 5px;
    text-align: center;
}
"""

PROGRESS_BAR_STYLE_FAIL = """
QProgressBar::chunk {
    background-color: #d63031; /* 빨간색 */
    width: 20px;
}
QProgressBar {
    border: 2px solid grey;
    border-radius: 5px;
    text-align: center;
}
"""

class SimplifiedActionClientGUI(Node, QWidget):
    def __init__(self):
        QWidget.__init__(self)
        Node.__init__(self, "simplified_action_client_gui")

        pkg_path = get_package_share_directory("rci_h12_controller")

        se3_img_path = pkg_path + "/gui_images/reach.png"
        home_img_path = pkg_path + "/gui_images/home.png"

        self.setWindowTitle("Simplified Action Client")
        self.setGeometry(100, 100, 600, 500)

        # Joint Posture
        joint_groupbox = QGroupBox("Joint Posture Goal")
        joint_layout = QVBoxLayout()
        joint_grid_layout = QGridLayout()
        self.joint_edits = []
        default_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] * 2
        for i in range(7):
            label = QLabel(f"L{i+1}:"); edit = QLineEdit(str(default_joints[i]))
            joint_grid_layout.addWidget(label, 0, i * 2); joint_grid_layout.addWidget(edit, 0, i * 2 + 1)
            self.joint_edits.append(edit)
        for i in range(7):
            label = QLabel(f"R{i+1}:"); edit = QLineEdit(str(default_joints[i+7]))
            joint_grid_layout.addWidget(label, 1, i * 2); joint_grid_layout.addWidget(edit, 1, i * 2 + 1)
            self.joint_edits.append(edit)
        self.joint_duration_edit = QLineEdit("3.0")
        joint_grid_layout.addWidget(QLabel("Duration (s):"), 2, 0); joint_grid_layout.addWidget(self.joint_duration_edit, 2, 1, 1, 3)
        self.joint_posture_button = QPushButton("Send Home Goal")
        self.joint_posture_button.setIcon(QIcon(home_img_path)); self.joint_posture_button.clicked.connect(self.on_joint_posture_clicked)
        joint_layout.addLayout(joint_grid_layout)
        joint_layout.addWidget(self.joint_posture_button)
        joint_groupbox.setLayout(joint_layout)

        # SE3
        se3_groupbox = QGroupBox("SE3 Goal")
        se3_layout = QVBoxLayout()
        left_group = QGroupBox("Left Target")
        left_grid = QGridLayout()
        self.se3_l_edits = [QLineEdit(v) for v in ["0.1", "0.0", "0.0", "0.0", "0.0", "0.0", "1.0"]]
        labels_l = ["P.X:", "P.Y:", "P.Z:", "Q.X:", "Q.Y:", "Q.Z:", "Q.W:"]
        for i, label in enumerate(labels_l):
            left_grid.addWidget(QLabel(label), i // 4, (i % 4) * 2)
            left_grid.addWidget(self.se3_l_edits[i], i // 4, (i % 4) * 2 + 1)
        left_group.setLayout(left_grid)
        right_group = QGroupBox("Right Target")
        right_grid = QGridLayout()
        self.se3_r_edits = [QLineEdit(v) for v in ["0.1", "0.0", "0.0", "0.0", "0.0", "0.0", "1.0"]]
        labels_r = ["P.X:", "P.Y:", "P.Z:", "Q.X:", "Q.Y:", "Q.Z:", "Q.W:"]
        for i, label in enumerate(labels_r):
            right_grid.addWidget(QLabel(label), i // 4, (i % 4) * 2)
            right_grid.addWidget(self.se3_r_edits[i], i // 4, (i % 4) * 2 + 1)
        right_group.setLayout(right_grid)
        se3_options_layout = QHBoxLayout()
        self.se3_duration_edit = QLineEdit("3.0")
        self.relative_checkbox = QCheckBox("Relative Mode")
        self.relative_checkbox.setChecked(True)
        se3_options_layout.addWidget(QLabel("Duration (s):")); se3_options_layout.addWidget(self.se3_duration_edit)
        se3_options_layout.addWidget(self.relative_checkbox)
        self.se3_button = QPushButton("Send Reach Goal")
        self.se3_button.setIcon(QIcon(se3_img_path))
        self.se3_button.clicked.connect(self.on_se3_clicked)
        se3_layout.addWidget(left_group)
        se3_layout.addWidget(right_group)
        se3_layout.addLayout(se3_options_layout)
        se3_layout.addWidget(self.se3_button)
        se3_groupbox.setLayout(se3_layout)

        # Feedback
        self.progress = QProgressBar(self)
        self.progress.setAlignment(Qt.AlignCenter)
        self.progress.setRange(0, 100); self.progress.setValue(0)
        self.progress.setStyleSheet(PROGRESS_BAR_STYLE_NORMAL)
        self.progress.setFormat('%p%')

        main_layout = QVBoxLayout()
        main_layout.addWidget(joint_groupbox)
        main_layout.addWidget(se3_groupbox)
        main_layout.addWidget(self.progress)
        self.setLayout(main_layout)

        # Client
        self.se3_client = ActionClient(self, SE3, "rci_h12_server/se3_server")
        self.joint_posture_client = ActionClient(self, JointPosture, "rci_h12_server/joint_posture_server")
        self.ros_timer = QTimer(self); self.ros_timer.timeout.connect(self.spin_ros); self.ros_timer.start(10)
        self.get_logger().info("Waiting for action servers..."); self.se3_client.wait_for_server(); self.joint_posture_client.wait_for_server()
        self.get_logger().info("All action servers are available.")

    def reset_progress_bar(self):
        self.progress.setValue(0)
        self.progress.setStyleSheet(PROGRESS_BAR_STYLE_NORMAL)
        self.progress.setFormat('%p%')

    def spin_ros(self): rclpy.spin_once(self, timeout_sec=0)
    def feedback_callback(self, feedback_msg):
        self.progress.setValue(int(round(feedback_msg.feedback.progress)))

    # SE3 Button
    def on_se3_clicked(self):
        self.reset_progress_bar()

        try:
            l_values = [float(edit.text()) for edit in self.se3_l_edits]
            goal_target_l = Pose(); goal_target_l.position.x, goal_target_l.position.y, goal_target_l.position.z = l_values[0:3]; goal_target_l.orientation.x, goal_target_l.orientation.y, goal_target_l.orientation.z, goal_target_l.orientation.w = l_values[3:7]
            r_values = [float(edit.text()) for edit in self.se3_r_edits]
            goal_target_r = Pose(); goal_target_r.position.x, goal_target_r.position.y, goal_target_r.position.z = r_values[0:3]; goal_target_r.orientation.x, goal_target_r.orientation.y, goal_target_r.orientation.z, goal_target_r.orientation.w = r_values[3:7]
            duration = float(self.se3_duration_edit.text())
        except ValueError: self.get_logger().error("Invalid number format in SE3 inputs."); return
        goal_msgs = SE3.Goal(); goal_targets = PoseArray(); goal_targets.poses.extend([goal_target_l, goal_target_r])
        goal_msgs.target, goal_msgs.duration, goal_msgs.relative = goal_targets, duration, self.relative_checkbox.isChecked()
        log_msg = f"Sending SE3 goal with 2 poses (Duration: {duration}s, Relative: {goal_msgs.relative})..."
        self.get_logger().info(log_msg)
        self._send_goal_future = self.se3_client.send_goal_async(goal_msgs, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.se3_goal_response_callback)

    def se3_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted: self.get_logger().info('SE3 Goal rejected'); return
        self.get_logger().info('SE3 Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.se3_get_result_callback)

    def se3_get_result_callback(self, future):
        result = future.result().result
        log_msg = f'SE3 Result: {"Success" if result.success else "Failed"}'
        self.get_logger().info(log_msg)
        if result.success:
            self.progress.setValue(100)
        else:
            self.progress.setValue(100)
            self.progress.setStyleSheet(PROGRESS_BAR_STYLE_FAIL)
            self.progress.setFormat("Fail!")

    # --- Joint Posture 액션 메소드들 ---
    def on_joint_posture_clicked(self):
        self.reset_progress_bar() 
        try:
            joint_values = [float(edit.text()) for edit in self.joint_edits]
            duration = float(self.joint_duration_edit.text())
        except ValueError: self.get_logger().error("Invalid number format in joint target or duration inputs."); return
        goal_msgs = JointPosture.Goal()
        goal_msgs.target, goal_msgs.duration = joint_values, duration
        self.get_logger().info(f"Sending JointPosture goal with 14 joints (Duration: {duration}s)...")
        self._send_goal_future = self.joint_posture_client.send_goal_async(goal_msgs, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.joint_posture_goal_response_callback)

    def joint_posture_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted: self.get_logger().info('JointPosture Goal rejected'); return
        self.get_logger().info('JointPosture Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.joint_posture_get_result_callback)

    def joint_posture_get_result_callback(self, future):
        result = future.result().result
        log_msg = f'JointPosture Result: {"Success" if result.success else "Failed"}'
        self.get_logger().info(log_msg)
        if result.success:
            self.progress.setValue(100)
        else:
            self.progress.setValue(100)
            self.progress.setStyleSheet(PROGRESS_BAR_STYLE_FAIL)
            self.progress.setFormat("Fail!")

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    demo = SimplifiedActionClientGUI()
    demo.show()
    app.exec_()
    demo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()