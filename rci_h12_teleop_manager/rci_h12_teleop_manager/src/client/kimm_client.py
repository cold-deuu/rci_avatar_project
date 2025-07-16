# ROS2
import rclpy

# ROS2 - Action
from rclpy.action import ActionClient
from rclpy.node import Node

# ROS2 - Message
from geometry_msgs.msg import Pose

# Action File
from action_interface.action import SE3, JointPosture



# CMD Shell
import cmd

# PyQt GUI
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QProgressBar, QPushButton, QHBoxLayout
from PyQt5.QtCore import Qt, QTimer, QSize
from PyQt5.QtGui import QIcon


class ProgressBarDemo(Node, QWidget):
    """
    ---------------------------------------
    Action SE3
    ## GOAL ##
    target : geometry_msgs.msg.Pose
    duration : float64

    ## RESULT ##
    progress : float64 (percent of progress)

    ## FEEDBACK ##
    success : bool 
    final_position : geometry_msgs.msg.Pose

    ---------------------------------------
    Action JointPosture
    ## GOAL ##
    target : float64[]
    duration : float64

    ## RESULT ##
    progress : float64 (percent of progress)

    ## FEEDBACK ##
    success : bool 
    final_position : geometry_msgs.msg.Pose
    """
    def __init__(self):
        rclpy.node.Node.__init__(self, "rb5_action_client", namespace = "rb5_action_manager")
        QWidget.__init__(self)

        # Imanges
        se3_img_path = "/home/chan/kimm_ws/src/rb5_action_manager/gui_images/reach.png"
        home_img_path = "/home/chan/kimm_ws/src/rb5_action_manager/gui_images/home.png"


        self.setWindowTitle("Joint Progress")
        self.setGeometry(100, 100, 300, 100)

        self.progress = QProgressBar(self)
        self.progress.setAlignment(Qt.AlignCenter)
        self.progress.setRange(0, 100)  # 0~100% 범위
        self.progress.setValue(0)


        # Button
        self.send_button = QPushButton()
        self.send_button.setIcon(QIcon(home_img_path))
        self.send_button.clicked.connect(self.on_send_goal_clicked)
        self.send_button.setIconSize(QSize(32, 32))  
        self.send_button.setToolTip("Reach")  


        self.se3_button = QPushButton()
        self.se3_button.setIcon(QIcon(se3_img_path))
        self.se3_button.setIconSize(QSize(32, 32))  
        self.se3_button.setToolTip("Reach")  
        self.se3_button.clicked.connect(self.on_se3_clicked)

        button_layout = QHBoxLayout()
        button_layout.addWidget(self.send_button)
        button_layout.addWidget(self.se3_button)

        main_layout = QVBoxLayout()
        main_layout.addWidget(self.progress)
        main_layout.addLayout(button_layout)
        self.setLayout(main_layout)

        # Client
        self.JointPosture_client = ActionClient(self, JointPosture, "rci_rb5_control/joint_controller")
        self.JointPosture_client.wait_for_server()
        self.client = ActionClient(self, SE3, "rci_rb5_control/ik_controller")
        self.client.wait_for_server()

    def feedback_callback(self, feedback_msg):
        self.progress.setValue(int(round(feedback_msg.feedback.progress)))
        self.get_logger().info(f"[Feedback] progress: {feedback_msg.feedback.progress:.2f}%")

    def on_send_goal_clicked(self):
        self.progress.setValue(0)

        goal_msgs = JointPosture.Goal()
        goal_duration : float

        # Target Joint Value
        goal_target = [0.0, 1.0, -1.0, 1.0, 0.0, 0.0]

        # Target Duration
        goal_duration = 3.0

        goal_msgs.target = goal_target
        goal_msgs.duration = goal_duration


        # Client : Send Goal
        # Future --> 미래에 값을 저장하기 위해 비워둠
        # 만약 미래에 값을 전달 받으면 그 값으로 대체
        self._send_goal_future = self.JointPosture_client.send_goal_async(goal_msgs, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, self._send_goal_future)
        goal_handle = self._send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        # Client : Waif for Result
        future_result = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, future_result)
        result = future_result.result().result
        if result.success:
            self.progress.setValue(int(100))

        self.get_logger().info(f'Result: {result}')

    def on_se3_clicked(self):
        self.progress.setValue(0)

        goal_msgs = SE3.Goal()

        
        goal_target = Pose()
        goal_duration : float

        # Action
        # Target Translation
        goal_target.position.x = 0.0
        goal_target.position.y = 0.0
        goal_target.position.z = -0.2

        # Target Orientation
        goal_target.orientation.x = 0.0
        goal_target.orientation.y = 0.0
        goal_target.orientation.z = 0.0
        goal_target.orientation.w = 1.0

        # Target Duration
        goal_duration = 3.0

        goal_msgs.target = goal_target
        goal_msgs.duration = goal_duration


        # Client : Send Goal
        self._send_goal_future = self.client.send_goal_async(goal_msgs, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, self._send_goal_future)
        goal_handle = self._send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        # Client : Waif for Result
        future_result = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, future_result)
        result = future_result.result().result
        if result.success:
            self.progress.setValue(int(100))

        self.get_logger().info(f'Result: {result}')




    def do_test(self, arg):
        goal_msgs = SE3.Goal()

        
        goal_target = Pose()
        goal_duration : float

        # Action
        # Target Translation
        goal_target.position.x = 0.0
        goal_target.position.y = 0.0
        goal_target.position.z = 0.0

        # Target Orientation
        goal_target.orientation.x = 0.0
        goal_target.orientation.y = 0.0
        goal_target.orientation.z = 0.0
        goal_target.orientation.w = 1.0

        # Target Duration
        goal_duration = 3.0


        goal_msgs.target = goal_target
        goal_msgs.duration = goal_duration


        self._send_goal_future = self.client.send_goal_async(goal_msgs)


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class Client(cmd.Cmd, Node):
    """
    ---------------------------------------
    Action SE3
    ## GOAL ##
    target : geometry_msgs.msg.Pose
    duration : float64

    ## RESULT ##
    progress : float64 (percent of progress)

    ## FEEDBACK ##
    success : bool 
    final_position : geometry_msgs.msg.Pose

    ---------------------------------------
    Action JointPosture
    ## GOAL ##
    target : float64[]
    duration : float64

    ## RESULT ##
    progress : float64 (percent of progress)

    ## FEEDBACK ##
    success : bool 
    final_position : geometry_msgs.msg.Pose
    """
    intro = bcolors.OKBLUE + "Welcome to the control suite shell.\nType help or ? to list commands.\n" + bcolors.ENDC
    prompt = "(csuite) "

    def __init__(self):
        cmd.Cmd.__init__(self)
        rclpy.node.Node.__init__(self, "rb5_action_client", namespace = "rb5_action_manager")
        
       # Node.__init__("rb5_GUI_Controller")

        # self.client = ActionClient(self, SE3, "rci_rb5_control/ik_controller")
        # self.client.wait_for_server()
        
        self.JointPosture_client = ActionClient(self, JointPosture, "rci_rb5_control/joint_controller")
        self.JointPosture_client.wait_for_server()

    def do_test(self, arg):
        goal_msgs = SE3.Goal()

        
        goal_target = Pose()
        goal_duration : float

        # Action
        # Target Translation
        goal_target.position.x = 0.0
        goal_target.position.y = 0.0
        goal_target.position.z = 0.0

        # Target Orientation
        goal_target.orientation.x = 0.0
        goal_target.orientation.y = 0.0
        goal_target.orientation.z = 0.0
        goal_target.orientation.w = 1.0

        # Target Duration
        goal_duration = 3.0


        goal_msgs.target = goal_target
        goal_msgs.duration = goal_duration


        self._send_goal_future = self.client.send_goal_async(goal_msgs)

    def do_test2(self, arg):
        goal_msgs = JointPosture.Goal()

        
        goal_duration : float

        # Target Joint Value
        goal_target = [0.0, 0.0, -1.0, -1.0, 0.0, 0.0]
        # Target Duration
        goal_duration = 3.0


        goal_msgs.target = goal_target
        goal_msgs.duration = goal_duration


        self._send_goal_future = self.JointPosture_client.send_goal_async(goal_msgs, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, self._send_goal_future)
        goal_handle = self._send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        future_result = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, future_result)
        result = future_result.result().result
        self.get_logger().info(f'Result: {result}')

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"[Feedback] progress: {feedback_msg.feedback.progress:.2f}%")


def main(args = None):
    # rclpy.init(args=args)
    # client = Client()
    # client.cmdloop()

    import sys
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    demo = ProgressBarDemo()
    demo.show()
    sys.exit(app.exec_())

# if __name__ == '__main__':
#     import sys
#     app = QApplication(sys.argv)
#     demo = ProgressBarDemo()
#     demo.show()
#     sys.exit(app.exec_())