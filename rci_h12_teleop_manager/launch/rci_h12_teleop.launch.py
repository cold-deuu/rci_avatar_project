from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rci_h12_teleop_manager',
            executable='unity_wrist_node',
            name='unity_wrist_node',
            output='screen',
        ),
        Node(
            package='rci_h12_teleop_manager',
            executable='rci_h12_teleop_arm_node',
            name='rci_h12_teleop_arm_node',
            output='screen',
        ),
        Node(
            package='rci_h12_teleop_manager',
            executable='rci_h12_teleop_hand_node',
            name='rci_h12_teleop_hand_node',
            output='screen',
        ),
    ])