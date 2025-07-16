#pragma once

// RCLCPP
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

// ROS MSGS
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

// Eigen : Linear Algebra
#include <Eigen/QR>
#include <Eigen/Dense>

// Logger
#include "rci_h12_controller/utils/logger.hpp"

// RobotWrapper
#include "rci_h12_controller/robot/robot_wrapper.hpp"

// IK Solver
#include "rci_h12_controller/solver/ik_solver.hpp"

// Math
#include "rci_h12_controller/utils/math.hpp"

// Trajectory
#include "rci_h12_controller/trajectory/se3_traj.hpp"

// Controller
#include "rci_h12_controller/controller/controller.hpp"

namespace rci_simulator
{
    class H12_Simulator : public rclcpp::Node
    {
    public:
        H12_Simulator();
        std::shared_ptr<rci_robot::H12Wrapper> robot_;
        std::shared_ptr<rci_ik_solver::IkSolver> ik_solver_;
        std::shared_ptr<rci_trajectory::SE3CubicTrajectory> se3_traj_l_;
        std::shared_ptr<rci_trajectory::SE3CubicTrajectory> se3_traj_r_;
        std::shared_ptr<rci_controller::H12Controller> ctrl_;

        std::vector<std::string> joint_to_lock_list_ = {
            "left_hip_yaw_joint", "left_hip_pitch_joint", "left_hip_roll_joint",
            "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
            "right_hip_yaw_joint", "right_hip_pitch_joint", "right_hip_roll_joint",
            "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
            "torso_joint",
            "L_index_proximal_joint", "L_index_intermediate_joint",
            "L_middle_proximal_joint", "L_middle_intermediate_joint",
            "L_pinky_proximal_joint", "L_pinky_intermediate_joint",
            "L_ring_proximal_joint", "L_ring_intermediate_joint",
            "L_thumb_proximal_yaw_joint", "L_thumb_proximal_pitch_joint",
            "L_thumb_intermediate_joint", "L_thumb_distal_joint",
            "R_index_proximal_joint", "R_index_intermediate_joint",
            "R_middle_proximal_joint", "R_middle_intermediate_joint",
            "R_pinky_proximal_joint", "R_pinky_intermediate_joint",
            "R_ring_proximal_joint", "R_ring_intermediate_joint",
            "R_thumb_proximal_yaw_joint", "R_thumb_proximal_pitch_joint",
            "R_thumb_intermediate_joint", "R_thumb_distal_joint"
        };

    private:
        // Timer
        void ctrl_timer();
        rclcpp::TimerBase::SharedPtr timer_;

        // Pinocchio
        pinocchio::Model model_;
        pinocchio::Data data_;

        // ROS Publisher and Subscriber
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr ar_sensor_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

        // Robot Data
        Eigen::VectorXd q_, v_;
        
        // Target
        pinocchio::SE3 target_l_;
        pinocchio::SE3 target_r_;


        int nq_, nv_;
        bool init_;
                
        // Callback
        bool joint_callback_ok_;
        void joint_state_callback(sensor_msgs::msg::JointState::SharedPtr msg);
        void ar_sensor_callback(geometry_msgs::msg::PoseArray::SharedPtr msg);

        void initialize();
        void publish(std::vector<Eigen::VectorXd> qdes_list);

    };
}
