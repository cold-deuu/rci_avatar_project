#pragma once

// ROS
#include "rclcpp/rclcpp.hpp"

// ROS MSGS
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

// RCI H12 WRAPPER
#include "rci_h12_controller/robot/robot_wrapper.hpp"

// RCI IK SOLVER
#include "rci_h12_controller/solver/ik_solver.hpp"

// RCI TRAJECTORY
#include "rci_h12_controller/trajectory/se3_traj.hpp"

// RCI UTILS
#include "rci_h12_controller/utils/logger.hpp"
#include "rci_h12_controller/utils/math.hpp"
#include "rci_h12_controller/utils/utils.hpp"

// C++
#include <vector>
#include <string>

namespace rci_controller
{
    class H12Controller
    {
        public:
            H12Controller(rclcpp::Clock::SharedPtr clock, std::shared_ptr<rci_robot::H12Wrapper> robot, bool is_reduced = false);
            ~H12Controller(){};
            
            rclcpp::Clock::SharedPtr clock_;
            std::shared_ptr<rci_robot::H12Wrapper> robot_;
            std::shared_ptr<rci_ik_solver::IkSolver> ik_solver_;
            std::shared_ptr<rci_trajectory::SE3CubicTrajectory> se3_traj_l_;
            std::shared_ptr<rci_trajectory::SE3CubicTrajectory> se3_traj_r_;

            std::vector<pinocchio::SE3> get_dual_oMi(){
                return dual_oMi_;
            }

            // Pinocchio
            pinocchio::Model model_;
            pinocchio::Data data_;

            // Clock
            void update_controller(Eigen::VectorXd q, Eigen::VectorXd v);
            std::pair<std::vector<Eigen::VectorXd>, bool> init_teleop(std::vector<pinocchio::SE3> target);


        private:

            std::vector<pinocchio::SE3> dual_oMi_;
            std::vector<Eigen::MatrixXd> dual_jacob_;
            Eigen::VectorXd q_, v_;
            bool traj_init_;

    };
}