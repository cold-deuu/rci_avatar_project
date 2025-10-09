#pragma once

#include <memory>
#include <mutex>
#include <chrono>

// ROS2 and Action related headers
#include "action_interface/action/se3.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

// RCI_H12_CONTROLLER
#include "rci_h12_controller/server/server_base.hpp"


using namespace std::chrono_literals;

// Type Aliases for convenience
using SE3Action = action_interface::action::SE3;
using GoalHandleSE3 = rclcpp_action::ServerGoalHandle<SE3Action>;

class SE3ActionServer : ActionServerBase
{
    private:
        
        // SERVER
        rclcpp_action::Server<SE3Action>::SharedPtr action_server_;

        // SERVER COMPONENTS
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const SE3Action::Goal> goal);
        void handle_accepted(const std::shared_ptr<GoalHandleSE3> goal_handle);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleSE3> goal_handle);

        std::mutex state_mutex_;

        std::shared_ptr<GoalHandleSE3> current_goal_handle_;
    
        // For Feedback
        Eigen::VectorXd err_;
        std::vector<pinocchio::SE3> init_oMi_;

    public:
        SE3ActionServer(std::string name, rclcpp::Node::SharedPtr node_ptr, std::shared_ptr<rci_controller::H12Controller> ctrl);
        void compute();
};