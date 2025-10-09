#pragma once

#include <memory>
#include <mutex>
#include <chrono>

// ROS2 and Action related headers
#include "action_interface/action/joint_posture.hpp"
#include "geometry_msgs/msg/pose.hpp"


// RCI_H12_CONTROLLER
#include "rci_h12_controller/server/server_base.hpp"


using namespace std::chrono_literals;

// Type Aliases for convenience
using JointPostureAction = action_interface::action::JointPosture;
using GoalHandleJointPosture = rclcpp_action::ServerGoalHandle<JointPostureAction>;

class JointPostureActionServer : ActionServerBase
{
    private:
        
        // SERVER
        rclcpp_action::Server<JointPostureAction>::SharedPtr action_server_;

        // SERVER COMPONENTS
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const JointPostureAction::Goal> goal);
        void handle_accepted(const std::shared_ptr<GoalHandleJointPosture> goal_handle);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleJointPosture> goal_handle);

        std::mutex state_mutex_;

        std::shared_ptr<GoalHandleJointPosture> current_goal_handle_;
    
        double err_;
        Eigen::VectorXd init_q_;
    public:
        JointPostureActionServer(std::string name, rclcpp::Node::SharedPtr node_ptr, std::shared_ptr<rci_controller::H12Controller> ctrl);
        void compute();
};