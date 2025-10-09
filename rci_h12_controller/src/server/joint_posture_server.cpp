#include "rci_h12_controller/server/joint_posture_server.hpp"

using namespace std::chrono_literals;
using JointPostureAction = action_interface::action::JointPosture;
using GoalHandleJointPosture = rclcpp_action::ServerGoalHandle<JointPostureAction>;

#define ANSI_COLOR_BLUE    "\033[94m"
#define ANSI_COLOR_RED     "\033[91m"
#define ANSI_COLOR_RESET   "\033[0m"

JointPostureActionServer::JointPostureActionServer(std::string name, rclcpp::Node::SharedPtr node_ptr, std::shared_ptr<rci_controller::H12Controller> ctrl)
:ActionServerBase(name, node_ptr, ctrl)
{
    action_server_ = rclcpp_action::create_server<JointPostureAction>(
        node_,
        "rci_h12_server/joint_posture_server",
        std::bind(&JointPostureActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&JointPostureActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&JointPostureActionServer::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(node_->get_logger(), "[Joint Posture Server] Server was Registered");

}

rclcpp_action::GoalResponse JointPostureActionServer::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const JointPostureAction::Goal> goal)
{
    (void)uuid;

    std::lock_guard<std::mutex> lock(state_mutex_);
    
    const auto& target_pose = goal -> target;
    Eigen::Map<const Eigen::VectorXd> target_joint(target_pose.data(), target_pose.size());
    
    rclcpp::Duration duration = rclcpp::Duration::from_seconds(goal->duration);
    stime_ = node_->now();
    duration_ = duration.seconds();
    
    ctrl_->init_joint_posture(target_joint, node_->now(), duration);

    RCLCPP_INFO(node_->get_logger(), "[Joint Posture Server] Goal was Received");

    // Compute Feedback
    init_q_ = ctrl_->q_;
    err_ = (target_joint - init_q_).norm();

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


void JointPostureActionServer::handle_accepted(const std::shared_ptr<GoalHandleJointPosture> goal_handle)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (is_running_) {
        current_goal_handle_->abort(std::make_shared<JointPostureAction::Result>());
    }
    
    current_goal_handle_ = goal_handle;
    is_running_ = true;
}

rclcpp_action::CancelResponse JointPostureActionServer::handle_cancel(const std::shared_ptr<GoalHandleJointPosture> goal_handle)
{
    RCLCPP_INFO(node_->get_logger(), "[Joint Posture Server] Received request to cancel goal");
    (void)goal_handle;

    std::lock_guard<std::mutex> lock(state_mutex_);
    is_running_ = false;
    current_goal_handle_.reset();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void JointPostureActionServer::compute()
{
    if(!is_running_)
    {
        return;
    }

    auto feedback = std::make_shared<JointPostureAction::Feedback>();
    feedback->progress = 100 * ((ctrl_->q_ - init_q_).norm()/err_);
    current_goal_handle_ -> publish_feedback(feedback);
    auto ok = ctrl_->joint_posture(node_->now());
    auto result = std::make_shared<JointPostureAction::Result>();

    double stime = static_cast<double>(stime_.nanoseconds()) * 1e-9;
    double ctime = static_cast<double>(node_->now().nanoseconds()) * 1e-9;

    if (ctime-stime < duration_ + 1.0 && ok)
    {
        RCLCPP_INFO(node_->get_logger(), "[Joint Posture Server] " ANSI_COLOR_BLUE "Action Success");
        result -> success = true;

        current_goal_handle_ -> succeed(result);
        is_running_ = false;
        current_goal_handle_.reset();        
    }

    if (ctime-stime > duration_ + 2.0)
    {
        RCLCPP_WARN(node_->get_logger(), "[Joint Posture Server] " ANSI_COLOR_RED "Action Failed");
        result -> success = false;

        current_goal_handle_ -> abort(result);
        is_running_ = false;
        current_goal_handle_.reset(); 
    }

}