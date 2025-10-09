#include "rci_h12_controller/server/se3_server.hpp"

using namespace std::chrono_literals;

// Type Aliases for convenience
using SE3Action = action_interface::action::SE3;
using GoalHandleSE3 = rclcpp_action::ServerGoalHandle<SE3Action>;

#define ANSI_COLOR_BLUE    "\033[94m"
#define ANSI_COLOR_RED     "\033[91m"
#define ANSI_COLOR_RESET   "\033[0m"

SE3ActionServer::SE3ActionServer(std::string name, rclcpp::Node::SharedPtr node_ptr, std::shared_ptr<rci_controller::H12Controller> ctrl)
:ActionServerBase(name, node_ptr, ctrl)
{
    action_server_ = rclcpp_action::create_server<SE3Action>(
        node_,
        "rci_h12_server/se3_server",
        std::bind(&SE3ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&SE3ActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&SE3ActionServer::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(node_->get_logger(), "[SE3 Server] Server was Registered");
}

rclcpp_action::GoalResponse SE3ActionServer::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const SE3Action::Goal> goal)
{
    (void)uuid;

    std::lock_guard<std::mutex> lock(state_mutex_);
    
    const auto& target_pose = goal -> target;
    bool is_rel = goal -> relative;
    
    Eigen::Vector3d target_position_l(target_pose.poses[0].position.x, target_pose.poses[0].position.y, target_pose.poses[0].position.z);
    Eigen::Quaterniond target_orientation_l(target_pose.poses[0].orientation.w, target_pose.poses[0].orientation.x, target_pose.poses[0].orientation.y, target_pose.poses[0].orientation.z);
    pinocchio::SE3 target_l(target_orientation_l, target_position_l);

    Eigen::Vector3d target_position_r(target_pose.poses[1].position.x, target_pose.poses[1].position.y, target_pose.poses[1].position.z);
    Eigen::Quaterniond target_orientation_r(target_pose.poses[1].orientation.w, target_pose.poses[1].orientation.x, target_pose.poses[1].orientation.y, target_pose.poses[1].orientation.z);
    pinocchio::SE3 target_r(target_orientation_r, target_position_r);


    rclcpp::Duration duration = rclcpp::Duration::from_seconds(goal->duration);
    stime_ = node_->now();
    duration_ = duration.seconds();

    init_oMi_ = ctrl_->get_dual_oMi();
    if(is_rel)
    {
        target_l.translation() += init_oMi_[0].translation();
        target_l.rotation() = init_oMi_[0].rotation() * target_l.rotation();

        target_r.translation() += init_oMi_[1].translation();
        target_r.rotation() = init_oMi_[1].rotation() * target_r.rotation();
    }

    std::vector<pinocchio::SE3> target_list = {target_l, target_r};
    ctrl_->init_control(target_list, is_rel, node_->now(), duration);

    RCLCPP_INFO(node_->get_logger(), "[SE3 Server] Goal was Received");

    // For Feedback
    err_.resize(12);
    pinocchio::Motion err_l = pinocchio::log6(init_oMi_[0].inverse() * target_l);
    pinocchio::Motion err_r = pinocchio::log6(init_oMi_[1].inverse() * target_r);
    err_.head(6) = err_l.toVector();
    err_.tail(6) = err_r.toVector();

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


// 목표가 수락된 직후 호출되는 콜백
void SE3ActionServer::handle_accepted(const std::shared_ptr<GoalHandleSE3> goal_handle)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (is_running_) {
        current_goal_handle_->abort(std::make_shared<SE3Action::Result>());
    }
    
    current_goal_handle_ = goal_handle;
    is_running_ = true;
}

rclcpp_action::CancelResponse SE3ActionServer::handle_cancel(const std::shared_ptr<GoalHandleSE3> goal_handle)
{
    RCLCPP_INFO(node_->get_logger(), "[SE3 Server] Received request to cancel goal");

    (void)goal_handle;

    std::lock_guard<std::mutex> lock(state_mutex_);
    is_running_ = false;
    current_goal_handle_.reset();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SE3ActionServer::compute()
{
    if(!is_running_)
    {
        return;
    }

    // Compute Feedback
    auto feedback = std::make_shared<SE3Action::Feedback>();
    Eigen::VectorXd current_err(12);
    pinocchio::Motion err_l = pinocchio::log6(init_oMi_[0].inverse() * ctrl_->get_dual_oMi()[0]);
    pinocchio::Motion err_r = pinocchio::log6(init_oMi_[1].inverse() * ctrl_->get_dual_oMi()[1]);
    current_err.head(6) = err_l.toVector();
    current_err.tail(6) = err_r.toVector();

    feedback->progress = 100 * (current_err.norm()/err_.norm());
    current_goal_handle_ -> publish_feedback(feedback);
    auto ok = ctrl_->control(node_->now());
    auto result = std::make_shared<SE3Action::Result>();

    double stime = static_cast<double>(stime_.nanoseconds()) * 1e-9;
    double ctime = static_cast<double>(node_->now().nanoseconds()) * 1e-9;

    if (ctime-stime < duration_ + 1.0 && ok)
    {
        RCLCPP_INFO(node_->get_logger(), "[SE3 Server] " ANSI_COLOR_BLUE "Action Success");
        result -> success = true;

        current_goal_handle_ -> succeed(result);
        is_running_ = false;
        current_goal_handle_.reset();        
    }

    if (ctime-stime > duration_ + 2.0)
    {
        RCLCPP_WARN(node_->get_logger(), "[SE3 Server] " ANSI_COLOR_RED "Action Failed");
        result -> success = false;

        current_goal_handle_ -> abort(result);
        is_running_ = false;
        current_goal_handle_.reset(); 
    }

}