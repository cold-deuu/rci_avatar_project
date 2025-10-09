#pragma once

#include <memory>
#include <mutex>
#include <chrono>

// ROS2 and Action related headers
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_interface/action/se3.hpp"
#include "geometry_msgs/msg/pose.hpp"


using namespace std::chrono_literals;

// Type Aliases for convenience
using SE3Action = action_interface::action::SE3;
using GoalHandleSE3 = rclcpp_action::ServerGoalHandle<SE3Action>;

class ControllerActionServer : public rclcpp::Node
{
public:
    ControllerActionServer() : Node("controller_action_server_node")
    {
        // Controller 인스턴스 생성
        controller_ = std::make_unique<DummyController>();

        // Action Server 생성 및 콜백 함수 바인딩
        action_server_ = rclcpp_action::create_server<SE3Action>(
            this,
            "rci_rb5_control/ik_controller",
            std::bind(&ControllerActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ControllerActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ControllerActionServer::handle_accepted, this, std::placeholders::_1)
        );

        // 주기적인 compute() 함수를 위한 타이머 생성
        compute_timer_ = this->create_wall_timer(
            10ms, // 100Hz control loop
            std::bind(&ControllerActionServer::compute, this)
        );

        RCLCPP_INFO(this->get_logger(), "Controller Action Server has been started.");
    }

private:
    // ROS2 & Action Server members
    rclcpp_action::ActionServer<SE3Action>::SharedPtr action_server_;
    rclcpp::TimerBase::SharedPtr compute_timer_;
    std::shared_ptr<GoalHandleSE3> current_goal_handle_;
    
    // State management members
    bool is_running_ = false;
    std::mutex state_mutex_;

    // Controller and Robot specific members
    std::unique_ptr<DummyController> controller_;
    pinocchio::SE3 oMi_0_;
    pinocchio::Motion initErr_;
    double duration_;
    rclcpp::Time stime_;

    // Python의 goalCB를 C++로 변환한 함수
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const SE3Action::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received new goal request. Performing pre-computation...");
        (void)uuid;

        std::lock_guard<std::mutex> lock(state_mutex_);
        
        // self.oMi_0 = deepcopy(self.controller.robot.state.oMi)
        oMi_0_ = controller_->robot.state.oMi;

        // Target XYZ QUAT from geometry_msgs::msg::Pose
        const auto& target_pose = goal->target;
        Eigen::Vector3d target_position(target_pose.position.x, target_pose.position.y, target_pose.position.z);
        Eigen::Quaterniond target_orientation(target_pose.orientation.w, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z);
        
        pinocchio::SE3 targetSE3(target_orientation, target_position);

        // QUAT To SE3 and apply relative transform
        targetSE3.translation() = oMi_0_.translation() + targetSE3.translation();
        targetSE3.rotation() = oMi_0_.rotation() * targetSE3.rotation();
        
        RCLCPP_INFO_STREAM(this->get_logger(), "Computed Target SE3:\n" << targetSE3);

        // Target Duration
        duration_ = goal->duration;

        // Control Setting
        controller_->initSE3(targetSE3, duration_);

        // Timer
        stime_ = this->get_clock()->now();

        // For Feedback
        initErr_ = pinocchio::log6(oMi_0_.inverse() * targetSE3);

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // 목표가 수락된 직후 호출되는 콜백
    void handle_accepted(const std::shared_ptr<GoalHandleSE3> goal_handle)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        // 이전 작업이 있다면 중단 (간단한 예시)
        if (is_running_) {
            current_goal_handle_->abort(std::make_shared<SE3Action::Result>());
        }
        
        current_goal_handle_ = goal_handle;
        is_running_ = true;
    }

    // 취소 콜백
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleSE3> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        // 취소 요청 시 루프를 중단하도록 플래그를 false로 설정
        std::lock_guard<std::mutex> lock(state_mutex_);
        is_running_ = false;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // 주기적인 제어 루프 (Python의 compute)
    void compute()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!is_running_ || !current_goal_handle_) {
            return;
        }
        // ... Python의 compute()와 동일한 제어/피드백/종료 로직 구현 ...
    }
};
