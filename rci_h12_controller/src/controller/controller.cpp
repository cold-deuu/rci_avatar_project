#include "rci_h12_controller/controller/controller.hpp"

namespace rci_controller
{
    H12Controller::H12Controller(rclcpp::Clock::SharedPtr clock, std::shared_ptr<rci_robot::H12Wrapper> robot, bool is_reduced):
     clock_(clock), robot_(robot)
    {
        model_ = robot_->model(is_reduced);
        data_ = robot_->data(is_reduced);

        std::vector<std::string> dual_eef = {"left_wrist_yaw_joint", "right_wrist_yaw_joint"};
        robot_->regist_dual_eef(dual_eef);

        // Solver
        ik_solver_ = std::make_shared<rci_ik_solver::IkSolver>(model_.nq, model_.nv, 0.01);

        // Trajectory
        se3_traj_l_ = std::make_shared<rci_trajectory::SE3CubicTrajectory>("Cubic_SE3_Traj_left");
        se3_traj_r_ = std::make_shared<rci_trajectory::SE3CubicTrajectory>("Cubic_SE3_Traj_right");

        q_.setZero(model_.nq); v_.setZero(model_.nq);
        traj_init_ = true;
    }

    void H12Controller::update_controller(Eigen::VectorXd q, Eigen::VectorXd v)
    {
        q_ = q;
        v_ = v;

        robot_->compute_all_terms(data_, q, v, true);
        
        // dual_oMi_ = robot_->compute_dual_oMi(data_, model_, true);
        // TEST
        dual_oMi_ = robot_->compute_dual_oMi(data_, model_, false);
        dual_jacob_ = robot_->compute_dual_jacobian_local(data_, model_);
    }

    std::pair<std::vector<Eigen::VectorXd>, bool> H12Controller::init_teleop(std::vector<pinocchio::SE3> target)
    {
        assert((target.size() == 1 || target.size() == 2) && "target must be single/dual");
        if (traj_init_)
        {
            pinocchio::SE3 goal_l = target[0];
            pinocchio::SE3 goal_r = target[1];
            rclcpp::Duration duration = rclcpp::Duration::from_seconds(1.0);

            se3_traj_l_ -> SetStartTime(clock_->now());
            se3_traj_l_ -> SetDuration(duration);
            se3_traj_l_ -> SetGoalSample(goal_l);
            se3_traj_l_ -> SetInitSample(dual_oMi_[0]);

            se3_traj_r_ -> SetStartTime(clock_->now());
            se3_traj_r_ -> SetDuration(duration);
            se3_traj_r_ -> SetGoalSample(goal_r);
            se3_traj_r_ -> SetInitSample(dual_oMi_[1]);
            traj_init_ = false;
        }
        se3_traj_l_ -> SetCurrentTime(clock_->now());
        se3_traj_r_ -> SetCurrentTime(clock_->now());
        pinocchio::SE3 target_l = se3_traj_l_ -> computeNext();
        pinocchio::SE3 target_r = se3_traj_r_ -> computeNext();

        std::vector<pinocchio::SE3> c_target = {target_l, target_r};

        ik_solver_->update_parameter(dual_jacob_, q_, v_);
        auto qdes_list = ik_solver_ -> compute_ikvel(c_target, dual_oMi_);
        bool is_finished = rci_utils::check_reach(target, dual_oMi_);

        if(is_finished)
            traj_init_ = true;
        return std::make_pair(qdes_list, is_finished);
        
    }

}