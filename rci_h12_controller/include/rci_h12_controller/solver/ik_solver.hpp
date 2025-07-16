#pragma once
// ROS
#include "rclcpp/rclcpp.hpp"


// Math : Pinv
#include "rci_h12_controller/utils/math.hpp"

// Logger
#include "rci_h12_controller/utils/logger.hpp"

// Pinocchio : SE3
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/se3-tpl.hpp>
#include <pinocchio/spatial/log.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
// CPP
#include <vector>
#include <cassert>


using namespace rci_utils;

namespace rci_ik_solver
{
    class IkSolver
    {
        public:
            IkSolver(int nq, int nv, double dt);
            ~IkSolver(){};

            void update_parameter(std::vector<Eigen::MatrixXd> jacobs, Eigen::VectorXd q, Eigen::VectorXd v);
            // Eigen::VectorXd compute_ik(pinocchio::SE3 target, pinocchio::SE3 oMi);
            std::vector<Eigen::VectorXd> compute_ikvel(std::vector<pinocchio::SE3> target, std::vector<pinocchio::SE3> oMi);

        private:
            // Parameters
            std::vector<Eigen::MatrixXd> jacob_list_;
            std::vector<Eigen::VectorXd> q_list_, v_list_;
            int nq_, nv_;

            // Gains
            Eigen::VectorXd Kp_, Kd_;

            // Loop
            double dt_;

            // Logger
            rclcpp::Logger logger_ = rclcpp::get_logger("ik_solver");
            
    };
}