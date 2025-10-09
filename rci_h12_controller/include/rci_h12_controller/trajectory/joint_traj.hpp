#pragma once

#include <Eigen/QR>    
#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <rclcpp/rclcpp.hpp>
#include "rci_h12_controller/utils/math.hpp"

namespace rci_trajectory{
    class JointCubicTrajectory{
        public:
            JointCubicTrajectory(const std::string & filename, int armjoint_num);
            rclcpp::Time m_stime, m_ctime;
            rclcpp::Duration m_duration= rclcpp::Duration::from_seconds(0.0);
            
            Eigen::VectorXd m_init, m_goal;

            //function
            void SetCurrentTime(rclcpp::Time ctime);
            void SetStartTime(rclcpp::Time stime);
            void SetDuration(rclcpp::Duration duration);
            void SetInitSample(Eigen::VectorXd init_sample);
            void SetGoalSample(Eigen::VectorXd goal_sample);

            Eigen::VectorXd GetGoalSample()
            {
                return m_goal;
            }

            Eigen::VectorXd computeNext();

        protected:
            int m_na;
    };

}
