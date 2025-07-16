#pragma once

#include <Eigen/QR>    
#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>
#include "rclcpp/rclcpp.hpp"

using namespace Eigen;
using namespace std;
using namespace pinocchio;



namespace rci_trajectory{
    class SE3CubicTrajectory{
        public:
        SE3CubicTrajectory(const std::string & filename);
        rclcpp::Time m_stime, m_ctime;
        rclcpp::Duration m_duration= rclcpp::Duration::from_seconds(0.0);
        
        SE3 m_init, m_goal;

        //function
        void SetCurrentTime(rclcpp::Time ctime);
        void SetStartTime(rclcpp::Time stime);
        void SetDuration(rclcpp::Duration duration);
        void SetInitSample(SE3 &init_sample);
        void SetGoalSample(SE3 &goal_sample);
        SE3 computeNext();

    };

}
