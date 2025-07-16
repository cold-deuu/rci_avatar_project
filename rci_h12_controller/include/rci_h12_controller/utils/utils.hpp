#pragma once

#include <sstream>
#include <string>
#include <vector>

// Pinocchio
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/se3-tpl.hpp>
#include <pinocchio/spatial/log.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

// Eigen
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/QR> 

// ROS 
#include <rclcpp/rclcpp.hpp>

// ROS MSGS
#include "geometry_msgs/msg/pose.hpp"

namespace rci_utils
{
    inline bool check_reach(std::vector<pinocchio::SE3> target_list, std::vector<pinocchio::SE3> oMi_list)
    {
        pinocchio::SE3 target_l = target_list[0];
        pinocchio::SE3 target_r = target_list[1];
        pinocchio::SE3 oMi_l = oMi_list[0];
        pinocchio::SE3 oMi_r = oMi_list[1];

        if((pinocchio::log6(oMi_l.inverse() * target_l).toVector().norm())<1e-3
         && (pinocchio::log6(oMi_r.inverse() * target_r).toVector().norm())<1e-3)
        {
            return true;
        }
        else{
            std::cout<<"Error l : "<<pinocchio::log6(oMi_l.inverse() * target_l).toVector().norm()<<std::endl;
            std::cout<<"Error r : "<<pinocchio::log6(oMi_r.inverse() * target_r).toVector().norm()<<std::endl;
            return false;
        }
        
    }
}