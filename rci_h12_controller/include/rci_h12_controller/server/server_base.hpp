#pragma once

#include <memory>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// RCI_H12_CONTROLLER
#include "rci_h12_controller/controller/controller.hpp"

class ActionServerBase
{
    private:
        std::string action_name_;
        

        bool is_running()
        {
            return is_running_;
        }

    public:
        // Initialize
        ActionServerBase(std::string name, rclcpp::Node::SharedPtr node_ptr, std::shared_ptr<rci_controller::H12Controller> ctrl):
        action_name_(name), node_(node_ptr), ctrl_(ctrl)
        {}    

        virtual void compute() = 0;


        std::shared_ptr<rci_controller::H12Controller> ctrl_;

        rclcpp::Node::SharedPtr node_;
        bool is_running_{false};

        rclcpp::Time stime_;
        double duration_;
        
        
};