#include "rci_h12_controller/simulator/simulator.hpp"

pinocchio::SE3 goal_l, goal_r;

namespace rci_simulator
{
    H12_Simulator::H12_Simulator()
    : Node("rci_h12_simulator") 
    {
        // Robot Path
        std::string pkg_path = ament_index_cpp::get_package_share_directory("h1_2_description");
        std::string urdf_path = "/h1_2.urdf";

        std::vector<std::string> pkg_dir;
        pkg_dir.push_back(pkg_path);
        std::string urdf_file_name = pkg_dir[0] + urdf_path;

        // Robot Wrapper
        robot_ = std::make_shared<rci_robot::H12Wrapper>(urdf_file_name, pkg_dir, joint_to_lock_list_);
        nq_ = robot_->nq(true);
        nv_ = robot_->nv(true);

        // Controller
        ctrl_ = std::make_shared<rci_controller::H12Controller>(this->get_clock(), robot_, true);

        // 10ms == 100Hz
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&H12_Simulator::ctrl_timer, this));

        // Publisher
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/mujoco_ctrl", 10);

        // Subscirber 
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/mujoco/joint_states", 10,std::bind(&H12_Simulator::joint_state_callback, this, std::placeholders::_1));
        ar_sensor_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/rci_h12_manager/ar_sensor_pose", 10,std::bind(&H12_Simulator::ar_sensor_callback, this, std::placeholders::_1));
        
        initialize();


        // Test
        init_ = true;


    }

    void H12_Simulator::ctrl_timer()
    {
        if(joint_callback_ok_)
        {
            ctrl_->update_controller(q_,v_);
            auto dual_oMi = ctrl_->get_dual_oMi();
            if(init_)
            {
                target_l_ = dual_oMi[0];
                target_r_ = dual_oMi[1];
                target_l_.translation()(0) += 0.1;
                target_r_.translation()(2) += 0.1;    
                init_ = false;
            }

            std::vector<pinocchio::SE3> target_list = {target_l_, target_r_};
            auto [qdes_list, ok] = ctrl_->init_teleop(target_list);

            this->publish(qdes_list);
            if(ok)
            {
                RCLCPP_INFO(this->get_logger(), "Control Finished!");
                RCLCPP_INFO(this->get_logger(), "Node shutdown");
                timer_->cancel();
                rclcpp::shutdown();
                return;
            }
            
        }
    }

    void H12_Simulator::joint_state_callback(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        joint_callback_ok_ = true;
        std::copy(msg->position.begin(), msg->position.begin() + nq_, q_.data());
        std::copy(msg->velocity.begin(), msg->velocity.begin() + nv_, v_.data());
        
    }

    void H12_Simulator::ar_sensor_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        // geometry_msgs::msg::Pose head = msg->poses[0];
        // geometry_msgs::msg::Pose l_wrist = msg->poses[1];
        // geometry_msgs::msg::Pose r_wrist = msg->poses[2];
        
        pinocchio::SE3 head_se3 = pose_msgs_to_se3(msg->poses[0]);
        pinocchio::SE3 lwrist_se3 = pose_msgs_to_se3(msg->poses[1]);
        pinocchio::SE3 rwrist_se3 = pose_msgs_to_se3(msg->poses[2]);
        
        target_l_ = head_se3.inverse() * lwrist_se3;
        target_r_ = head_se3.inverse() * rwrist_se3;
    }

    void H12_Simulator::initialize()
    {
        q_.setZero(14);
        v_.setZero(14);
        joint_callback_ok_ = false;
    }

    void H12_Simulator::publish(std::vector<Eigen::VectorXd> qdes_list)
    {
        assert((qdes_list.size() == 1 || qdes_list.size() == 2) && "robot must be single/dual arm");
        int nq_arm = (qdes_list.size() == 1) ? nq_ : nq_ / 2;
        
        std_msgs::msg::Float64MultiArray msg;
        msg.data.resize(nq_);

        std::copy(qdes_list[0].data(), qdes_list[0].data() + nq_arm, msg.data.begin());
        if (qdes_list.size() == 2) 
        {
            std::copy(qdes_list[1].data(), qdes_list[1].data() + nq_arm, msg.data.begin() + nq_arm);
        }

        publisher_->publish(msg);
    }


}
