#include "h12_bridge/node/state_publisher.hpp"
#include <memory>

using namespace unitree::common;
using namespace unitree::robot;

H12Bridge::H12Bridge(std::string networkInterface)
: Node("topic_subscriber"), network_interface_(networkInterface), control_dt_(0.002), time_(0.0), mode_(PR)
{
  h12_cmd_topic_ = "rt/lowcmd";
  h12_state_topic_ = "rt/lowstate";
  h12_num_motors_ = 27;
  duration_ = 1.0;

  subscription_ = this->create_subscription<unitree_hg::msg::LowState>(
    "/lowstate", 10, std::bind(&H12Bridge::topic_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Subscriber is ready. Waiting for messages...");

  // Test
  // timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&H12Bridge::ctrl_timer, this));
  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/rci_h12_manager/real/joint_states", 10);

}

void H12Bridge::ctrl_timer()
{
  std::cout<<"LeftWristRoll :"<<H1JointIndex::LeftWristRoll<<std::endl;
}

void H12Bridge::Control_H12()
{
  unitree_hg::msg::dds_::LowCmd_ dds_low_command;
  dds_low_command.mode_pr() = mode_;
  dds_low_command.mode_machine() = low_state_.mode_machine();
  for (int i = 0; i < h12_num_motors_; ++i) {
    dds_low_command.motor_cmd().at(i).mode() = 1;  // 1:Enable, 0:Disable
    dds_low_command.motor_cmd().at(i).tau() = 0.0;
    dds_low_command.motor_cmd().at(i).q() = 0.0;
    dds_low_command.motor_cmd().at(i).dq() = 0.0;
    dds_low_command.motor_cmd().at(i).kp() = (i < 13) ? 100.0 : 50.0;
    dds_low_command.motor_cmd().at(i).kd() = 1.0;
  }

  // Time May not be needed
  // I do not use duration
  time_ += control_dt_;

  // Duration 보다 Time 이 낮으면
  // Zero Posture 로 움직이게 하는 코드 << 필요할지도 모르겠음
  if (time_ < duration_) {
    // [Stage 1]: set robot to zero posture
    for (int i = 0; i < h12_num_motors_; ++i) {
      double ratio = std::clamp(time_ / duration_, 0.0, 1.0);
      dds_low_command.motor_cmd().at(i).q() =
          (1. - ratio) * low_state_.motor_state().at(i).q();
    }
  } else {
    // [Stage 2]: swing ankle's PR
    mode_ = PR;  // Enable PR mode

    // generate sin/cos trajectory
    // double max_P = 0.25;  // [rad]
    // double max_R = 0.25;  // [rad]
    // double t = time_ - duration_;
    // double L_P_des = max_P * std::cos(2.0 * M_PI * t);
    // double L_R_des = max_R * std::sin(2.0 * M_PI * t);
    // double R_P_des = max_P * std::cos(2.0 * M_PI * t);
    // double R_R_des = -max_R * std::sin(2.0 * M_PI * t);

    // update ankle joint position targets
    // float Kp_Pitch = 80;
    // float Kd_Pitch = 1;
    // float Kp_Roll = 80;
    // float Kd_Roll = 1;

    float Kp = 50.0;
    float Kd = 1.0;


    for (int i=LeftShoulderPitch; i<=RightWristYaw+1;i++)
    {
      dds_low_command.motor_cmd().at(i).q() = target_msgs_.position[i-LeftShoulderPitch];
      dds_low_command.motor_cmd().at(i).kp() = Kp;
      dds_low_command.motor_cmd().at(i).kd() = Kd;

      // Position Control --> Zero Torque, Zero Velocity
      dds_low_command.motor_cmd().at(i).tau() = 0.0;
      dds_low_command.motor_cmd().at(i).dq() = 0.0;

    }



    // dds_low_command.motor_cmd().at(4).q() = L_P_des;  // 4: LeftAnklePitch
    // dds_low_command.motor_cmd().at(4).dq() = 0;
    // dds_low_command.motor_cmd().at(4).kp() = Kp_Pitch;
    // dds_low_command.motor_cmd().at(4).kd() = Kd_Pitch;
    // dds_low_command.motor_cmd().at(4).tau() = 0;
    // dds_low_command.motor_cmd().at(5).q() = L_R_des;  // 5: LeftAnkleRoll
    // dds_low_command.motor_cmd().at(5).dq() = 0;
    // dds_low_command.motor_cmd().at(5).kp() = Kp_Roll;
    // dds_low_command.motor_cmd().at(5).kd() = Kd_Roll;
    // dds_low_command.motor_cmd().at(5).tau() = 0;
    // dds_low_command.motor_cmd().at(10).q() = R_P_des;  // 10: RightAnklePitch
    // dds_low_command.motor_cmd().at(10).dq() = 0;
    // dds_low_command.motor_cmd().at(10).kp() = Kp_Pitch;
    // dds_low_command.motor_cmd().at(10).kd() = Kd_Pitch;
    // dds_low_command.motor_cmd().at(10).tau() = 0;
    // dds_low_command.motor_cmd().at(11).q() = R_R_des;  // 11: RightAnkleRoll
    // dds_low_command.motor_cmd().at(11).dq() = 0;
    // dds_low_command.motor_cmd().at(11).kp() = Kp_Roll;
    // dds_low_command.motor_cmd().at(11).kd() = Kd_Roll;
    // dds_low_command.motor_cmd().at(11).tau() = 0;

    // float L_P_m = low_state_.motor_state().at(4).q();
    // float L_R_m = low_state_.motor_state().at(5).q();
    // float R_P_m = low_state_.motor_state().at(10).q();
    // float R_R_m = low_state_.motor_state().at(11).q();
    // clang-format off
      // printf("%f,%f,%f,%f,%f,%f,%f,%f\n",
      //         L_P_des, L_P_m, L_R_des, L_R_m,
      //         R_P_des, R_P_m, R_R_des, R_R_m);
    // clang-format on
  }

  // dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command,
  //                                   (sizeof(dds_low_command) >> 2) - 1);
  // lowcmd_publisher_->Write(dds_low_command);

  // Publisher 내일 작성하기

}

void H12Bridge::LowStateHandler(const void *message) {
  low_state_ = *(const unitree_hg::msg::dds_::LowState_ *)message;

  if (low_state_.crc() !=
      Crc32Core((uint32_t *)&low_state_,
                (sizeof(unitree_hg::msg::dds_::LowState_) >> 2) - 1)) {
    std::cout << "low_state CRC Error" << std::endl;
    return;
  }
}

void H12Bridge::topic_callback(const unitree_hg::msg::LowState::SharedPtr msg)
{
  auto mode_machine_ = static_cast<int>(msg->mode_machine);
  
  // TEST
  auto imu_ = msg->imu_state;
  unitree_hg::msg::MotorState motor_[h12_num_motors_];
  for (int i = 0; i < h12_num_motors_; i++) {
    motor_[i] = msg->motor_state[i];
  }

  sensor_msgs::msg::JointState joint_msg;
  joint_msg.position.resize(h12_num_motors_);
  




  // Info IMU states
  // RPY euler angle(ZYX order respected to body frame)
  // Quaternion
  // Gyroscope (raw data)
  // Accelerometer (raw data)
  RCLCPP_INFO(this->get_logger(),
              "Euler angle -- roll: %f; pitch: %f; yaw: %f", imu_.rpy[0],
              imu_.rpy[1], imu_.rpy[2]);
  RCLCPP_INFO(this->get_logger(),
              "Quaternion -- qw: %f; qx: %f; qy: %f; qz: %f",
              imu_.quaternion[0], imu_.quaternion[1], imu_.quaternion[2],
              imu_.quaternion[3]);
  RCLCPP_INFO(this->get_logger(), "Gyroscope -- wx: %f; wy: %f; wz: %f",
              imu_.gyroscope[0], imu_.gyroscope[1], imu_.gyroscope[2]);
  RCLCPP_INFO(this->get_logger(), "Accelerometer -- ax: %f; ay: %f; az: %f",
              imu_.accelerometer[0], imu_.accelerometer[1],
              imu_.accelerometer[2]);
  // Info motor states
  // q: angluar (rad)
  // dq: angluar velocity (rad/s)
  // ddq: angluar acceleration (rad/(s^2))
  // tau_est: Estimated external torque
  for (int i = 0; i < h12_num_motors_; i++) {
    motor_[i] = msg->motor_state[i];
    joint_msg.position.push_back(motor_[i].q);
    joint_msg.velocity.push_back(motor_[i].dq);

    RCLCPP_INFO(this->get_logger(),
                "Motor state -- num: %d; q: %f; dq: %f; ddq: %f; tau: %f",
                i, motor_[i].q, motor_[i].dq, motor_[i].ddq,
                motor_[i].tau_est);
  }

  joint_state_publisher_ -> publish(joint_msg);
}

