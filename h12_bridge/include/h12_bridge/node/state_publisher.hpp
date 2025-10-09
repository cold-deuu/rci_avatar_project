#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp" // String 메시지 타입을 사용하기 위해 포함
#include "sensor_msgs/msg/joint_state.hpp"

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

// Test
// #include "common/motor_crc_hg.h" << 필요하면 예제에서 가져오기
#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "unitree_hg/msg/motor_cmd.hpp"


static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_STATE_TOPIC = "rt/lowstate";

using namespace unitree::common;
using namespace unitree::robot;


enum PRorAB { PR = 0, AB = 1 };

enum H1JointIndex {
  // legs
  LeftHipYaw = 0,
  LeftHipPitch = 1,
  LeftHipRoll = 2,
  LeftKnee = 3,
  LeftAnklePitch = 4,
  LeftAnkleB = 4,
  LeftAnkleRoll = 5,
  LeftAnkleA = 5,
  RightHipYaw = 6,
  RightHipPitch = 7,
  RightHipRoll = 8,
  RightKnee = 9,
  RightAnklePitch = 10,
  RightAnkleB = 10,
  RightAnkleRoll = 11,
  RightAnkleA = 11,
  // torso
  WaistYaw = 12,
  // arms
  LeftShoulderPitch = 13,
  LeftShoulderRoll = 14,
  LeftShoulderYaw = 15,
  LeftElbow = 16,
  LeftWristRoll = 17,
  LeftWristPitch = 18,
  LeftWristYaw = 19,
  RightShoulderPitch = 20,
  RightShoulderRoll = 21,
  RightShoulderYaw = 22,
  RightElbow = 23,
  RightWristRoll = 24,
  RightWristPitch = 25,
  RightWristYaw = 26
};


// Move Util File if is possible
inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++) {
    xbit = 1 << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (CRC32 & 0x80000000) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      } else
        CRC32 <<= 1;
      if (data & xbit) CRC32 ^= dwPolynomial;

      xbit >>= 1;
    }
  }
  return CRC32;
};

class H12Bridge : public rclcpp::Node
{
public:
  // 생성자: 노드 이름을 받아 초기화
  H12Bridge(std::string networkInterface);

private:
  void topic_callback(const unitree_hg::msg::LowState::SharedPtr msg);
  void Control_H12();
  void Check_H12_Index();
  void LowStateHandler(const void *message);

  // Control Input Subscribe
  // rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  // std::shared_ptr<rclcpp::Publisher>


  
  sensor_msgs::msg::JointState target_msgs_;

  unitree_hg::msg::dds_::LowState_ low_state_;

  double time_;
  double control_dt_;  // [2ms]
  double duration_;    // [3 s]
  PRorAB mode_;

  std::string h12_cmd_topic_;
  std::string h12_state_topic_;
  std::string network_interface_;
  int h12_num_motors_;

  ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> lowcmd_publisher_;
  ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> lowstate_subscriber_;
  ThreadPtr command_writer_ptr_, control_thread_ptr_;

  // Test
  rclcpp::TimerBase::SharedPtr timer_;
  void ctrl_timer();


};
