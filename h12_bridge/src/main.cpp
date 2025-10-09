#include "rclcpp/rclcpp.hpp"
#include "h12_bridge/node/state_publisher.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::string netwokr_interface = "for_test";
  auto node = std::make_shared<H12Bridge>(netwokr_interface);

  // 3. 정보 로그 출력
  RCLCPP_INFO(node->get_logger(), "Hello, rclcpp world!");

  rclcpp::spin(node);
  rclcpp::shutdown();

};