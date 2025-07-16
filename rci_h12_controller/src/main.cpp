#include "rclcpp/rclcpp.hpp"
#include "rci_h12_controller/simulator/simulator.hpp"


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rci_simulator::H12_Simulator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}