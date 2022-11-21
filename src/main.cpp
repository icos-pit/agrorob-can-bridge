
#include <memory>

#include "agrorob_driver/agrorob_interface.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Agrorob driver started");
  rclcpp::spin(std::make_shared<agrorob_interface::AgrorobInterface>());
  rclcpp::shutdown();
  return 0;
}
