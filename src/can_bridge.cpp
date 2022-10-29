#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "can_msgs/msg/frame.hpp"


using std::placeholders::_1;

class CanBridge : public rclcpp::Node
{
  public:
    CanBridge()
    : Node("can_bridge")
    {
      can_subscription_ = this->create_subscription<can_msgs::msg::Frame>(
      "from_can_bus", 10, std::bind(&CanBridge::can_callback, this, _1));
    }

  private:
    void can_callback(const can_msgs::msg::Frame & can_msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard id: %u", can_msg.id);
    }
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanBridge>());
  rclcpp::shutdown();
  return 0;
}
