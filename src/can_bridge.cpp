#include <memory>
#include <iostream>
#include <iterator>
#include <set>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "can_msgs/msg/frame.hpp"

using namespace std;
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

    

    void showset(set<int> g)
    {
      set<int>::iterator it;
      for (it = g.begin(); it != g.end(); ++it)
          cout << '\t' << *it;
      cout << '\n';
    }

    void make_set(const can_msgs::msg::Frame & can_msg)
    {
      can_id_set.emplace(can_msg.id);
      showset(can_id_set);
    }

    void single_can_msg_printer(const can_msgs::msg::Frame & can_msg) const
    {
      cout << "id: " << can_msg.id << endl << "data:"<< endl;

      for (int each_byte : can_msg.data)
      {
        cout << "- " << each_byte << endl;
      }
    
    }

    void can_callback(const can_msgs::msg::Frame & can_msg) 
    {
      if(can_msg.id == 200)
        single_can_msg_printer(can_msg);
      // make_set(can_msg)
      // RCLCPP_INFO(this->get_logger(), "I heard id: %u", can_msg.id);
      
    }
    
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_subscription_;
    set<int> can_id_set;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanBridge>());
  rclcpp::shutdown();
  return 0;
}
