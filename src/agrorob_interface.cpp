#include <memory>
#include <iostream>
#include <iterator>
#include <set>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "can_msgs/msg/frame.hpp"

#include "agrorob_interfaces/mode_manager.hpp"

#include "agrorob_msgs/msg/engine_state.hpp"
// #include "agrorob_can_msgs/msg/failure_state.hpp"
// #include "agrorob_can_msgs/msg/mode_control.hpp"
// #include "agrorob_can_msgs/msg/robot_control.hpp"
// #include "agrorob_can_msgs/msg/robot_state.hpp"
// #include "agrorob_can_msgs/msg/sprayer_control.hpp"
// #include "agrorob_can_msgs/msg/tool_control.hpp"
// #include "agrorob_can_msgs/msg/tool_state.hpp"

using namespace std;
using std::placeholders::_1;
using namespace agrorob_interface;



class AgrorobInterface : public rclcpp::Node
{
  public:
    AgrorobInterface() : Node("agrorob_interface"), modeManager()
    {
      

      raw_can_sub_ = this->create_subscription<can_msgs::msg::Frame>("from_can_bus", 10, std::bind(&AgrorobInterface::can_callback, this, _1));
      engine_stats_pub_ = this->create_publisher<agrorob_msgs::msg::EngineState>("/agrorob/engine_state", 10);


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
      auto can_msg_ = can_msg;
      // if(can_msg.id == 200)
      //   single_can_msg_printer(can_msg);
      // make_set(can_msg)
      // RCLCPP_INFO(this->get_logger(), "I heard id: %u", can_msg.id);
      
      

      switch(can_msg.id)
      {
        case 100: // This id belongs to msgs from manual control pad 
        {
          modeManager.update_remote_state(can_msg_);
          // modeManager.is_agrorob_ready();
          break;
        }
        
        case 51: 
        {
          auto engine_state_message = agrorob_msgs::msg::EngineState();
          engine_state_message.engine_coolant_temp_celsius = can_msg.data[3];
          engine_stats_pub_->publish(engine_state_message);
          RCLCPP_INFO(this->get_logger(), "I heard id: %u", can_msg.id);
          break;
          
        }
        
        
      }

      
    }
    
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr raw_can_sub_;
    rclcpp::Publisher<agrorob_msgs::msg::EngineState>::SharedPtr engine_stats_pub_; 
    
    // auto failure_state_message = agrorob_msgs::msg::FailureState();
      // auto mode_control_message = agrorob_msgs::msg::ModeControl();
      // auto robot_control_message = agrorob_msgs::msg::RobotControl();
      // auto robot_state_message = agrorob_msgs::msg::RobotState();
      // auto sprayer_control_message = agrorob_msgs::msg::SprayerControl();
      // auto tool_control_message = agrorob_msgs::msg::ToolControl();
      // auto tool_state_message = agrorob_msgs::msg::ToolState();
    
    ModeManager modeManager;
    set<int> can_id_set;
    

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgrorobInterface>());
  rclcpp::shutdown();
  return 0;
}
