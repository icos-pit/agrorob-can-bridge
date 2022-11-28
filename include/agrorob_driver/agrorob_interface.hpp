#ifndef __AGROROB_INTERFACE_HPP__
#define __AGROROB_INTERFACE_HPP__


#include <memory>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "can_msgs/msg/frame.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "agrorob_msgs/msg/engine_state.hpp"
#include "agrorob_msgs/msg/failure_state.hpp"
#include "agrorob_msgs/msg/tool_state.hpp"
#include "agrorob_msgs/msg/remote_state.hpp"
#include "agrorob_msgs/msg/robot_state.hpp"

// #include "agrorob_msgs/msg/mode_control.hpp"
// #include "agrorob_msgs/msg/robot_control.hpp"
// #include "agrorob_msgs/msg/sprayer_control.hpp"
// #include "agrorob_msgs/msg/tool_control.hpp"

using namespace std;
using std::placeholders::_1;
namespace agrorob_interface
{
  class AgrorobInterface : public rclcpp::Node
  {
    public:
      AgrorobInterface(); 
      can_msgs::msg::Frame initialize_can_frame();

    private:
      
      void joy_callback(const sensor_msgs::msg::Joy & joy_msg);
      void can_callback(const can_msgs::msg::Frame & can_msg);
    
      rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr raw_can_sub_;
      rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
      
      rclcpp::Publisher<agrorob_msgs::msg::RemoteState>::SharedPtr remote_state_pub_;
      rclcpp::Publisher<agrorob_msgs::msg::RobotState>::SharedPtr robot_state_pub_;
      rclcpp::Publisher<agrorob_msgs::msg::ToolState>::SharedPtr tool_stats_pub_;
      rclcpp::Publisher<agrorob_msgs::msg::EngineState>::SharedPtr engine_stats_pub_; 
      rclcpp::Publisher<agrorob_msgs::msg::FailureState>::SharedPtr failure_state_pub_; 
      
      rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr raw_can_pub_;

      // rclcpp::Service<common_interfaces::srv::Trigger>::SharedPtr engine_start_srv_;
      
      agrorob_msgs::msg::RemoteState remote_state_msg = agrorob_msgs::msg::RemoteState();  
      agrorob_msgs::msg::RobotState robot_state_msg = agrorob_msgs::msg::RobotState();
      agrorob_msgs::msg::ToolState tool_state_msg = agrorob_msgs::msg::ToolState();
      agrorob_msgs::msg::EngineState engine_state_msg = agrorob_msgs::msg::EngineState();
      agrorob_msgs::msg::FailureState failure_state_msg =  agrorob_msgs::msg::FailureState();

      can_msgs::msg::Frame can_id1 = initialize_can_frame();
      can_msgs::msg::Frame can_id25 = initialize_can_frame();
      
      // auto mode_control_message = agrorob_msgs::msg::ModeControl();
      // auto robot_control_message = agrorob_msgs::msg::RobotControl();
      // auto sprayer_control_message = agrorob_msgs::msg::SprayerControl();
      // auto tool_control_message = agrorob_msgs::msg::ToolControl();

      bool agrorob_ready_to_move = false;
      bool initializing = true;
      double rpm_to_rad_s;
        
      
  };

}

#endif  //__AGROROB_INTERFACE_HPP__