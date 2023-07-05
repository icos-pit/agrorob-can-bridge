#include <memory>
#include <math.h>

#include "agrorob_driver/agrorob_interface.hpp"

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
  AgrorobInterface::AgrorobInterface() : Node("agrorob_interface")
  {
    raw_can_sub_ = this->create_subscription<can_msgs::msg::Frame>("from_can_bus", 10, std::bind(&AgrorobInterface::can_callback, this, _1));
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&AgrorobInterface::joy_callback, this, _1));

    remote_state_pub_ = this->create_publisher<agrorob_msgs::msg::RemoteState>("/agrorob/remote_state", 10);
    robot_state_pub_ = this->create_publisher<agrorob_msgs::msg::RobotState>("/agrorob/robot_state", 10);
    tool_stats_pub_ = this->create_publisher<agrorob_msgs::msg::ToolState>("/agrorob/tool_state", 10);
    engine_stats_pub_ = this->create_publisher<agrorob_msgs::msg::EngineState>("/agrorob/engine_state", 10);
    failure_state_pub_ = this->create_publisher<agrorob_msgs::msg::FailureState>("/agrorob/failure_state", 10);
    raw_can_pub_ = this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 10);

    // engine_start_srv_ = this->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);
    rpm_to_rad_s =  0.10472;  
    rpm_velocity = 140;    
  }

  can_msgs::msg::Frame AgrorobInterface::initialize_can_frame()
  {
    can_msgs::msg::Frame frame;
    frame.header.frame_id = "can";
    frame.is_rtr = false;
    frame.is_extended = false;
    frame.is_error = false;
    frame.dlc = 8;
    return frame;
  }
  
  void AgrorobInterface::joy_callback(const sensor_msgs::msg::Joy & joy_msg)
  {
    int x = AgrorobInterface::get_engine_rpm(joy_msg);
    RCLCPP_INFO(this->get_logger(), "Velocity of engine in rpm: %d", x);

    can_id1.id = 1;
    can_id25.id = 25;    


    if(remote_state_msg.auto_mode_enabled == 1)
    {

      if(agrorob_ready_to_move == false)
      {
        if(initializing)
          RCLCPP_INFO(this->get_logger(), "Initializing...");
        can_id1.data[0] = 0;
        can_id1.data[1] = 0;
        can_id1.data[3] = 170;
        can_id1.data[7] = 0;
        can_id1.data[6] = 0;
        can_id1.data[5] = 0;
        can_id1.data[4] = 0;
      }


      if(engine_state_msg.engine_running == false)
        {
          if(initializing)
            RCLCPP_INFO(this->get_logger(), "Engine is off, click B (red) for ignition");

          if(joy_msg.buttons[1] == 1)
          {
            RCLCPP_INFO(this->get_logger(), "ignition command send");
            can_id1.data[2] = 1;
          }else
            can_id1.data[2] = 0;

        }


      if(engine_state_msg.engine_running == true)
      {
        if(!agrorob_ready_to_move)
          RCLCPP_INFO(this->get_logger(), "Engine is running");
        can_id1.data[0] = 0;
        can_id1.data[1] = 0;
        can_id1.data[2] = 0;
        can_id1.data[3] = 170;
        can_id1.data[4] = 0;
        can_id1.data[5] = 1;
        can_id1.data[7] = 0;
        if(!agrorob_ready_to_move)
          RCLCPP_INFO(this->get_logger(), "Agrorob ready to move");
        agrorob_ready_to_move = true;
      }
      
      if(joy_msg.axes[5] < -0.9) //break
        can_id1.data[4] = 1;
      else
        can_id1.data[4] = 0;

      can_id25.data[0] = 2;
      can_id25.data[1] = (joy_msg.axes[2] * -90) + 90;  //steering 
      
      if(joy_msg.axes[1] > 0.1)     //direction of movement
        can_id25.data[2] = 1;
      else if(joy_msg.axes[1] < -0.1)
        can_id25.data[2] = 2;
      else
        can_id25.data[2] = 0;
      
      can_id25.data[3] = abs(joy_msg.axes[1]) * 100;
      can_id25.data[4] = 200;
      can_id25.data[5] = 20;
      can_id25.data[6] = 180;

      can_id1.header.stamp = this->get_clock()->now();
      can_id25.header.stamp = this->get_clock()->now();

      raw_can_pub_->publish(can_id1);
      raw_can_pub_->publish(can_id25);      
      initializing = false;
    }
    
  }

  int AgrorobInterface::get_engine_rpm(const sensor_msgs::msg::Joy & joy_msg)
  {
    if (joy_msg.buttons[3] == 1 && joy_msg.buttons[5] == 1 && AgrorobInterface::rpm_velocity < 180){
      rpm_velocity += 10;
    }
    if (joy_msg.buttons[0] == 1 && joy_msg.buttons[5] == 1 && AgrorobInterface::rpm_velocity > 140){
      rpm_velocity -= 10;
    }
    return rpm_velocity;
  }

  void AgrorobInterface::can_callback(const can_msgs::msg::Frame & can_msg) 
  {
    
    
    switch(can_msg.id)
    {

      case 100: //From REMOTE
      {

        if(remote_state_msg.auto_mode_enabled != can_msg.data[2])
        {
          if(can_msg.data[2] == 0) RCLCPP_INFO(this->get_logger(), "REMOTE: Operation mode: --MANUAL--");
          else RCLCPP_INFO(this->get_logger(), "REMOTE: Operation mode: --AUTO--");
        }

        if(remote_state_msg.stop_engine != can_msg.data[1])
        {
          if(can_msg.data[1] == 0) RCLCPP_INFO(this->get_logger(), "REMOTE: Engine: --ACTIVE--");
          else RCLCPP_INFO(this->get_logger(), "REMOTE: Engine: --SHUTDOWN--");
        }

        if(remote_state_msg.stop_drivers != can_msg.data[0])
        {
          if(can_msg.data[0] == 0) RCLCPP_INFO(this->get_logger(), "REMOTE: Drivers: --ACTIVE--");
          else RCLCPP_INFO(this->get_logger(), "REMOTE: Drivers: --SHUTDOWN--");
        }

        remote_state_msg.stop_drivers       =  can_msg.data[0]; //0 - inactive; 1 - STOP of all drives active
        remote_state_msg.stop_engine        =  can_msg.data[1]; //0 - the internal combustion engine can work; 1 - engine shutdown
        remote_state_msg.auto_mode_enabled  =  can_msg.data[2]; //0 - manual mode; 1 - automatic mode
        break;
      }

      case 31:
      {
        robot_state_msg.left_front_wheel_encoder_imp  = (can_msg.data[1] << 8) + can_msg.data[0]; // high << 8 + low
        robot_state_msg.right_front_wheel_encoder_imp = (can_msg.data[3] << 8) + can_msg.data[2];
        robot_state_msg.left_rear_wheel_encoder_imp   = (can_msg.data[5] << 8) + can_msg.data[4];
        robot_state_msg.right_rear_wheel_encoder_imp  = (can_msg.data[7] << 8) + can_msg.data[6];

        robot_state_pub_->publish(robot_state_msg);
        break;
      }

      case 32: 
      {
        robot_state_msg.left_front_wheel_turn_angle_rad  = (can_msg.data[0] - 90) * (M_PI/180) ; //
        robot_state_msg.right_front_wheel_turn_angle_rad = (can_msg.data[1] - 90) * (M_PI/180) ;
        robot_state_msg.left_rear_wheel_turn_angle_rad   = (can_msg.data[2] - 90) * (M_PI/180) ;
        robot_state_msg.right_rear_wheel_turn_angle_rad  = (can_msg.data[3] - 90) * (M_PI/180) ;

        robot_state_msg.left_front_wheel_rotational_speed_rad_s   = can_msg.data[4] * rpm_to_rad_s;
        robot_state_msg.right_front_wheel_rotational_speed_rad_s  = can_msg.data[5] * rpm_to_rad_s;
        robot_state_msg.left_rear_wheel_rotational_speed_rad_s    = can_msg.data[6] * rpm_to_rad_s;
        robot_state_msg.right_rear_wheel_rotational_speed_rad_s   = can_msg.data[7] * rpm_to_rad_s;
        robot_state_pub_->publish(robot_state_msg);
        break;
      }

      case 33: 
      {

        tool_state_msg.vertical_actuator_pos             = can_msg.data[0];
        tool_state_msg.horizental_actuator_pos           = can_msg.data[1];
        tool_state_msg.tiller_left_limit_enabled         = can_msg.data[2];
        tool_state_msg.tiller_right_limit_enabled        = can_msg.data[3];
        tool_state_msg.sowing_shaft_rotational_speed_rpm = can_msg.data[4];
        tool_state_msg.fertilizer_tank1_level            = can_msg.data[5];
        tool_state_msg.fertilizer_tank2_level            = can_msg.data[6];
        tool_stats_pub_->publish(tool_state_msg);
        break;
      }

      case 34: 
      {
        if(can_msg.data[0] <= 60)  // degrees minus 60(e.g. 65 means 65-60 = 5 degrees)
          tool_state_msg.tool_level_rad = can_msg.data[0] * (M_PI/180);
        else
          tool_state_msg.tool_level_rad = (can_msg.data[0] - 60) * (M_PI/180);

        tool_state_msg.hydraulic_oil_tem_celsius           = can_msg.data[2]; 
        tool_state_msg.hydraulic_oil_pressure              = can_msg.data[3]; 
        tool_state_msg.hydraulic_oil_under_min_level       = can_msg.data[4]; 
        tool_state_msg.hydraulic_oil_above_max_level       = can_msg.data[5]; 
        tool_state_msg.wheels_average_rotational_speed_rpm = can_msg.data[6];
        tool_stats_pub_->publish(tool_state_msg);
        break; 
      }
      
      case 51: 
      {
        engine_state_msg.engine_rotation_speed_rpm  = (can_msg.data[1] << 8) + can_msg.data[0]; // high << 8 + low
        engine_state_msg.engine_running               = can_msg.data[2];                        // 0 - off; 1 - the engine is running
        engine_state_msg.engine_coolant_temp_celsius  = can_msg.data[3];                        // 0 - 100 - value determining the temperature ( o C )
        engine_state_msg.engine_fuel_level_percent    = can_msg.data[5];                        // 0-100 - value specifying% of tank filling
        engine_state_msg.engine_oil_pressure_bar      = can_msg.data[7];                        // 0 - 100 (100 means 10 bar)
        engine_stats_pub_->publish(engine_state_msg);
        break;
      }

      case 99: 
      {
        failure_state_msg.left_front_turn_failed           = can_msg.data[0];
        failure_state_msg.right_front_turn_failed          = can_msg.data[1];
        failure_state_msg.left_rear_turn_failed            = can_msg.data[2];
        failure_state_msg.right_rear_turn_failed           = can_msg.data[3];
        failure_state_msg.tool_vertical_actuator_failed    = can_msg.data[4];
        failure_state_msg.tool_horizontal_actuator_failed  = can_msg.data[5];
        failure_state_msg.tool_level_sensor_failed         = can_msg.data[6];

        for(auto const&  any_failure_end : can_msg.data)
          if(any_failure_end) RCLCPP_INFO(this->get_logger(), "Sensor failure detected!");

        failure_state_pub_->publish(failure_state_msg);
        break;
      }
    }
  }
  
}
