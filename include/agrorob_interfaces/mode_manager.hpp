#pragma once

#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"

namespace agrorob_interface
{
    using namespace rclcpp;

    class ModeManager
    {

        bool drivers_stop_ = true;
        bool engine_stop_ = true;
        bool auto_mode_ = false;
        
        bool agrorob_ready_;

        void check_if_agrorob_ready()
        {
            if(auto_mode_ && !engine_stop_ && !drivers_stop_)
                agrorob_ready_ = true;
            else
            {
                agrorob_ready_ = false;
                // RCLCPP_INFO(get_logger(), "Creating");
            }
                
             
        }



    public:
        ModeManager(): agrorob_ready_(false) {}

        void update_remote_state(can_msgs::msg::Frame & can_msg)
        {    
            drivers_stop_ = can_msg.data[0];       //B0 - drives STOP; 0 - inactive; 1 - STOP of all drives active
            engine_stop_ = can_msg.data[1];        //B1 - engine STOP; 0 - the internal combustion engine can work; 1 - engine shutdown
            auto_mode_ = can_msg.data[1];          //B2 - MAN / AUTO; 0 - manual mode; 1 - automatic mode
            
            check_if_agrorob_ready();
        }

        bool is_agrorob_ready()
        {
            return agrorob_ready_;
        }
    

    };
  

}
