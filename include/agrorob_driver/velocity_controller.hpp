#pragma once

#include <melex_ros2/digital_filters.hpp>
#include <melex_ros2/adrc_eso.hpp>
#include <melex_ros2/throttle_and_brake.hpp>

namespace melex_ros2
{
    class VelocityController
    {

    double throttle_;
    double brake_;
    
    double loopRate_; 
    double controlScale = 1.0;
    //BCM pin numbering mode
    double kd = 0.4; 
    double ki = 0.1;
    double errorIntegral = 0.0;

    double kf = 0.0;

    double prevVelocity = 0.0;
    LowPassFilter3 velocityRateFilter;
    
    double prevControl = 0.0;

    bool braking_ = false;
    //SecondOrderESO eso;

    // std::shared_ptr<rclcpp::Node> node_handle_;
    rclcpp::Node::SharedPtr& nh_;
    ThrottleAndBrake *tbVel_ = nullptr;
    

    public:
        VelocityController(rclcpp::Node::SharedPtr& nodeHandle, double loopRate, ThrottleAndBrake *tbVel) 
        : nh_(nodeHandle), loopRate_(loopRate), velocityRateFilter(1.0/loopRate, 200.0), tbVel_(tbVel)
        {
            // node_handle_ = nodeHandle;
        }
        //eso(1.0/loopRate, 1.0, 100.0, 0.1)
      
        void update(double velocity, double referenceVelocity, double referenceVelocityRate )
        {
            // Check, if car goes to opossite direction than requesed, If so, start braking.
            if ((referenceVelocity > 0.0 && tbVel_->getGear() == 2) || (referenceVelocity < 0.0 && tbVel_->getGear() == 1))
            {
                braking_ = true;
            }

            // Change gear only, when the car is stopped
            if (velocity == 0.0){
                if (referenceVelocity > 0.0){
                    tbVel_->setGear(1);
                } else if (referenceVelocity < 0.0){
                    tbVel_->setGear(2);
                } else {
                    //tbVel_->setGear(0);
                }
            }

            // If going backwards, change sign of refVel to opposite, control then is the same as going forward
            if (tbVel_->getGear() == 2){
                referenceVelocity = -referenceVelocity;
            }
            
            double velocityError = referenceVelocity - velocity;
            double controlSignal = 0.0;

            if (velocity <= referenceVelocity && referenceVelocity > 0.0)
                braking_ = false;
            // braking_ = velocity <= referenceVelocity ? false;

            if (velocity > 1.5*referenceVelocity || braking_) {
                braking_ = true;
                controlSignal = -1.0;
            } else {
                errorIntegral += velocityError*(1.0/loopRate_);
                double velocityRate = velocityRateFilter.update((velocity - prevVelocity)*loopRate_);
                prevVelocity = velocity;
                double velocityRateError = referenceVelocityRate - velocityRate;
                
                //TODO: try dual output maybe (uncomment those 2 lines and modify ESO)
                // Eigen::Vector2d output;
                // output<<steeringAngle, steeringError;
                controlSignal = kd*velocityError + ki*errorIntegral + kf*velocityRateError;//eso.feedbackUpdate(steeringAngle, prevControl, kd*steeringError + kf*steeringRateError);


                

                // if(controlSignal > dutyCycleLimit)
                // {
                //     controlSignal = dutyCycleLimit;
                // } 
                // else if(controlSignal < -dutyCycleLimit)
                // {
                //     controlSignal = -dutyCycleLimit;
                // } 

                // if(fabs(steeringAngle) >= steeringAngleLimit_)
                // {
                //     controlSignal = 0.0;
                //     RCLCPP_WARN(nh_->get_logger(), "Steering angle limit reached");
                // }


                // pwm.setSignedDutyCycle(controlSignal);
                // prevControl = controlSignal;

                // RCLCPP_INFO_STREAM(nh_->get_logger(), "Control signal before conditions: " << controlSignal);

                if (controlSignal > 1.0)
                    controlSignal = 1.0;
                if (controlSignal < 0.0)
                    controlSignal = 0.0;
            }

            // RCLCPP_INFO_STREAM(nh_->get_logger(), "Control signal: " << controlSignal);

            tbVel_->setThrottleValue(controlSignal*controlScale);


        }
    };
}