#pragma once

#include <agrorob_driver/digital_filters.hpp>
// #include <agrorob_driver/adrc_eso.hpp>


namespace agrorob_interface
{
    class VelocityController
    {

    double throttle_;
    double brake_;
    
    double dt_; 
    double controlScale = 1.0;
    //BCM pin numbering mode
    double kd = 0.4; 
    double ki = 0.1;
    double kf = 0.0;

    double errorIntegral = 0.0;

    double prevVelocity = 0.0;
    LowPassFilter3 velocityRateFilter;
    
    double prevControl = 0.0;

    int direction = 0; //0 - the robot is not moving; 1 - forward drive  2 - driving backwards 
    

    

    public:

        VelocityController(double loopRate) 
        : dt_(1.0/loopRate), velocityRateFilter(dt_, 200.0)
        {
            
        }


        double getDirection()
        {
            // return direction;
            return 0;

        }
        
        
        
        double getVelocity()
        {

           return 0.0;
        }
        
        
      
        void update(double velocity, double referenceVelocity, double referenceVelocityRate )
        {
            
            // // Check, if car goes to opossite direction than requesed, If so, start braking.
            // if ((referenceVelocity > 0.0 && tbVel_->getGear() == 2) || (referenceVelocity < 0.0 && tbVel_->getGear() == 1))
            // {
            //     braking_ = true;
            // }


            // // If going backwards, change sign of refVel to opposite, control then is the same as going forward
            // if (tbVel_->getGear() == 2){
            //     referenceVelocity = -referenceVelocity;
            // }
            
            // double velocityError = referenceVelocity - velocity;
            // double controlSignal = 0.0;

            // if (velocity <= referenceVelocity && referenceVelocity > 0.0)
            //     braking_ = false;
            // // braking_ = velocity <= referenceVelocity ? false;

            // if (velocity > 1.5*referenceVelocity || braking_) {
            //     braking_ = true;
            //     controlSignal = -1.0;
            // } else {
            //     errorIntegral += velocityError*(1.0/loopRate_);
            //     double velocityRate = velocityRateFilter.update((velocity - prevVelocity)*loopRate_);
            //     prevVelocity = velocity;
            //     double velocityRateError = referenceVelocityRate - velocityRate;
                

            //     controlSignal = kd*velocityError + ki*errorIntegral + kf*velocityRateError;


                

              

            //     if (controlSignal > 1.0)
            //         controlSignal = 1.0;
            //     if (controlSignal < 0.0)
            //         controlSignal = 0.0;
            // }

            // // RCLCPP_INFO_STREAM(nh_->get_logger(), "Control signal: " << controlSignal);

            // tbVel_->setThrottleValue(controlSignal*controlScale);


        }
    };
}