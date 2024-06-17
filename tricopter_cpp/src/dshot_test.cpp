#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <chrono>
#include <memory>
#include <string>

#include "tricopter_msgs/msg/motors.hpp"

#include "rclcpp/rclcpp.hpp"

// #include "motor-dshot.cpp"
extern void motorImplementationInitialize(int motorPins[], int motorMax) ;
extern void motorImplementationFinalize(int motorPins[], int motorMax) ;
extern void motorImplementationSendThrottles(int motorPins[], int motorMax, double motorThrottle[]) ;

using namespace std::chrono_literals;

float motor_lu=0;

class MotorControl : public rclcpp::Node

{
    public: 
    MotorControl() : Node("motor_controller")
    {
        RCLCPP_INFO(this->get_logger(), "Started program");
        auto topic_callback = [this](tricopter_msgs::msg::Motors::UniquePtr msg) -> void  //Fancy lambda function !
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->motor_lu);
            motor_lu = msg->motor_lu;
        };
        subscription_ =
        this->create_subscription<tricopter_msgs::msg::Motors>("tricopter/motor_cmd", 1, topic_callback);
    }

    private:
    rclcpp::Subscription<tricopter_msgs::msg::Motors>::SharedPtr subscription_;    
};

int main(int argc, char **argv) 
{

    int motorPins[]={16, 19, 20, 21, 12};
    double throttles[]={0, 0, 0, 0, 0};
    int i, n=5;

    // printf("Started program \n");

    motorImplementationInitialize(motorPins, n);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControl>());


    printf("Initializing ESC / Arm, waiting 5 seconds.\n");
    // send 0 throttle during 5 seconds
    for(i=0; i<n; i++) throttles[i] = 0.1;
    for(i=0; i<5000; i++) {
        motorImplementationSendThrottles(motorPins, n, throttles);
        usleep(1000);
    }

    // printf("Spinning.\n");
    // // make motors spinning on 15% throttle during 5 seconds
    // for(i=0; i<n; i++) throttles[i] = 0.15;
    // for(i=0; i<5000; i++) {
    // motorImplementationSendThrottles(motorPins, n, throttles);
    // usleep(1000);
    // }
    
    printf("Stop.\n");
    // stop motors
    for(i=0; i<n; i++) throttles[i] = 0;
    motorImplementationSendThrottles(motorPins, n, throttles);

    // finalize
    motorImplementationFinalize(motorPins, n);
    rclcpp::shutdown();
    return(0);
    }