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


int motorPins[]={16, 19, 20, 21, 12};
double throttles[]={0, 0, 0, 0, 0};
int i, n=5;
#define MAX_MOTOR_THROTTLE 70/100

class MotorControl : public rclcpp::Node

{
    public: 
    MotorControl() : Node("motor_controller")
    {
        RCLCPP_INFO(this->get_logger(), "Started motor control");
        auto topic_callback = [this](tricopter_msgs::msg::Motors::UniquePtr msg) -> void  //Fancy lambda function !
        {
            RCLCPP_INFO(this->get_logger(), "Running motors LU: '%f' RU: '%f' LD: '%f' RD: '%f' B: '%f'", msg->motor_lu, msg->motor_ru, msg->motor_ld, msg->motor_rd, msg->motor_b);
            throttles[0]=(msg->motor_lu-1000)/1000;
            throttles[1]=(msg->motor_ru-1000)/1000;
            throttles[2]=(msg->motor_ld-1000)/1000;
            throttles[3]=(msg->motor_rd-1000)/1000;
            throttles[4]=(msg->motor_b-1000)/1000;
            if (throttles[0]>0.7) throttles[0]=MAX_MOTOR_THROTTLE;
            if (throttles[1]>0.7) throttles[1]=MAX_MOTOR_THROTTLE;
            if (throttles[2]>0.7) throttles[2]=MAX_MOTOR_THROTTLE;
            if (throttles[3]>0.7) throttles[3]=MAX_MOTOR_THROTTLE;
            if (throttles[4]>0.7) throttles[4]=MAX_MOTOR_THROTTLE;
        };
        subscription_ =
        this->create_subscription<tricopter_msgs::msg::Motors>("tricopter/motor_cmd", 1, topic_callback);
    
        auto send_dshot_frames =
        [this]() -> void {
            // RCLCPP_INFO(this->get_logger(), "Publishing");
            motorImplementationSendThrottles(motorPins, n, throttles);
        };
        timer_ = this->create_wall_timer(1ms, send_dshot_frames);

    }

    private:
    rclcpp::Subscription<tricopter_msgs::msg::Motors>::SharedPtr subscription_;    
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char **argv) 
{
    motorImplementationInitialize(motorPins, n);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControl>());

    printf("Stop.\n");
    // stop motors
    for (i=0;i<n;i++) throttles[i]=0;
    motorImplementationSendThrottles(motorPins, n, throttles);
    // RCLCPP_INFO(motor_control_node->get_logger(), "Publishing");
    // finalize
    motorImplementationFinalize(motorPins, n);
    rclcpp::shutdown();
    return(0);
    }