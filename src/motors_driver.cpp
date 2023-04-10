//Includes------------------------------------------------------
#include <pigpiod_if2.h>

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
//--------------------------------------------------------------

//namespace-----------------------------------------------------
using namespace std;
using std::placeholders::_1;
//--------------------------------------------------------------

//defines-------------------------------------------------------
#define PWM_GPIO_MOTOR_LEFT 12
#define PWM_GPIO_MOTOR_RIGHT 13
#define DIRECTION_GPIO_MRIGHT 23
#define DIRECTION_GPIO_MLEFT 24
#define GEAR_RATIO 60
#define LBWheels 0.3                                //in meters
#define RWheels 0.0575                              //in meters
#define MAX_RPM 5000
#define MIN_RPM 0
#define MAX_PERCENTAGE 90
#define MIN_PERCENTAGE 10
#define CW 0
#define CCW 1
#define PWM_FREQ 100
//--------------------------------------------------------------

int gpio;

//Function/class------------------------------------------------
class Motor_driver_subscriber : public rclcpp::Node
{
    public:
        Motor_driver_subscriber() 
        : Node("motor_driver")
        {
            subscription_ = this -> create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10,std::bind(&Motor_driver_subscriber::topic_callback, this, _1));
        }

    private:
        double aVelL = 0.0;
        double aVelR = 0.0;
        double RPMR = 0.0;
        double RPML = 0.0;
        double PWM_L = 0.0;
        double PWM_R = 0.0;

        void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            //converting velocities into RPM's
            aVelL = msg->linear.x/RWheels - LBWheels*msg->angular.z/(2*RWheels);
            aVelR = msg->linear.x/RWheels + LBWheels*msg->angular.z/(2*RWheels);

            RPML = aVelL*60*GEAR_RATIO/(2*M_PI);
            RPMR = aVelR*60*GEAR_RATIO/(2*M_PI);

            if(RPML > MAX_RPM)
            {
                RCLCPP_INFO(this->get_logger(),"Warning : Left motor speed too high : '%f'",RPML);
                RPML = MAX_RPM;
            }
            if(RPMR > MAX_RPM)
            {
                RCLCPP_INFO(this->get_logger(),"Warning : Right motor speed too high : '%f'",RPMR);
                RPMR = MAX_RPM;
            }
            
            //choosing the right direction for each motors
            if(RPMR>0){
                gpio_write(gpio,DIRECTION_GPIO_MRIGHT,CW);
            }
            else{
                gpio_write(gpio, DIRECTION_GPIO_MRIGHT,CCW);
            }
            if(RPML>0){
                gpio_write(gpio, DIRECTION_GPIO_MLEFT,CCW);
            }
            else{
                gpio_write(gpio, DIRECTION_GPIO_MLEFT,CW);
            }

            //converting RPM's into percentage for pwm
            PWM_L = (RPML-MIN_RPM)*(MAX_PERCENTAGE-MIN_PERCENTAGE)/MAX_RPM + MIN_PERCENTAGE;
            PWM_R = (RPMR-MIN_RPM)*(MAX_PERCENTAGE-MIN_PERCENTAGE)/MAX_RPM + MIN_PERCENTAGE;
            ;
            //outputting the pwm duty cycle
            set_PWM_dutycycle(gpio, PWM_GPIO_MOTOR_LEFT,static_cast<size_t>(fabs(PWM_L)*255./100.)); //fully on is 1000000
            set_PWM_dutycycle(gpio, PWM_GPIO_MOTOR_RIGHT,static_cast<size_t>(fabs(PWM_R)*255./100.)); 
        }
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};
//--------------------------------------------------------------

//Main----------------------------------------------------------
int main(int argc, char **argv)
{
    // if(gpioInitialise()<0)
    // {
    //     cerr << "Failed to initialise gpio library" << endl;
    //     return 0;
    // }
    // cout << "initialisation success" << endl;
    gpio = pigpio_start("localhost", "8888");
    std::cerr << gpio << std::endl;
    set_mode(gpio, PWM_GPIO_MOTOR_LEFT,PI_ALT0);
    set_mode(gpio, PWM_GPIO_MOTOR_RIGHT,PI_ALT0);
    set_PWM_frequency(gpio, PWM_GPIO_MOTOR_LEFT, PWM_FREQ);
    set_PWM_frequency(gpio, PWM_GPIO_MOTOR_RIGHT, PWM_FREQ);


    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Motor_driver_subscriber>());

    rclcpp::shutdown();
    return 0;
}
//--------------------------------------------------------------