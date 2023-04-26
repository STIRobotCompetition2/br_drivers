//Includes------------------------------------------------------
#include <pigpiod_if2.h>

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <std_srvs/srv/set_bool.hpp>
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
#define STARTUP_GPIO 17
#define ON 1
#define OFF 0
#define GEAR_RATIO 60
#define LBWheels 0.42                                //in meters
#define RWheels 0.0575                              //in meters
// #define LBWheels 0.41                                //in meters
//#define MAX_RPM 5000/60
#define MAX_RPM 5000
#define MIN_RPM 0
#define MAX_PERCENTAGE 90
#define MIN_PERCENTAGE 9
#define CW 0
#define CCW 1
#define PWM_FREQ 100
#define STARTUP_WAIT_TIME 500
//--------------------------------------------------------------

//
#define SERVO_FREQ 50
#define SERVO_PIN 16
#define SERVO_UP 2500
#define SERVO_DOWN 500
//


//Global variables----------------------------------------------
bool startup = 0;
//--------------------------------------------------------------

using namespace std::placeholders;

//Function/class------------------------------------------------
class MotorDriver : public rclcpp::Node
{
    public:
        MotorDriver() 
        : Node("motor_driver")
        {
            gpio = pigpio_start("localhost", "8888");
            if(gpio < 0){
                RCLCPP_ERROR(this->get_logger(), "Cannot find GPIO daemon at localhost:8888 - Shutting down motor driver. Did you run 'sudo pigpiod'?");
                rclcpp::shutdown();
                return;
            }
            set_mode(gpio, PWM_GPIO_MOTOR_LEFT,PI_ALT0);
            set_mode(gpio, PWM_GPIO_MOTOR_RIGHT,PI_ALT0);
            set_mode(gpio, PWM_GPIO_MOTOR_RIGHT,PI_ALT0);
            set_mode(gpio, SERVO_PIN,PI_ALT0);

            set_mode(gpio, STARTUP_GPIO, PI_OUTPUT);
            set_PWM_frequency(gpio, PWM_GPIO_MOTOR_LEFT, PWM_FREQ);
            set_PWM_frequency(gpio, PWM_GPIO_MOTOR_RIGHT, PWM_FREQ);
            set_PWM_frequency(gpio, SERVO_PIN, SERVO_FREQ);


            RCLCPP_INFO(this->get_logger(),"Startup routine started\n");
            set_PWM_dutycycle(gpio, PWM_GPIO_MOTOR_LEFT,30); //fully on is 1000000
            set_PWM_dutycycle(gpio, PWM_GPIO_MOTOR_RIGHT,30);

            rclcpp::sleep_for(std::chrono::milliseconds(100));

            set_PWM_dutycycle(gpio, PWM_GPIO_MOTOR_LEFT,25); //fully on is 1000000
            set_PWM_dutycycle(gpio, PWM_GPIO_MOTOR_RIGHT,25);
            gpio_write(gpio,STARTUP_GPIO, OFF);
            rclcpp::sleep_for(std::chrono::milliseconds(STARTUP_WAIT_TIME));
            gpio_write(gpio,STARTUP_GPIO, ON);
            RCLCPP_INFO(this->get_logger(),"Startup complete\n check if the ESCON controller led is GREEN!\n");
            
            subscription_ = this-> create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10,std::bind(&MotorDriver::topic_callback, this, _1));
            servo_service_ = this->create_service<std_srvs::srv::SetBool>(
                "set_servo",
                std::bind(&MotorDriver::set_servo_callback, this, _1, _2, _3)
        );

        }

    private:
        double aVelL = 0.0;
        double aVelR = 0.0;
        double RPMR = 0.0;
        double RPML = 0.0;
        double PWM_L = 0.0;
        double PWM_R = 0.0;
        int gpio;

        void set_servo_callback(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
 
        {
            response->success = true;

            if(request->data){
                RCLCPP_INFO(this->get_logger(), "Opening Gate ...");
                rclcpp::sleep_for(std::chrono::milliseconds(1000));

                set_servo_pulsewidth(gpio, SERVO_PIN, SERVO_UP);
                response->message = "Gate is now open";

                
                RCLCPP_INFO(this->get_logger(), "... opened Gate !");
            }
            else {
                RCLCPP_INFO(this->get_logger(), "Closing Gate ...");

                set_servo_pulsewidth(gpio, SERVO_PIN, SERVO_DOWN);
                rclcpp::sleep_for(std::chrono::milliseconds(1000));
                response->message = "Gate is now closed";
                RCLCPP_INFO(this->get_logger(), "... closed Gate !");

                
            }
            

        }

        void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            //converting velocities into RPM's
            aVelL = msg->linear.x/RWheels - LBWheels*msg->angular.z/(2*RWheels);
            aVelR = msg->linear.x/RWheels + LBWheels*msg->angular.z/(2*RWheels);

            RPML = aVelL*60*GEAR_RATIO/(2*M_PI);
            RPMR = aVelR*60*GEAR_RATIO/(2*M_PI);

            //RCLCPP_INFO(this->get_logger(),"RPML : '%f'",RPML); //FOR DEBUGGING
            //RCLCPP_INFO(this->get_logger(),"RPMR : '%f'",RPMR); //FOR DEBUGGING

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
            PWM_L = (std::fabs(RPML)-MIN_RPM)*(MAX_PERCENTAGE-MIN_PERCENTAGE)/MAX_RPM + MIN_PERCENTAGE;
            PWM_R = (std::fabs(RPMR)-MIN_RPM)*(MAX_PERCENTAGE-MIN_PERCENTAGE)/MAX_RPM + MIN_PERCENTAGE;

            //RCLCPP_INFO(this->get_logger(),"The linear velocity : '%f'",msg->linear.x); //FOR DEBUGGING PURPOSE
            //RCLCPP_INFO(this->get_logger(),"The angular velocity : '%f'",msg->angular.z); //FOR DEBUGGING PURPOSE

            //RCLCPP_INFO(this->get_logger(),"The left PWM is set to : '%f'",PWM_L);      //FOR DEBUGGING PURPOSE
            //RCLCPP_INFO(this->get_logger(),"The Right PWM is set to : '%f' \n \n",PWM_R);     //FOR DEBUGGING PURPOSE
            
            //outputting the pwm duty cycle
            set_PWM_dutycycle(gpio, PWM_GPIO_MOTOR_LEFT,static_cast<size_t>(fabs(PWM_L)*255./100.)); //fully on is 1000000
            set_PWM_dutycycle(gpio, PWM_GPIO_MOTOR_RIGHT,static_cast<size_t>(fabs(PWM_R)*255./100.)); 
        }
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr servo_service_;
};
//--------------------------------------------------------------

//Main----------------------------------------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);

    MotorDriver::SharedPtr md(new MotorDriver());

    rclcpp::spin(md);

    rclcpp::shutdown();
    return 0;
}
//--------------------------------------------------------------