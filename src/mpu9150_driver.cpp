#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include <std_srvs/srv/trigger.hpp>

#include "linux/i2c-dev.h"
#include <fcntl.h>
#include <mutex>
#include <unistd.h>
#include <sys/ioctl.h>
#include <eigen3/Eigen/Geometry>
#define G 9.81
#define F 50.

using namespace std::placeholders;
class IMUNode : public rclcpp::Node
{
  public:
  IMUNode() : Node("imu_node"), fd(-1), omega(Eigen::Matrix4d::Zero()), gyro_offset(Eigen::Vector3d::Zero()), acc_offset(Eigen::Vector3d::Zero())
  {
    // Set up I2C connection to MPU9150
    fd = open("/dev/i2c-1", O_RDWR);
    ioctl(fd, I2C_SLAVE, 0x68);

    // Enable the accelerometer and gyroscope
    uint8_t buffer[2];
    buffer[0] = 0x6B; // PWR_MGMT_1 register
    buffer[1] = 0x00; // Set clock source to internal oscillator
    write(fd, buffer, 2);

    buffer[0] = 0x19; // Clock-Division
    buffer[1] = 0x07; // Sample-Rate 1000Hz
    write(fd, buffer, 2);

    buffer[0] = 0x1A; // Low-Pass
    buffer[1] = 0x02; // 94Hz Bandwidth -> ~3ms delay
    write(fd, buffer, 2);

    buffer[0] = 0x1B; // Gyro-Config
    buffer[1] = 0x00; // Full-Range (+- 250 deg per s)
    write(fd, buffer, 2);

    buffer[0] = 0x1C; // Gyro-Config
    buffer[1] = 0x00; // Full-Range (+- 2g)
    write(fd, buffer, 2);

    q << 1,0,0,0;

    acc_covariance.fill(0);
    acc_covariance.at(0) = std::pow(4 * G * 1e-3, 2);
    acc_covariance.at(4) = std::pow(4 * G * 1e-3, 2);
    acc_covariance.at(8) = std::pow(4 * G * 1e-3, 2);

    gyro_covariance.fill(0);
    gyro_covariance.at(0) = std::pow(0.6 * M_PI / 180 * 1e-3, 2);
    gyro_covariance.at(4) = std::pow(0.6 * M_PI / 180 * 1e-3, 2);
    gyro_covariance.at(8) = std::pow(0.6 * M_PI / 180 * 1e-3, 2);

    // Set up publisher for IMU data
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("data", 10);
    temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature", 10);


    // Set up timer for publishing IMU data
    
    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<size_t>(1e3 / F)), std::bind(&IMUNode::timerCallback, this));
    calib_server_ = this->create_service<std_srvs::srv::Trigger>("calibrate", std::bind(&IMUNode::calibrateCallback, this, _1, _2, _3));
  }

  void getData(sensor_msgs::msg::Imu& imu_msg, sensor_msgs::msg::Temperature& temp_msg, bool apply_offset = true){

        // Read data from MPU9150
        uint8_t buffer[14];
        buffer[0] = 0x3B;
        write(fd, buffer, 1);
        read(fd, buffer, 14);

        // Extract IMU data
        int16_t ax = buffer[0] << 8 | buffer[1];
        int16_t ay = buffer[2] << 8 | buffer[3];
        int16_t az = buffer[4] << 8 | buffer[5];
        int16_t temp = buffer[6] << 8 | buffer[7];
        int16_t gx = buffer[8] << 8 | buffer[9];
        int16_t gy = buffer[10] << 8 | buffer[11];
        int16_t gz = buffer[12] << 8 | buffer[13];

        Eigen::Vector3d a(
          ax / 16384.0 * G,
          ay / 16384.0 * G,
          az / 16384.0 * G
          );
        

        Eigen::Vector3d w(
          gx / 131.0 * M_PI / 180.0, 
          gy / 131.0 * M_PI / 180.0, 
          gz / 131.0 * M_PI / 180.0
          );
        
        if(apply_offset){
          w -= gyro_offset;
          a -= acc_offset;
        }


        // https://ahrs.readthedocs.io/en/latest/filters/angular.html
        omega <<  0, -w(0), -w(1), -w(2),
                  w(0), 0, w(2), -w(1),
                  w(1), -w(2), 0, w(0),
                  w(2), w(1), -w(0), 0;

        double w_norm = w.norm();
        q =  (1 / w_norm * sin(w_norm * 1. / F / 2.) * omega + Eigen::Matrix4d::Identity() * std::cos(w_norm * 1. / F / 2.)) * q;
        q /= q.norm();

        // Create IMU message
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.stamp.nanosec -= 3e6;    // Due to delay due to low-pass filtering
        imu_msg.header.frame_id = "imu_sensor_link";
        imu_msg.angular_velocity.x = w.x();
        imu_msg.angular_velocity.y = w.y();
        imu_msg.angular_velocity.z = w.z();
        imu_msg.linear_acceleration.x = a.x();
        imu_msg.linear_acceleration.y = a.y();
        imu_msg.linear_acceleration.z = a.z();

        imu_msg.orientation.x = q(1);
        imu_msg.orientation.y = q(2);
        imu_msg.orientation.z = q(3);
        imu_msg.orientation.w = q(0);


        imu_msg.angular_velocity_covariance = gyro_covariance;
        imu_msg.linear_acceleration_covariance = acc_covariance;

        // Publish IMU message

        temp_msg.header = imu_msg.header;
        temp_msg.temperature = temp / 340. + 35.;
        temp_msg.variance = -1;
  }

  void calibrateCallback( 
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            const std::shared_ptr<std_srvs::srv::Trigger::Response> response
            ){
    i2c_lock.lock();
    size_t n_measurements = 500;

    acc_offset = Eigen::Vector3d::Zero();
    gyro_offset = Eigen::Vector3d::Zero();

    rclcpp::Rate rate(100);
    for(size_t i = 0; i < n_measurements; i++){
      sensor_msgs::msg::Imu imu_msg;
      sensor_msgs::msg::Temperature temp_msg;
      getData(imu_msg, temp_msg, false);

      acc_offset.x() += imu_msg.linear_acceleration.x;
      acc_offset.y() += imu_msg.linear_acceleration.y;
      acc_offset.z() += imu_msg.linear_acceleration.z;
      gyro_offset.x() += imu_msg.angular_velocity.x;
      gyro_offset.y() += imu_msg.angular_velocity.y;
      gyro_offset.z() += imu_msg.angular_velocity.z;

      rate.sleep();
    }
    acc_offset /= static_cast<double>(n_measurements);
    gyro_offset /= static_cast<double>(n_measurements);
    
    q << 1,0,0,0;
    i2c_lock.unlock();
    response->success = true;
    return;
  }

  void timerCallback() {
        if(!i2c_lock.try_lock()) return;
        sensor_msgs::msg::Imu imu_msg;
        sensor_msgs::msg::Temperature temp_msg;
        getData(imu_msg, temp_msg);
        // Publish IMU message
        imu_pub_->publish(imu_msg);
        temp_pub_->publish(temp_msg);
        i2c_lock.unlock();
    }

  private:
  int fd;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calib_server_;

  rclcpp::TimerBase::SharedPtr timer_;
  Eigen::Vector4d q;
  Eigen::Vector3d gyro_offset;
  Eigen::Vector3d acc_offset;


  Eigen::Matrix4d omega;
  std::array<double, 9u> acc_covariance;
  std::array<double, 9u> gyro_covariance;
  std::mutex i2c_lock;

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IMUNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}