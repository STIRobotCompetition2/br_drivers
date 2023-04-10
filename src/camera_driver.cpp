#include "opencv2/opencv.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/header.hpp>



class CameraDriver : public rclcpp::Node {
    public:
    CameraDriver() : Node("camera_driver"), publish_raw(true), publish_compressed(true){
        vid_in = std::make_shared<cv::VideoCapture>(cv::VideoCapture(0));
        image_raw_pub = this->create_publisher<sensor_msgs::msg::Image>("/camera/image", 1);
        image_compressed_pub = this->create_publisher<sensor_msgs::msg::CompressedImage>("/camera/image/compressed", 1);
        this->run();
    }
    private:
    void run(){
        rclcpp::Rate video_rate(5);
        cv::Mat cv_image;
        
        cv_bridge::CvImage cv_bridge_image;
        cv_bridge_image.encoding = "bgr8";
        sensor_msgs::msg::Image ros_image_raw;
        sensor_msgs::msg::CompressedImage ros_image_compressed;
        std_msgs::msg::Header header;
        header.frame_id = "camera_sensor_link";


        while(vid_in->isOpened() && rclcpp::ok()){
            *vid_in >> cv_image;
            if(publish_raw || publish_compressed){
                cv_bridge_image.image = cv_image;
                header.stamp = this->get_clock()->now();
            }
            if(publish_raw){
                cv_bridge_image.toImageMsg(ros_image_raw);
                ros_image_raw.encoding = sensor_msgs::image_encodings::BGR8;
                ros_image_raw.header = header;

                image_raw_pub->publish(ros_image_raw);
            }
            if(publish_compressed){
                cv_bridge_image.toCompressedImageMsg(ros_image_compressed, cv_bridge::JPEG);
                ros_image_compressed.format = static_cast<std::string>("jpeg");
                ros_image_compressed.header = header;

                
                image_compressed_pub->publish(ros_image_compressed);
            }

            if(publish_raw || publish_compressed) RCLCPP_DEBUG(this->get_logger(), "Driver sent picture");
            else RCLCPP_DEBUG(this->get_logger(), "Driver sent picture no picture although loop was executed (no pub-flag true)");

        }


    }
    std::shared_ptr<cv::VideoCapture> vid_in;
    double rate;
    bool publish_raw;
    bool publish_compressed;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_raw_pub;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_compressed_pub;


};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    CameraDriver::SharedPtr cd(new CameraDriver());
    rclcpp::spin(cd);
}