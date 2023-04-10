#include "opencv2/opencv.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>


class CameraDriver : public rclcpp::Node {
    public:
    CameraDriver() : Node("camera_driver"){
        vid_in = std::make_shared<cv::VideoCapture>(cv::VideoCapture(0));
        image_pub = this->create_publisher<sensor_msgs::msg::Image>("/image", 1);
        this->run();
    }
    private:
    void run(){
        rclcpp::Rate video_rate(5);
        cv::Mat cv_image;
        cv_bridge::CvImage cv_bridge_image;
        sensor_msgs::msg::Image ros_image;

        while(vid_in->isOpened() && rclcpp::ok()){
            *vid_in >> cv_image;
            cv_bridge_image.image = cv_image;
            cv_bridge_image.toImageMsg(ros_image);
            ros_image.encoding = sensor_msgs::image_encodings::BGR8;
            image_pub->publish(ros_image);

        }


    }
    std::shared_ptr<cv::VideoCapture> vid_in;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;

};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    CameraDriver::SharedPtr cd(new CameraDriver());
    rclcpp::spin(cd);
}