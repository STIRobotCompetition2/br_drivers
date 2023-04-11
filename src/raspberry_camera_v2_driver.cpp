#include "opencv2/opencv.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/header.hpp>
#include <lccv.hpp>


class CameraDriver : public rclcpp::Node {
    public:
    CameraDriver() : Node("camera_driver"), publish_raw(true), publish_compressed(true){
        // Configure Camera
        this->declare_parameter("libcamera_resolution_width", 1920);
        this->declare_parameter("libcamera_resolution_height", 1080);
        this->declare_parameter("libcamera_resolution_framerate", 30);
        this->declare_parameter("libcamera_verbose", true);



        cam.options->video_width=this->get_parameter("libcamera_resolution_width").as_int();
        cam.options->video_height=this->get_parameter("libcamera_resolution_height").as_int();
        cam.options->framerate=this->get_parameter("libcamera_resolution_framerate").as_int();
        cam.options->verbose=this->get_parameter("libcamera_verbose").as_bool();
        cam.startVideo();
        // Somehow necessary to start the stream ...
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        if(!cam.getVideoFrame(cv_image,5000)) {
            RCLCPP_ERROR(this->get_logger(), "Camera did not open after 5s wait. Aborting ...");
            rclcpp::shutdown();
        }

        // Configure ROS
        this->declare_parameter("frame_id", "camera_sensor_link");
        this->declare_parameter("sample_period", 0.2);

        this->declare_parameter("publish_image_raw", true);
        publish_raw = this->get_parameter("publish_image_raw").as_bool();
        if(publish_raw) this->declare_parameter("image_raw_topic", "/camera/image");

        this->declare_parameter("publish_image_compressed", true);
        publish_compressed = this->get_parameter("publish_image_compressed").as_bool();
        if(publish_compressed) this->declare_parameter("image_compressed_topic", "/camera/image/compressed");

        



        cv_bridge_image.encoding = "bgr8";
        header.frame_id = this->get_parameter("frame_id").as_string();
        if(publish_raw) image_raw_pub = this->create_publisher<sensor_msgs::msg::Image>(this->get_parameter("image_raw_topic").as_string(), 10);
        if(publish_compressed) image_compressed_pub = this->create_publisher<sensor_msgs::msg::CompressedImage>(this->get_parameter("image_compressed_topic").as_string(), 10);
        sample_camera_timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<size_t>(1e3 * this->get_parameter("sample_period").as_double())), std::bind(&CameraDriver::sampleCameraCallback, this));
        RCLCPP_INFO(this->get_logger(), "Camera driver running");
    }
    ~CameraDriver(){
        sample_camera_timer->cancel();
        cam.stopVideo();
        RCLCPP_INFO(this->get_logger(), "Camera Driver shut down properly");
    }

    private:
    void sampleCameraCallback(){
        if(!cam.getVideoFrame(cv_image,1000)){
            RCLCPP_WARN(this->get_logger(), "Camera could not fetch image (but this worked at least once during initialization)");
            return;
        }
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
    lccv::PiCamera cam;
    double rate;
    bool publish_raw;
    bool publish_compressed;
    cv::Mat cv_image;
        
    cv_bridge::CvImage cv_bridge_image;
    sensor_msgs::msg::Image ros_image_raw;
    sensor_msgs::msg::CompressedImage ros_image_compressed;
    std_msgs::msg::Header header;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_raw_pub;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_compressed_pub;
    rclcpp::TimerBase::SharedPtr sample_camera_timer;


};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    CameraDriver::SharedPtr cd(new CameraDriver());
    rclcpp::spin(cd);
}