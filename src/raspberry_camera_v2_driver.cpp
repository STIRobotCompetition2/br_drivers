#include "opencv2/opencv.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <chrono>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/header.hpp>
#include <lccv.hpp>
#include <mutex>
#include <opencv2/aruco.hpp>
#include <eigen3/Eigen/Geometry>


#include <br_drivers/srv/calibrate_camera_extrinsically.hpp>

using namespace std::placeholders;

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
        if(!cam.getVideoFrame(cv_image_raw,5000)) {
            RCLCPP_ERROR(this->get_logger(), "Camera did not open after 5s wait. Aborting ...");
            rclcpp::shutdown();
        }

        // Configure ROS
        this->declare_parameter("frame_id", "camera_sensor_link");
        this->declare_parameter("ros_sample_period", 0.2);
        this->declare_parameter("ros_resolution_height", 1080);
        this->declare_parameter("ros_resolution_width", 30);

        this->declare_parameter("publish_image_raw", true);
        publish_raw = this->get_parameter("publish_image_raw").as_bool();
        if(publish_raw) this->declare_parameter("image_raw_topic", "/camera/image");

        this->declare_parameter("publish_image_compressed", true);
        publish_compressed = this->get_parameter("publish_image_compressed").as_bool();
        if(publish_compressed) this->declare_parameter("image_compressed_topic", "/camera/image/compressed");

        ros_resolution_height = this->get_parameter("ros_resolution_height").as_int();
        ros_resolution_width = this->get_parameter("ros_resolution_width").as_int();




        cv_bridge_image.encoding = "bgr8";
        header.frame_id = this->get_parameter("frame_id").as_string();
        rclcpp::SensorDataQoS qos_config;
        

        if(publish_raw) image_raw_pub = this->create_publisher<sensor_msgs::msg::Image>(this->get_parameter("image_raw_topic").as_string(), rclcpp::SensorDataQoS());
        if(publish_compressed) image_compressed_pub = this->create_publisher<sensor_msgs::msg::CompressedImage>(this->get_parameter("image_compressed_topic").as_string(), rclcpp::SensorDataQoS());
        sample_camera_timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<size_t>(1e3 * this->get_parameter("ros_sample_period").as_double())), std::bind(&CameraDriver::sampleCameraCallback, this));
        calib_server_ = this->create_service<br_drivers::srv::CalibrateCameraExtrinsically>("/calibrate_ext", std::bind(&CameraDriver::calibrateCameraExtrinsicallyCallback, this, _1, _2, _3));

        RCLCPP_INFO(this->get_logger(), "Camera driver running !");
    }
    ~CameraDriver(){
        sample_camera_timer->cancel();
        cam.stopVideo();
        RCLCPP_INFO(this->get_logger(), "Camera Driver shut down properly");
    }

    private:
    void sampleCameraCallback(){
        if(!camera_lock.try_lock()){
            return;
        }
        if(!cam.getVideoFrame(cv_image_raw,1000)){
            RCLCPP_WARN(this->get_logger(), "Camera could not fetch image (but this worked at least once during initialization)");
            return;
        }
        if(publish_raw || publish_compressed){
            if(warp_available) cv::warpPerspective(cv_image_raw, cv_image_raw, warp_transform, cv::Size(cam.options->video_width, cam.options->video_height));
            cv::resize(cv_image_raw, cv_image_ds, cv::Size(ros_resolution_width,ros_resolution_height), cv::INTER_LINEAR);
            cv_bridge_image.image = cv_image_ds;
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
        camera_lock.unlock();

    }

    // void calibrateCameraIntrinsicallyCallback(){

    // }

    void calibrateCameraExtrinsicallyCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<br_drivers::srv::CalibrateCameraExtrinsically::Request> request,
        const std::shared_ptr<br_drivers::srv::CalibrateCameraExtrinsically::Response> response
    ){
        camera_lock.lock();

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

        // cv_image_raw = cv::imread("/home/budget_roomba_2/sven_space/ros_ws/src/camera_calib.jpg", cv::IMREAD_COLOR);
        cv::aruco::detectMarkers(cv_image_raw, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
        std::array<cv::Point2f, 4u> border_points, dst_points;
        for(size_t id = 0; id < markerIds.size(); id++){
            if(markerIds.at(id) == 0) 
                border_points.at(0) = markerCorners.at(id).at(3);
            if(markerIds.at(id) == request->number_in_y - 1) 
                border_points.at(1) = markerCorners.at(id).at(2);
            if(markerIds.at(id) == request->number_in_y * (request->number_in_x - 1)) 
                border_points.at(2) = markerCorners.at(id).at(0);
            if(markerIds.at(id) == request->number_in_y * request->number_in_x - 1) 
                border_points.at(3) = markerCorners.at(id).at(1);
        }
        // size_t image_center_x = cv_image_raw.size().width / 2;
        // size_t image_center_y = cv_image_raw.size().height / 2;

        dst_points.at(0).x = std::min(border_points.at(0).x, border_points.at(2).x);
        dst_points.at(2).x = dst_points.at(0).x;
        dst_points.at(1).x = std::max(border_points.at(1).x, border_points.at(3).x);
        dst_points.at(3).x = dst_points.at(1).x;

        dst_points.at(0).y = std::max(border_points.at(0).y, border_points.at(1).y);
        dst_points.at(1).y = dst_points.at(0).y;
        dst_points.at(2).y = std::min(border_points.at(2).y, border_points.at(3).y);
        dst_points.at(3).y = dst_points.at(2).y;
        // Inverted Logic (1 step in x-direction of pattern = 1 step in y-direction of image)
        double ratio = static_cast<double>(request->number_in_y * request->spacing_y_mm + request->corner_dist_mm) / static_cast<double>(request->number_in_x * request->spacing_x_mm + request->corner_dist_mm);
        size_t delta_x = dst_points.at(1).x - dst_points.at(0).x;
        size_t delta_y = dst_points.at(0).y - dst_points.at(2).y;
        if(delta_x > ratio * delta_y){
            delta_x = static_cast<size_t>(ratio * delta_y);
            size_t midpoint = (dst_points.at(1).x + dst_points.at(0).x) / 2;
            dst_points.at(0).x = midpoint - delta_x / 2;
            dst_points.at(2).x = midpoint - delta_x / 2;
            dst_points.at(1).x = midpoint + delta_x / 2;
            dst_points.at(3).x = midpoint + delta_x / 2;
        }
        else{
            delta_y = static_cast<size_t>(delta_x / ratio);
            size_t midpoint = (dst_points.at(0).y + dst_points.at(2).y) / 2;
            dst_points.at(0).y = midpoint + delta_y / 2;
            dst_points.at(1).y = midpoint + delta_y / 2;
            dst_points.at(2).y = midpoint - delta_y / 2;
            dst_points.at(3).y = midpoint - delta_y / 2;
        }

        delta_x = dst_points.at(1).x - dst_points.at(0).x;
        delta_y = dst_points.at(0).y - dst_points.at(2).y;
        std::cerr << "dx: " << delta_x << std::endl;
        std::cerr << "dy: " << delta_y << std::endl;
        std::cerr << "ra: " << ratio << std::endl;




        warp_transform = cv::getPerspectiveTransform(border_points, dst_points);
        warp_available = true;
        std::cerr << "W T: " << warp_transform << std::endl;



        // // Construct Dataset
        // Eigen::MatrixXd Y, X1, X2;;
        // Y.resize(markerIds.size() * 4, 2);
        // X1.resize(markerIds.size() * 4, 2);
        // X2.resize(markerIds.size() * 4, 2);
        // X1.col(1).fill(1);
        // X2.col(1).fill(1);

        
        // double y_offset_mm = -1 * (request->number_in_y - 1.) / 2. * request->spacing_y_mm;
        // for(size_t id = 0; id < markerIds.size(); id++){
        //     Eigen::Vector2d center;
        //     center.x() = request->x_offset_mm + static_cast<double>(markerIds.at(id) / static_cast<size_t>(request->number_in_y)) * request->spacing_x_mm;
        //     center.y() = -1 * (y_offset_mm + static_cast<double>(markerIds.at(id) % static_cast<size_t>(request->number_in_y)) * request->spacing_y_mm);
        //     std::cerr << "ID " << markerIds.at(id) << " CTR: " << center.transpose() << std::endl << std::endl;
        //     for(size_t c = 0; c < 4; c++){
        //         Y.row(c + id * 4) = center + Eigen::Vector2d(1,0) * (c < 2 ? 1 : -1) * request->corner_dist_mm / 2.0 + Eigen::Vector2d(0,1) * (c % 3 == 0 ? 1 : -1) * request->corner_dist_mm / 2.0;
        //         std::cerr << "\t\t<<>> " << c << " CRNR GT: " << Y.row(c + id * 4) << std::endl;
        //         X1(c + id * 4, 0) = markerCorners.at(id).at(c).y;
        //         X2(c + id * 4, 0) = markerCorners.at(id).at(c).x;
        //         std::cerr << "\t\t<<>> " << c << " CRNR DT: " << X1(c + id * 4, 0) << " " << X2(c + id * 4, 0) << std::endl;

        //     }
        // }


        // // Perform Linear Regression
        // Eigen::Vector2d sol_x = (X1.transpose() * X1).inverse() * X1.transpose() * Y.col(0);
        // Eigen::Vector2d sol_y = (X2.transpose() * X2).inverse() * X2.transpose() * Y.col(1);
        
        // std::cerr << "SOLX: " << sol_x.transpose() << std::endl;
        // std::cerr << "SOLY: " << sol_y.transpose() << std::endl;
        // // std::cerr << X1 << std::endl;
        // // std::cerr << Y << std::endl;
        // std::cerr << "X ex: " << X1.block<10,2>(0,0).transpose() << std::endl;
        // std::cerr << "SOLX ex: " << sol_x.transpose() * X1.block<10,2>(0,0).transpose() << std::endl;
        // std::cerr << "GTX " << Y.col(0).block<10,1>(0,0) << std::endl;

        // std::cerr << "SOLY ex: " << sol_y.transpose() * X2.block<10,2>(0,0).transpose() << std::endl;
        // std::cerr << "GTY " << Y.col(1).block<10,1>(0,0) << std::endl;



        
        camera_lock.unlock();
    }

    lccv::PiCamera cam;
    double rate;
    bool publish_raw;
    bool publish_compressed;
    cv::Mat cv_image_raw, cv_image_ds;

    cv::Mat warp_transform;
    bool warp_available = false;
        
    cv_bridge::CvImage cv_bridge_image;
    sensor_msgs::msg::Image ros_image_raw;
    sensor_msgs::msg::CompressedImage ros_image_compressed;
    std_msgs::msg::Header header;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_raw_pub;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_compressed_pub;
    rclcpp::TimerBase::SharedPtr sample_camera_timer;

    std::mutex camera_lock;
    rclcpp::Service<br_drivers::srv::CalibrateCameraExtrinsically>::SharedPtr calib_server_;
    size_t ros_resolution_width;
    size_t ros_resolution_height;




};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    CameraDriver::SharedPtr cd(new CameraDriver());
    rclcpp::spin(cd);
}