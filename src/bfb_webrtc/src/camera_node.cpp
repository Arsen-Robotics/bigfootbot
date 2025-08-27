#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraNode : public rclcpp::Node {
public:
    CameraNode() : Node("camera_node") {
        // Declare parameters
        this->declare_parameter<std::string>("device_path", "/dev/cam-arducam");
        this->declare_parameter<int>("width", 640);
        this->declare_parameter<int>("height", 480);
        this->declare_parameter<double>("fps", 30.0);
        this->declare_parameter<std::string>("frame_id", "camera_frame");
        
        // Get parameters
        std::string device_path = this->get_parameter("device_path").as_string();
        int width = this->get_parameter("width").as_int();
        int height = this->get_parameter("height").as_int();
        double fps = this->get_parameter("fps").as_double();
        frame_id = this->get_parameter("frame_id").as_string();
        std::string topic_name = "camera/image/raw";
        
        // Initialize camera
        cap = std::make_shared<cv::VideoCapture>(device_path, cv::CAP_V4L2);

        if (!cap->isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera device %s", device_path.c_str());
            throw std::runtime_error("Failed to open camera");
        }

        // Set camera properties
        cap->set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap->set(cv::CAP_PROP_FRAME_HEIGHT, height);
        cap->set(cv::CAP_PROP_FPS, fps);

        // Create publisher
        pub = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);

        // Start timer to capture and publish images
        timer = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / fps)),
            std::bind(&CameraNode::capture_and_publish, this)
        );

        RCLCPP_INFO(this->get_logger(), "Camera Node initialized.");
        RCLCPP_INFO(this->get_logger(), "Device: %s, Resolution: %dx%d, FPS: %.1f", device_path.c_str(), width, height, fps);
    }

    ~CameraNode() {
        if (cap && cap->isOpened()) {
            cap->release();
        }
    }

private:
    void capture_and_publish() {
        cv::Mat frame;
        if (!cap->read(frame)) {
            RCLCPP_WARN(this->get_logger(), "Failed to capture image from camera");
            return;
        }

        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Captured empty frame");
            return;
        }

        // Convert OpenCV Mat to ROS2 Image message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = frame_id;
        
        // Publish the image
        pub->publish(*msg);
    }

    std::shared_ptr<cv::VideoCapture> cap;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer;
    std::string frame_id;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<CameraNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("camera_publisher"), "Exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}