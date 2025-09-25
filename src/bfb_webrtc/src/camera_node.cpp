#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>

class CameraNode : public rclcpp::Node {
public:
    CameraNode() : Node("camera_node") {
        width = 640;
        height = 480;
        fps = 30.0;
        frame_id = "camera_frame";

        gst_init(nullptr, nullptr);

        // Simple pipeline: videotestsrc -> videoconvert -> appsink
        std::string pipeline_str =
            "videotestsrc is-live=true ! "
            "video/x-raw,width=640,height=480,framerate=30/1 ! "
            "videoconvert ! video/x-raw,format=BGR ! "
            "appsink name=appsink0 emit-signals=true sync=false";

        pipeline = gst_parse_launch(pipeline_str.c_str(), nullptr);
        appsink = gst_bin_get_by_name(GST_BIN(pipeline), "appsink0");
        gst_element_set_state(pipeline, GST_STATE_PLAYING);

        pub = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

        timer = this->create_wall_timer(
            std::chrono::milliseconds(33),  // ~30 FPS
            [this]() { capture_and_publish(); }
        );
    }

    ~CameraNode() {
        if (pipeline) {
            gst_element_set_state(pipeline, GST_STATE_NULL);
            gst_object_unref(pipeline);
        }
    }

private:
    void capture_and_publish() {
        GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink), GST_SECOND / 2);
        if (!sample) return;

        GstBuffer* buffer = gst_sample_get_buffer(sample);
        GstMapInfo map;
        if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            gst_sample_unref(sample);
            return;
        }

        cv::Mat frame(height, width, CV_8UC3, map.data);
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = frame_id;
        pub->publish(*msg);

        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer;
    GstElement* pipeline;
    GstElement* appsink;
    int width, height;
    double fps;
    std::string frame_id;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
