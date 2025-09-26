// src/camera_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <atomic>
#include <memory>
#include <string>

using std::placeholders::_1;

class CameraNode : public rclcpp::Node {
public:
    CameraNode()
    : Node("camera_node"),
      width_(640), height_(480), fps_(30), frame_id_("camera_frame"), pipeline_(nullptr), appsink_(nullptr)
    {
        // Initialize GStreamer (safe to call multiple times)
        gst_init(nullptr, nullptr);

        // Build pipeline string: replace videotestsrc with your v4l2src if needed
        // This pipeline produces BGR frames (we ask videoconvert to BGR for CV)
        std::string pipeline_str =
            "videotestsrc is-live=true ! "
            "video/x-raw,width=" + std::to_string(width_) + ",height=" + std::to_string(height_) + ",framerate=" + std::to_string((int)fps_) + "/1 ! "
            " nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! "
            "appsink name=appsink0 emit-signals=true sync=false max-buffers=2 drop=true";

        GError* error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
        if (!pipeline_ || error) {
            RCLCPP_FATAL(this->get_logger(), "Failed to create pipeline: %s", error ? error->message : "unknown");
            if (error) g_error_free(error);
            throw std::runtime_error("Failed to create pipeline");
        }

        // Get appsink element handle
        appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "appsink0");
        if (!appsink_) {
            RCLCPP_FATAL(this->get_logger(), "Failed to get appsink element from pipeline");
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
            throw std::runtime_error("Failed to get appsink");
        }

        // Configure appsink: emit-signals already set; set caps (optional but explicit)
        GstCaps* caps = gst_caps_new_simple(
            "video/x-raw",
            "format", G_TYPE_STRING, "BGR",
            "width", G_TYPE_INT, width_,
            "height", G_TYPE_INT, height_,
            "framerate", GST_TYPE_FRACTION, (gint)fps_, 1,
            nullptr);
        g_object_set(appsink_, "caps", caps, nullptr);
        gst_caps_unref(caps);

        // Connect the new-sample signal to our callback (runs in GStreamer thread)
        g_signal_connect(appsink_, "new-sample", G_CALLBACK(&CameraNode::on_new_sample_static), this);

        // Create ROS publisher (latching not desired; QoS keep last small)
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", rclcpp::QoS(10));

        // Set pipeline to PLAYING
        gst_element_set_state(pipeline_, GST_STATE_PLAYING);

        RCLCPP_INFO(this->get_logger(), "CameraNode started, publishing on 'camera/image_raw'");
    }

    ~CameraNode() override {
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
        }
        if (appsink_) {
            gst_object_unref(appsink_);
            appsink_ = nullptr;
        }
        if (pipeline_) {
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
        }
    }

private:
    // Static trampoline because GStreamer expects a C callback
    static GstFlowReturn on_new_sample_static(GstAppSink *appsink, gpointer user_data) {
        CameraNode* self = static_cast<CameraNode*>(user_data);
        return self->on_new_sample(appsink);
    }

    // Instance callback: called from GStreamer thread when a frame is ready
    GstFlowReturn on_new_sample(GstAppSink *appsink) {
        // Pull sample (non-blocking; we are in the signal context)
        GstSample* sample = gst_app_sink_pull_sample(appsink);
        if (!sample) return GST_FLOW_OK; // nothing to do

        GstBuffer* buffer = gst_sample_get_buffer(sample);
        GstCaps* caps = gst_sample_get_caps(sample);

        // Map buffer for read-only access
        GstMapInfo map;
        if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            gst_sample_unref(sample);
            return GST_FLOW_OK;
        }

        // Wrap mapped data into cv::Mat (no copy)
        // Note: be careful with lifetime - we copy into ROS msg below
        cv::Mat frame(height_, width_, CV_8UC3, (void*)map.data);

        // Convert to ROS Image message using cv_bridge (this copies data into msg->data)
        // We publish with current ROS time to ensure receiver timestamps are consistent
        auto cv_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame);
        auto msg = cv_img.toImageMsg();
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = frame_id_;

        // Publish (rclcpp is thread-safe for publishers)
        pub_->publish(*msg);

        // cleanup
        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);

        return GST_FLOW_OK;
    }

    // Member variables
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    GstElement* pipeline_;
    GstElement* appsink_;
    int width_, height_;
    int fps_;
    std::string frame_id_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
