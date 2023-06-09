#include <cstdio>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "depthai_ros_msgs/msg/spatial_detection.hpp"
#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"

#include "cv_bridge/cv_bridge.h"

#include <chrono>

using namespace std::chrono_literals;

using depthai_ros_msgs::msg::SpatialDetectionArray;
using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::Image;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("oak_detector_vis_node");

  RCLCPP_INFO(node->get_logger(), "creating subscribers...");

  std::mutex last_detection_mutex;
  SpatialDetectionArray last_detections;

  // pubs
  auto detections_pub = node->create_publisher<CompressedImage>(
      "/oak_detector/object_tracker_vis/detections/compressed", rclcpp::SensorDataQoS());

  // subs
  auto detections_sub = node->create_subscription<SpatialDetectionArray>(
      "/oak_detector/object_tracker/detections", rclcpp::SensorDataQoS(),
      [&](const SpatialDetectionArray &msg) {
        std::scoped_lock lock(last_detection_mutex);
        last_detections = msg;
      });

  auto preview_sub = node->create_subscription<CompressedImage>(
      "/oak_detector/color/compressed", rclcpp::SensorDataQoS(), [&](const CompressedImage &msg) {
        std::scoped_lock lock(last_detection_mutex);
        auto cv_img = cv_bridge::toCvCopy(msg);

        for (const auto &detection : last_detections.detections) {
          auto bb = detection.bbox;

          const auto width = cv_img->image.cols;
          const auto height = cv_img->image.rows;

          cv::Rect2i roi;
          roi.x = (bb.center.position.x - bb.size_x / 2.0f) * width;
          roi.y = (bb.center.position.y - bb.size_y / 2.0f) * height;
          roi.width = bb.size_x * width;
          roi.height = bb.size_y * height;

          cv::rectangle(cv_img->image, roi, cv::Scalar(0, 0, 200), 2);

          RCLCPP_INFO(node->get_logger(), "%i %i", roi.x, roi.y);
        }

        CompressedImage ros_img;
        cv_img->toCompressedImageMsg(ros_img);
        detections_pub->publish(ros_img);
      });

  RCLCPP_INFO(node->get_logger(), "spin...");
  rclcpp::spin(node);
}