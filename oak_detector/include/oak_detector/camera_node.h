#pragma once

#include "rclcpp/rclcpp.hpp"

#include "depthai_ros_msgs/msg/spatial_detection.hpp"
#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"

#include "depthai/pipeline/datatypes.hpp"

#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImgDetectionConverter.hpp"

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "oak_detector/pipeline.h"

class CameraNode : public rclcpp::Node {
  std::shared_ptr<dai::DataOutputQueue> sys_logger_queue_;
  std::shared_ptr<dai::DataOutputQueue> preview_queue_;
  std::shared_ptr<dai::DataOutputQueue> nnet_data_qeue_;
  std::shared_ptr<dai::DataOutputQueue> color_encoder_queue_;
  std::shared_ptr<dai::DataOutputQueue> mono_encoder_queue_;
  std::shared_ptr<dai::DataOutputQueue> depth_queue_;
  std::shared_ptr<dai::DataOutputQueue> tracker_queue_;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> compressed_image_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> preview_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> compressed_mono_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> depth_pub_;
  std::shared_ptr<rclcpp::Publisher<depthai_ros_msgs::msg::SpatialDetectionArray>>
      object_tracker_pub_;

  dai::Pipeline pipeline_;
  std::shared_ptr<dai::Device> device_;

  dai::rosBridge::ImageConverter img_converter_ = dai::rosBridge::ImageConverter("");

public:
  CameraNode(const std::string &name);
};