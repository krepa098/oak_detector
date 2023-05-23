#include <cstdio>
#include <iostream>

#include "oak_detector/pipeline.h"

#include "rclcpp/rclcpp.hpp"

#include "depthai/pipeline/datatypes.hpp"

#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImgDetectionConverter.hpp"

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "depthai_ros_msgs/msg/spatial_detection.hpp"
#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"

#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("oak_detector_node");
  auto compressed_image_pub = node->create_publisher<sensor_msgs::msg::CompressedImage>(
      "color/compressed", rclcpp::SensorDataQoS());

  auto preview_pub =
      node->create_publisher<sensor_msgs::msg::Image>("preview/image", rclcpp::SensorDataQoS());

  auto compressed_mono_pub = node->create_publisher<sensor_msgs::msg::CompressedImage>(
      "mono/compressed", rclcpp::SensorDataQoS());

  auto depth_pub =
      node->create_publisher<sensor_msgs::msg::Image>("depth/image", rclcpp::SensorDataQoS());

  auto object_tracker_pub = node->create_publisher<depthai_ros_msgs::msg::SpatialDetectionArray>(
      "object_tracker/detections", rclcpp::SensorDataQoS());

  RCLCPP_INFO(node->get_logger(), "creating pipeline");
  dai::Pipeline pipeline = createPipeline(BLOB_PATH);

  RCLCPP_INFO(node->get_logger(), "opening device");
  dai::Device device(pipeline);
  auto cal = device.getDeviceInfo();

  // enable laser projector
  device.setIrLaserDotProjectorBrightness(0.0);
  // device.setLogLevel(dai::LogLevel::DEBUG);
  // device.setLogOutputLevel(dai::LogLevel::DEBUG);

  // create queues
  // each queue has its own thread
  RCLCPP_INFO(node->get_logger(), "creating output queues");
  std::shared_ptr<dai::DataOutputQueue> sys_logger_queue =
      device.getOutputQueue("sys_logger", 4, false);
  std::shared_ptr<dai::DataOutputQueue> preview_queue = device.getOutputQueue("preview", 15, false);
  std::shared_ptr<dai::DataOutputQueue> nnet_data_qeue =
      device.getOutputQueue("detections", 4, false);
  std::shared_ptr<dai::DataOutputQueue> color_encoder_queue =
      device.getOutputQueue("color_mjpeg", 30, false);
  std::shared_ptr<dai::DataOutputQueue> mono_encoder_queue =
      device.getOutputQueue("mono_mjpeg", 15, false);
  std::shared_ptr<dai::DataOutputQueue> depth_queue = device.getOutputQueue("depth", 15, false);
  std::shared_ptr<dai::DataOutputQueue> tracker_queue =
      device.getOutputQueue("object_tracker", 4, false);

  // ros bridge
  dai::rosBridge::ImageConverter img_converter("rgb_frame");
  dai::rosBridge::ImageConverter depth_img_converter("depth_frame");
  auto rgb_cam_info =
      img_converter.calibrationToCameraInfo(device.readCalibration(), dai::CameraBoardSocket::RGB);

  //   dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>
  //       rgb_publish(preview_queue, node, std::string("color/image"),
  //                   std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
  //                             &img_converter, std::placeholders::_1,
  //                             std::placeholders::_2),
  //                   30, rgb_cam_info, "color");

  //   dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>
  //       mjpeg_publish(encoder_queue, node,
  //       std::string("color/image/compressed"),
  //                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
  //                               &img_converter, std::placeholders::_1,
  //                               std::placeholders::_2),
  //                     30);

  sys_logger_queue->addCallback([&](std::shared_ptr<dai::ADatatype> callback) {
    if (auto buf = dynamic_cast<dai::SystemInformation *>(callback.get())) {
      RCLCPP_INFO(
          node->get_logger(),
          "SysInfo\nCPU  | Leon CSS %.1f%%, Leon MSS %.1f%%\nTEMP | Chip %.1f°C\nMEM  | "
          "DDR %li/%li MB, CMX %li/%li MB\nHEAP | Leon CSS %li/%li MB, Leon MSS %li/%li MB",
          buf->leonCssCpuUsage.average * 100.0f, buf->leonMssCpuUsage.average * 100.0f,
          buf->chipTemperature.average, buf->ddrMemoryUsage.used / 1024 / 1024,
          buf->ddrMemoryUsage.total / 1024 / 1024, buf->cmxMemoryUsage.used / 1024 / 1024,
          buf->cmxMemoryUsage.total / 1024 / 1024, buf->leonCssMemoryUsage.used / 1024 / 1024,
          buf->leonCssMemoryUsage.total / 1024 / 1024, buf->leonMssMemoryUsage.used / 1024 / 1024,
          buf->leonMssMemoryUsage.total / 1024 / 1024);
    }
  });

  color_encoder_queue->addCallback([&](std::shared_ptr<dai::ADatatype> callback) {
    if (compressed_image_pub->get_subscription_count() == 0) {
      return;
    }

    if (auto buf = dynamic_cast<dai::ImgFrame *>(callback.get())) {
      sensor_msgs::msg::CompressedImage image;
      image.data = std::move(buf->getData());
      image.format = "jpeg";
      image.header.stamp = node->now();
      image.header.frame_id = "cam";

      compressed_image_pub->publish(image);
    }
  });

  preview_queue->addCallback([&](std::shared_ptr<dai::ADatatype> callback) {
    if (auto buf = std::dynamic_pointer_cast<dai::ImgFrame>(callback)) {
      auto msg = img_converter.toRosMsgPtr(buf);
      if (msg) {
        preview_pub->publish(*msg.get());
      }
    }
  });

  mono_encoder_queue->addCallback([&](std::shared_ptr<dai::ADatatype> callback) {
    if (auto buf = dynamic_cast<dai::ImgFrame *>(callback.get())) {
      sensor_msgs::msg::CompressedImage image;
      image.data = std::move(buf->getData());
      image.format = "jpeg";
      image.header.stamp = node->now();
      image.header.frame_id = "cam";

      compressed_mono_pub->publish(image);
    }
  });

  depth_queue->addCallback([&](std::shared_ptr<dai::ADatatype> callback) {
    if (auto buf = std::dynamic_pointer_cast<dai::ImgFrame>(callback)) {
      auto msg = depth_img_converter.toRosMsgPtr(buf);
      if (msg) {
        depth_pub->publish(*msg.get());
      }
    }
  });

  nnet_data_qeue->addCallback([&](std::shared_ptr<dai::ADatatype> callback) {
    if (auto buf = dynamic_cast<dai::SpatialImgDetections *>(callback.get())) {
      depthai_ros_msgs::msg::SpatialDetectionArray detections;

      for (const auto &detection : buf->detections) {
        RCLCPP_INFO(node->get_logger(), "Detection %i, pos [%.2f,%.2f,%.2f]", detection.label,
                    detection.spatialCoordinates.x, detection.spatialCoordinates.y,
                    detection.spatialCoordinates.z);

        depthai_ros_msgs::msg::SpatialDetection detection_msg;
        auto roi = detection.boundingBoxMapping.roi.denormalize(300, 300);
        detection_msg.position.x = detection.spatialCoordinates.x;
        detection_msg.position.y = detection.spatialCoordinates.y;
        detection_msg.position.z = detection.spatialCoordinates.z;
        detection_msg.bbox.center.position.x = (roi.x + roi.width) / 2.0f;
        detection_msg.bbox.center.position.y = (roi.y + roi.height) / 2.0f;
        detection_msg.bbox.size_x = roi.width;
        detection_msg.bbox.size_y = roi.height;

        detections.detections.push_back(detection_msg);

        RCLCPP_INFO(node->get_logger(), "ROI pos [%.1f %.1f %.1f %.1f]", roi.x, roi.y, roi.width,
                    roi.height);
      }

      object_tracker_pub->publish(detections);
    }
  });

  tracker_queue->addCallback([&](std::shared_ptr<dai::ADatatype> callback) {
    // if (auto buf = dynamic_cast<dai::Tracklets *>(callback.get())) {
    //   depthai_ros_msgs::msg::SpatialDetectionArray detections;
    //   detections.header.frame_id = "cam";
    //   detections.header.stamp = node->now();
    //   for (const auto &tracklet : buf->tracklets) {
    //     depthai_ros_msgs::msg::SpatialDetection detection;
    //     detection.position.x = tracklet.spatialCoordinates.x;
    //     detection.position.y = tracklet.spatialCoordinates.y;
    //     detection.position.z = tracklet.spatialCoordinates.z;
    //     detection.is_tracking = tracklet.status == dai::Tracklet::TrackingStatus::TRACKED;
    //     detection.bbox.center.position.x = (tracklet.roi.x + tracklet.roi.width) / 2.0f;
    //     detection.bbox.center.position.y = (tracklet.roi.y + tracklet.roi.height) / 2.0f;
    //     detection.bbox.size_x = tracklet.roi.width;
    //     detection.bbox.size_y = tracklet.roi.height;

    //     detections.detections.push_back(detection);

    //     RCLCPP_INFO(node->get_logger(), "ROI");
    //   }

    //   object_tracker_pub->publish(detections);
    // }
  });

  RCLCPP_INFO(node->get_logger(), "spin...");
  rclcpp::spin(node);
}