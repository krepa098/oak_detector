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

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("oak_detector_node");

  RCLCPP_INFO(node->get_logger(), "creating pipeline");
  dai::Pipeline pipeline = createPipeline(BLOB_PATH);

  RCLCPP_INFO(node->get_logger(), "opening device");
  dai::Device device(pipeline);
  auto cal = device.getDeviceInfo();

  // create queues
  // each queue has its own thread
  RCLCPP_INFO(node->get_logger(), "creating output queues");
  std::shared_ptr<dai::DataOutputQueue> preview_queue =
      device.getOutputQueue("preview", 30, false);
  std::shared_ptr<dai::DataOutputQueue> nnet_data_qeue =
      device.getOutputQueue("detections", 30, false);

  // ros bridge
  dai::rosBridge::ImageConverter img_converter("rgb_frame");
  auto rgb_cam_info = img_converter.calibrationToCameraInfo(
      device.readCalibration(), dai::CameraBoardSocket::RGB);

  //   nnet_data_qeue->addCallback([&](std::shared_ptr<dai::ADatatype> callback)
  //   {
  //     if (auto nn_data = dynamic_cast<dai::NNData *>(callback.get())) {
  //     }
  //   });

  dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>
      rgb_publish(preview_queue, node, std::string("color/image"),
                  std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                            &img_converter, std::placeholders::_1,
                            std::placeholders::_2),
                  30, rgb_cam_info, "color");

  rgb_publish.addPublisherCallback();

  RCLCPP_INFO(node->get_logger(), "spin...");
  rclcpp::spin(node);
}