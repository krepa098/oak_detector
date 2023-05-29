#include "oak_detector/camera_node.h"
#include "oak_detector/pipeline.h"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CameraNode>("oak_detector_node");

  RCLCPP_INFO(node->get_logger(), "spin...");
  rclcpp::spin(node);

  return 0;
}
