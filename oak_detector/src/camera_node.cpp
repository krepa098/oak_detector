#include "oak_detector/camera_node.h"

CameraNode::CameraNode(const std::string &name) : rclcpp::Node(name) {
  // params
  declare_parameter("nn_blob_path", "");
  declare_parameter("ir_projector_brightness", 0.0f); // mA

  // publishers
  compressed_image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "color/compressed", rclcpp::SensorDataQoS());

  preview_pub_ =
      create_publisher<sensor_msgs::msg::Image>("preview/image", rclcpp::SensorDataQoS());

  compressed_mono_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "mono/compressed", rclcpp::SensorDataQoS());

  depth_pub_ = create_publisher<sensor_msgs::msg::Image>("depth/image", rclcpp::SensorDataQoS());

  object_tracker_pub_ = create_publisher<depthai_ros_msgs::msg::SpatialDetectionArray>(
      "object_tracker/detections", rclcpp::SensorDataQoS());

  // pipeline
  RCLCPP_INFO(get_logger(), "creating pipeline");
  std::string blob_path = get_parameter("nn_blob_path").as_string().empty()
                              ? BLOB_PATH
                              : get_parameter("nn_blob_path").as_string();
  RCLCPP_INFO(get_logger(), "loading NN blob from path '%s'", blob_path.c_str());
  pipeline_ = createPipeline(blob_path);

  // device
  RCLCPP_INFO(get_logger(), "opening device");
  device_ = std::make_shared<dai::Device>(pipeline_);
  auto cal = device_->getDeviceInfo();

  // enable laser projector
  device_->setIrLaserDotProjectorBrightness(get_parameter("ir_projector_brightness").as_double());

  // create queues
  // each queue has its own thread
  RCLCPP_INFO(get_logger(), "creating output queues");
  sys_logger_queue_ = device_->getOutputQueue("sys_logger", 4, false);
  preview_queue_ = device_->getOutputQueue("preview", 8, false);
  nnet_data_qeue_ = device_->getOutputQueue("detections", 4, false);
  color_encoder_queue_ = device_->getOutputQueue("color_mjpeg", 8, false);
  mono_encoder_queue_ = device_->getOutputQueue("mono_mjpeg", 8, false);
  depth_queue_ = device_->getOutputQueue("depth", 8, false);
  tracker_queue_ = device_->getOutputQueue("object_tracker", 4, false);

  // ros bridge
  auto rgb_cam_info = img_converter_.calibrationToCameraInfo(device_->readCalibration(),
                                                             dai::CameraBoardSocket::RGB);

  sys_logger_queue_->addCallback([&](std::shared_ptr<dai::ADatatype> callback) {
    if (auto buf = dynamic_cast<dai::SystemInformation *>(callback.get())) {
      RCLCPP_INFO(
          get_logger(),
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

  color_encoder_queue_->addCallback([&](std::shared_ptr<dai::ADatatype> callback) {
    if (compressed_image_pub_->get_subscription_count() == 0) {
      return;
    }

    if (auto buf = dynamic_cast<dai::ImgFrame *>(callback.get())) {
      sensor_msgs::msg::CompressedImage image;
      image.data = std::move(buf->getData());
      image.format = "jpeg";
      image.header.stamp = now();
      image.header.frame_id = "cam";

      compressed_image_pub_->publish(image);
    }
  });

  preview_queue_->addCallback([&](std::shared_ptr<dai::ADatatype> callback) {
    if (const auto buf = std::dynamic_pointer_cast<dai::ImgFrame>(callback)) {
      auto msg = img_converter_.toRosMsgPtr(buf);
      if (msg) {
        preview_pub_->publish(*msg.get());
      }
    }
  });

  mono_encoder_queue_->addCallback([&](std::shared_ptr<dai::ADatatype> callback) {
    if (auto buf = dynamic_cast<dai::ImgFrame *>(callback.get())) {
      sensor_msgs::msg::CompressedImage image;
      image.data = std::move(buf->getData());
      image.format = "jpeg";
      image.header.stamp = now();
      image.header.frame_id = "cam";

      compressed_mono_pub_->publish(image);
    }
  });

  depth_queue_->addCallback([&](std::shared_ptr<dai::ADatatype> callback) {
    if (const auto buf = std::dynamic_pointer_cast<dai::ImgFrame>(callback)) {
      auto msg = img_converter_.toRosMsgPtr(buf);
      if (msg) {
        depth_pub_->publish(*msg.get());
      }
    }
  });

  nnet_data_qeue_->addCallback([&](std::shared_ptr<dai::ADatatype> callback) {
    if (auto buf = dynamic_cast<dai::SpatialImgDetections *>(callback.get())) {
      depthai_ros_msgs::msg::SpatialDetectionArray detections;

      for (const auto &detection : buf->detections) {
        RCLCPP_INFO(get_logger(), "Detection %i, pos [%.2f,%.2f,%.2f]", detection.label,
                    detection.spatialCoordinates.x, detection.spatialCoordinates.y,
                    detection.spatialCoordinates.z);

        dai::Rect roi;
        roi.x = detection.xmin;
        roi.y = detection.ymin;
        roi.width = detection.xmax - detection.xmin;
        roi.height = detection.ymax - detection.ymin;
        roi = roi.denormalize(300, 300);

        depthai_ros_msgs::msg::SpatialDetection detection_msg;
        detection_msg.position.x = detection.spatialCoordinates.x;
        detection_msg.position.y = detection.spatialCoordinates.y;
        detection_msg.position.z = detection.spatialCoordinates.z;
        detection_msg.bbox.center.position.x = roi.x + roi.width / 2.0f;
        detection_msg.bbox.center.position.y = roi.y + roi.height / 2.0f;
        detection_msg.bbox.size_x = roi.width;
        detection_msg.bbox.size_y = roi.height;

        detections.detections.push_back(detection_msg);

        RCLCPP_INFO(get_logger(), "ROI pos [%.1f %.1f %.1f %.1f]", roi.x, roi.y, roi.width,
                    roi.height);
      }

      object_tracker_pub_->publish(detections);
    }
  });
}