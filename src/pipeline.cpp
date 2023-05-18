#include "oak_detector/pipeline.h"

#include "depthai/depthai.hpp"

dai::Pipeline createPipeline(const std::string &nn_path) {
  dai::Pipeline pipeline;

  // color
  auto color = pipeline.create<dai::node::ColorCamera>();
  color->setPreviewSize(300, 300);
  color->setResolution(
      dai::ColorCameraProperties::SensorResolution::THE_1080_P);
  color->setInterleaved(false);
  color->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
  color->setFps(30);

  // depth
  //   auto depth = pipeline.create<dai::node::StereoDepth>();
  //   depth->setDefaultProfilePreset(
  //       dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
  //   depth->initialConfig.setMedianFilter(
  //       dai::MedianFilter::KERNEL_5x5); // in hardware -> fast

  // detection network
  auto dn = pipeline.create<dai::node::MobileNetDetectionNetwork>();
  dn->setConfidenceThreshold(0.5f);
  dn->setBlobPath(nn_path);

  // outputs
  auto color_out = pipeline.create<dai::node::XLinkOut>();
  auto nn_out = pipeline.create<dai::node::XLinkOut>();

  color_out->setStreamName("preview");
  nn_out->setStreamName("detections");

  // Link plugins CAM -> NN -> XLINK
  color->preview.link(dn->input);
  dn->passthrough.link(color_out->input);
  dn->out.link(nn_out->input);

  return pipeline;
}