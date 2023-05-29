#include "oak_detector/pipeline.h"

#include "depthai/depthai.hpp"

static const std::vector<std::string> label_map = {
    "background", "aeroplane", "bicycle",     "bird",  "boat",        "bottle", "bus",
    "car",        "cat",       "chair",       "cow",   "diningtable", "dog",    "horse",
    "motorbike",  "person",    "pottedplant", "sheep", "sofa",        "train",  "tvmonitor"};

dai::Pipeline createPipeline(const std::string &nn_path) {
  dai::Pipeline pipeline;

  const float fps = 30.0f;

  // sys logger (cpu usage, temperature etc.)
  auto sys_logger = pipeline.create<dai::node::SystemLogger>();

  // color
  auto color = pipeline.create<dai::node::ColorCamera>();
  color->setPreviewSize(300, 300);
  color->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
  color->setInterleaved(false);
  color->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
  color->setBoardSocket(dai::CameraBoardSocket::RGB);
  color->setFps(fps);
  color->setPreviewKeepAspectRatio(false);

  // video encoder
  auto color_encoder = pipeline.create<dai::node::VideoEncoder>();
  color_encoder->setDefaultProfilePreset(fps, dai::VideoEncoderProperties::Profile::MJPEG);

  auto mono_encoder = pipeline.create<dai::node::VideoEncoder>();
  mono_encoder->setDefaultProfilePreset(fps, dai::VideoEncoderProperties::Profile::MJPEG);

  // depth / stereo
  auto mono_res = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
  auto mono_left = pipeline.create<dai::node::MonoCamera>();
  mono_left->setResolution(mono_res);
  mono_left->setBoardSocket(dai::CameraBoardSocket::LEFT);
  mono_left->setFps(fps);
  auto mono_right = pipeline.create<dai::node::MonoCamera>();
  mono_right->setResolution(mono_res);
  mono_right->setBoardSocket(dai::CameraBoardSocket::RIGHT);
  mono_right->setFps(fps);

  auto stereo = pipeline.create<dai::node::StereoDepth>();
  stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
  stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7); // in hardware -> fast
  //   auto stereo_config = stereo->initialConfig.get();
  //   stereo_config.postProcessing.spatialFilter.alpha = 0.5;
  //   stereo_config.postProcessing.spatialFilter.delta = 0;
  //   stereo_config.postProcessing.spatialFilter.numIterations = 1;
  //   stereo_config.postProcessing.spatialFilter.enable = true;
  //   stereo_config.postProcessing.spatialFilter.holeFillingRadius = 3;
  //   stereo->initialConfig.set(stereo_config);
  stereo->setExtendedDisparity(true);
  stereo->setLeftRightCheck(true);
  stereo->setDepthAlign(dai::CameraBoardSocket::RGB);
  stereo->setOutputSize(mono_left->getResolutionWidth(), mono_left->getResolutionHeight());

  mono_left->out.link(stereo->left);
  mono_right->out.link(stereo->right);

  // detection network
  auto dn = pipeline.create<dai::node::MobileNetSpatialDetectionNetwork>();
  dn->setConfidenceThreshold(0.5f);
  dn->setBlobPath(nn_path);
  dn->setNumInferenceThreads(3);
  dn->input.setBlocking(false);
  dn->inputDepth.setBlocking(false);

  color->preview.link(dn->input);
  stereo->depth.link(dn->inputDepth);

  // object tracker
  auto object_tracker = pipeline.create<dai::node::ObjectTracker>();
  object_tracker->setDetectionLabelsToTrack({15}); // people
  object_tracker->setTrackerType(dai::TrackerType::ZERO_TERM_COLOR_HISTOGRAM);
  object_tracker->setTrackerIdAssignmentPolicy(dai::TrackerIdAssignmentPolicy::SMALLEST_ID);
  object_tracker->inputDetectionFrame.setBlocking(false);
  object_tracker->inputDetections.setBlocking(false);

  // outputs
  auto sys_logger_out = pipeline.create<dai::node::XLinkOut>();
  sys_logger_out->setStreamName("sys_logger");
  auto preview_out = pipeline.create<dai::node::XLinkOut>();
  preview_out->setStreamName("preview");
  auto nn_out = pipeline.create<dai::node::XLinkOut>();
  nn_out->setStreamName("detections");
  auto color_mjpeg_out = pipeline.create<dai::node::XLinkOut>();
  color_mjpeg_out->setStreamName("color_mjpeg");
  auto mono_mjpeg_out = pipeline.create<dai::node::XLinkOut>();
  mono_mjpeg_out->setStreamName("mono_mjpeg");
  auto depth_out = pipeline.create<dai::node::XLinkOut>();
  depth_out->setStreamName("depth");
  auto tracker_out = pipeline.create<dai::node::XLinkOut>();
  tracker_out->setStreamName("object_tracker");

  // create links
  //
  // sys logger
  sys_logger->out.link(sys_logger_out->input);

  // neural network
  dn->out.link(nn_out->input);
  dn->passthrough.link(preview_out->input);

  // link object tracker
  dn->passthrough.link(object_tracker->inputDetectionFrame);
  dn->passthrough.link(object_tracker->inputTrackerFrame); // this can be the full frame
  dn->out.link(object_tracker->inputDetections);
  object_tracker->out.link(tracker_out->input);

  // link color video encoder
  color->video.link(color_encoder->input);
  color_encoder->bitstream.link(color_mjpeg_out->input);

  // link mono video encoder
  mono_left->out.link(mono_encoder->input);
  mono_encoder->bitstream.link(mono_mjpeg_out->input);

  // link stereo out
  stereo->depth.link(depth_out->input);

  return pipeline;
}