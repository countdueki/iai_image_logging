/**
 * Copyright 2018 by Tammo Wuebbena, University of Bremen
 * Using functionality of the mongodb_log and mongodb_store packages
 * Author: Tammo Wuebbena <ta_wu@tzi.de>
 */

#include "../include/header/iai_updater.h"

/**
 * Set parameters for compression
 * @param cfg
 * @param req
 * @param res
 */
void IAIUpdater::setCompressedParameters(MainConfig& cfg, ReconfigureRequest req, ReconfigureResponse res)
{
  StrParam format;
  IntParam jpeg, png;

  format.name = "format";
  format.value = cfg.format;

  jpeg.name = "jpeg_quality";
  jpeg.value = cfg.jpeg_quality;

  png.name = "png_level";
  png.value = cfg.png_level;

  // Set parameters for compressed images
  req.config.strs.push_back(format);
  req.config.ints.push_back(jpeg);
  req.config.ints.push_back(png);
  ROS_DEBUG_STREAM("Setting parameters for compressed topic_ based on: " << cfg.topic);
  ros::service::call(cfg.topic + "/compressed/set_parameters", req, res);
}

/**
 * Set parameters for theora video
 * @param cfg
 * @param req
 * @param res
 */
void IAIUpdater::setTheoraParameters(MainConfig& cfg, ReconfigureRequest req, ReconfigureResponse res)
{
  IntParam optimize_for, keyframe_frequency, quality, target_bitrate;

  optimize_for.name = "optimize_for";
  optimize_for.value = cfg.optimize_for;

  target_bitrate.name = "target_bitrate";
  target_bitrate.value = cfg.target_bitrate;

  keyframe_frequency.name = "keyframe_frequency";
  keyframe_frequency.value = cfg.keyframe_frequency;

  quality.name = "quality";
  quality.value = cfg.quality;

  // Set parameters for theora video
  req.config.ints.push_back(optimize_for);
  req.config.ints.push_back(keyframe_frequency);
  req.config.ints.push_back(quality);
  req.config.ints.push_back(target_bitrate);

  ROS_DEBUG_STREAM("Setting parameters for theora topic_ based on: " << cfg.topic);
  ros::service::call(cfg.topic + "/theora/set_parameters", req, res);
}

/**
 * set Parameters for depth compression
 * @param cfg
 * @param req
 * @param res
 */
void IAIUpdater::setDepthCompressedParameters(MainConfig& cfg, ReconfigureRequest req, ReconfigureResponse res)
{
  IntParam png;
  DoubleParam depth_max, depth_quantization;

  png.name = "png_level";
  png.value = cfg.png_level;

  depth_max.name = "depth_max";
  depth_max.value = cfg.depth_max;

  depth_quantization.name = "depth_quantization";
  depth_quantization.value = cfg.depth_quantization;

  req.config.ints.push_back(png);
  req.config.doubles.push_back(depth_max);
  req.config.doubles.push_back(depth_quantization);

  ROS_DEBUG_STREAM("Setting parameters for compresseDepth topic_ based on: " << cfg.topic);
  ros::service::call(cfg.topic + "/compressedDepth/set_parameters", req, res);
}

/**
 * Callback for dynamic reconfiguration
 * @param cfg Configuration to update parameters of topics to be logged
 */
void IAIUpdater::mainConfigurationCb(MainConfig& cfg)
{
  ReconfigureRequest req;
  ReconfigureResponse res;
  if (cfg.mode == COMPRESSED)
  {
    setCompressedParameters(cfg, req, res);
  }
  else if (cfg.mode == THEORA)
  {
    setTheoraParameters(cfg, req, res);
  }
  else if (cfg.mode == COMPRESSED_DEPTH)
  {
    setDepthCompressedParameters(cfg, req, res);
  }
  else if (cfg.mode == RAW)
  {
    ROS_DEBUG_STREAM("Setting parameters for raw topic_: " << cfg.topic);
  }
/*
    setCompressedParameters(cfg, req, res);
    setTheoraParameters(cfg, req, res);
    setDepthCompressedParameters(cfg, req, res);

    */

}

int main(int argc, char** argv)
{
  string node_name = "iai_updater";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;

  dynamic_reconfigure::Server<MainConfig> server_;
  dynamic_reconfigure::Server<MainConfig>::CallbackType cb_type_;

  cb_type_ = boost::bind(&IAIUpdater::mainConfigurationCb, _1);
  server_.setCallback(cb_type_);

  ros::spin();

  return 0;
}
