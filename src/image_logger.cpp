/**
 * Copyright 2018 by Tammo Wuebbena, University of Bremen
 * Using functionality of the mongodb_log and mongodb_store packages
 * Author: Tammo Wuebbena <ta_wu@tzi.de>
 */

#include "image_logger.h"

using std::string;

using dynamic_reconfigure::ReconfigureRequest;
using dynamic_reconfigure::ReconfigureResponse;

typedef iai_image_logging_msgs::MainConfig MainConfig;

typedef dynamic_reconfigure::StrParameter StrParam;
typedef dynamic_reconfigure::IntParameter IntParam;
typedef dynamic_reconfigure::DoubleParameter DoubleParam;

enum
{
  RAW,
  COMPRESSED,
  THEORA,
  DEPTH,
  COMPRESSED_DEPTH
};

/**
 * Set parameters for compression
 * @param cfg
 * @param req
 * @param res
 */
void setCompressedParameters(MainConfig cfg, ReconfigureRequest req, ReconfigureResponse res)
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
void setTheoraParameters(MainConfig& cfg, ReconfigureRequest req, ReconfigureResponse res)
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
void setDepthCompressedParameters(MainConfig& cfg, ReconfigureRequest req, ReconfigureResponse res)
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
 * adds cam and compression suffixes to topics and calls storage::update function to add a new Subscriber
 * @param cfg information for new subscriber and mongoDB info
 * @return 0 if success, false otherwise
 */
int updateStorage(MainConfig& cfg)
{
  try
  {
    iai_image_logging_msgs::UpdateRequest req;
    iai_image_logging_msgs::UpdateResponse res;

    req.db_host = cfg.db_host;
    req.collection = cfg.collection;
    req.topic = cfg.topic;
    req.mode = cfg.mode;
    req.cam_no = cfg.cam_no;

    ROS_WARN_STREAM("Calling update storage");
    ros::service::call("storage/update", req, res);
    return 0;
  }
  catch (ros::Exception e)
  {
    ROS_ERROR_STREAM(e.what());
    return -1;
  }
}
/**
 * Callback for dynamic reconfiguration
 * @param cfg Configuration to update parameters of topics to be logged
 */
void mainConfigurationCb(MainConfig& cfg)
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
  if (updateStorage(cfg) != 0)
  {
    ROS_ERROR_STREAM("Could not update Storage node");
  }
}

/**
 * initialize storage node
 */
void initStorage()
{
  try
  {
    iai_image_logging_msgs::UpdateRequest req;
    iai_image_logging_msgs::UpdateResponse res;
    ros::service::call("storage/update", req, res);
  }
  catch (ros::Exception e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

int main(int argc, char** argv)
{
  string node_name = "image_logger";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;

  // Server for dynamic_reconfigure callback
  dynamic_reconfigure::Server<MainConfig> server;
  dynamic_reconfigure::Server<MainConfig>::CallbackType cb_type;

  cb_type = boost::bind(&mainConfigurationCb, _1);
  server.setCallback(cb_type);

  // initialize storage node with subscribers // TODO: Fix fail on empty storage node (i.e. when initStorage not used)
  initStorage();

  ros::spin();

  return 0;
}
