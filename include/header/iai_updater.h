/**
 * Copyright 2018 by Tammo Wuebbena, University of Bremen
 * Author: Tammo Wuebbena <ta_wu@tzi.de>
 */

#ifndef IAI_IMAGE_LOGGING_IMAGE_LOGGER_H
#define IAI_IMAGE_LOGGING_IMAGE_LOGGER_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <iai_image_logging_msgs/MainConfig.h>
#include <iai_image_logging_msgs/Update.h>
#include <iai_image_logging_msgs/DeleteRequest.h>
#include <iai_image_logging_msgs/DeleteResponse.h>
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
class IAIUpdater
{
public:
  static void setCompressedParameters(MainConfig& cfg, ReconfigureRequest req, ReconfigureResponse res);
  static void setTheoraParameters(MainConfig& cfg, ReconfigureRequest req, ReconfigureResponse res);
  static void setDepthCompressedParameters(MainConfig& cfg, ReconfigureRequest req, ReconfigureResponse res);
  static void mainConfigurationCb(MainConfig& cfg);
};

#endif  // IAI_IMAGE_LOGGING_IMAGE_LOGGER_H
