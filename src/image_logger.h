/**
 * Copyright 2018 by Tammo Wuebbena, University of Bremen
 * Author: Tammo Wuebbena <ta_wu@tzi.de>
 */

#ifndef IAI_IMAGE_LOGGING_IMAGE_LOGGER_H
#define IAI_IMAGE_LOGGING_IMAGE_LOGGER_H

#include <ros/ros.h>

#include <boost/smart_ptr/shared_ptr.hpp>
#include <dynamic_reconfigure/server.h>

#include <iai_image_logging_msgs/MainConfig.h>
#include <iai_image_logging_msgs/Update.h>

#include <iostream>

typedef iai_image_logging_msgs::MainConfig MainConfig;

typedef dynamic_reconfigure::StrParameter StrParam;
typedef dynamic_reconfigure::IntParameter IntParam;
typedef dynamic_reconfigure::DoubleParameter DoubleParam;

using std::string;
enum
{
  RAW,
  COMPRESSED,
  THEORA
};

class ImageLogger
{
public:
  ImageLogger()
  {
  }
};

#endif  // IAI_IMAGE_LOGGING_IMAGE_LOGGER_H
