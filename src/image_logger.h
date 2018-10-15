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

class ImageLogger
{
public:
  ImageLogger()
  {
  }
};

#endif  // IAI_IMAGE_LOGGING_IMAGE_LOGGER_H
