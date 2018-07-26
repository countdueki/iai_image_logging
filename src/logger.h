//
// Created by tammo on 26.07.18.
//

#ifndef IAI_IMAGE_LOGGING_LOGGER_H
#define IAI_IMAGE_LOGGING_LOGGER_H

#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <iai_image_logging_msgs/DefaultConfig.h>
#include <iai_image_logging_msgs/Log.h>
using std::vector;

extern vector<iai_image_logging_msgs::DefaultConfig> g_cfg_multi;

class Logger
{
public:
};

#endif  // IAI_IMAGE_LOGGING_LOGGER_H
