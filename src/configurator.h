//
// Created by tammo on 26.07.18.
//

#ifndef IAI_IMAGE_LOGGING_CONFIGURATOR_H
#define IAI_IMAGE_LOGGING_CONFIGURATOR_H
#include <iai_image_logging_msgs/DefaultConfig.h>
#include <dynamic_reconfigure/server.h>

#include <ros/ros.h>
using std::vector;

using std::vector;
extern vector<iai_image_logging_msgs::DefaultConfig> g_cfg_multi;

class Configurator
{
public:
  Configurator()
  {
  }

public:
  void configuration(vector<iai_image_logging_msgs::DefaultConfig> cfg_multi);
};

#endif  // IAI_IMAGE_LOGGING_CONFIGURATOR_H
