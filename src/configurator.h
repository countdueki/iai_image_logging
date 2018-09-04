//
// Created by tammo on 26.07.18.
//

#ifndef IAI_IMAGE_LOGGING_CONFIGURATOR_H
#define IAI_IMAGE_LOGGING_CONFIGURATOR_H
#include <iai_image_logging_msgs/DefaultConfig.h>
#include <dynamic_reconfigure/server.h>
#include "image_logger_types.h"

#include <ros/ros.h>
using std::vector;

using std::vector;

class Configurator
{
public:
  Configurator()
  {
  }

public:
  void configuration(vector<defcon_ptr> cfg_multi);
};

#endif  // IAI_IMAGE_LOGGING_CONFIGURATOR_H
