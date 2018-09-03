//
// Created by tammo on 26.07.18.
//

#include "configurator.h"

void Configurator::configuration(vector<iai_image_logging_msgs::DefaultConfig> cfg_multi)
{
/*  g_cfg_multi = cfg_multi;
  dynamic_reconfigure::ReconfigureRequest req;
  dynamic_reconfigure::ReconfigureResponse res;
  dynamic_reconfigure::StrParameter format;
  dynamic_reconfigure::IntParameter jpeg;
  dynamic_reconfigure::IntParameter png;
  dynamic_reconfigure::Config conf_req;

  format.name = "format";
  format.value = "";

  jpeg.name = "jpeg_quality";
  jpeg.value = 0;

  png.name = "png_level";
  png.value = 0;

  req.config.strs.push_back(format);
  req.config.ints.push_back(jpeg);
  req.config.ints.push_back(png);

  for (iai_image_logging_msgs::DefaultConfig cfg : cfg_multi)
  {
    req.config.strs.at(0).value = cfg.format;
    req.config.ints.at(0).value = cfg.jpeg_quality;
    req.config.ints.at(1).value = cfg.png_level;

    ros::service::call(cfg.topic + "/compressed/set_parameters", req, res);
    ROS_DEBUG_STREAM("Set parameters on topic " << cfg.topic + "/compressed");
    ROS_DEBUG_STREAM("Request " << req.config.ints[0].name << ": " << req.config.ints[0].value);
    ROS_DEBUG_STREAM("Respone " << res.config.ints[0].name << ": " << res.config.ints[0].value);
  }

  req.config.strs.clear();
  req.config.ints.clear();*/
}
