/**
 * Copyright 2018 by Tammo Wuebbena, University of Bremen
 * Using functionality of the mongodb_log and mongodb_store packages
 * Author: Tammo Wuebbena <ta_wu@tzi.de>
 */

#include "image_logger.h"

boost::shared_ptr<ImageLogger> g_imageLogger;

/**
 * Configuration callback for dynamic reconfiguration
 * @param cfg
 */
void configurationCb(iai_image_logging_msgs::CompressedConfig& cfg)
{
  dynamic_reconfigure::ReconfigureRequest req;
  dynamic_reconfigure::ReconfigureResponse res;
  dynamic_reconfigure::StrParameter format;
  dynamic_reconfigure::IntParameter jpeg;
  dynamic_reconfigure::IntParameter png;
  dynamic_reconfigure::Config conf_req;

  format.name = "format";
  format.value = cfg.format;

  jpeg.name = "jpeg_quality";
  jpeg.value = cfg.jpeg_quality;

  png.name = "png_level";
  png.value = cfg.png_level;

  req.config.strs.push_back(format);
  req.config.ints.push_back(jpeg);
  req.config.ints.push_back(png);

  ros::service::call(cfg.topic + "/compressed/set_parameters", req, res);

  iai_image_logging_msgs::ProcessRequest proc_req;
  iai_image_logging_msgs::ProcessResponse proc_res;

  proc_req.set.topic = cfg.topic;
  ros::service::call("preprocessor/process", proc_req, proc_res);
  ROS_INFO_STREAM("called preprocessor service");
  ROS_DEBUG_STREAM("Set parameters on topic " << cfg.topic + "/compressed");
  ROS_DEBUG_STREAM("Request " << req.config.ints[0].name << ": " << req.config.ints[0].value);
  ROS_DEBUG_STREAM("Respone " << res.config.ints[0].name << ": " << res.config.ints[0].value);
}

/**
 *
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_logger");
  ros::NodeHandle nh;

  // Server for dynamic_reconfigure callback
  dynamic_reconfigure::Server<iai_image_logging_msgs::CompressedConfig> server;
  dynamic_reconfigure::Server<iai_image_logging_msgs::CompressedConfig>::CallbackType f;

  f = boost::bind(&configurationCb, _1);
  server.setCallback(f);

  // ros::Publisher cfg_pub = nh.advertise<CompConf>("image_logger/config",1);
  ros::Rate r(1.0);

  while (nh.ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
