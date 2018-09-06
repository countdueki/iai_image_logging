/**
 * Copyright 2018 by Tammo Wuebbena, University of Bremen
 * Using functionality of the mongodb_log and mongodb_store packages
 * Author: Tammo Wuebbena <ta_wu@tzi.de>
 */

#include "image_logger.h"
#include <pluginlib/class_list_macros.h>

namespace image_logger
{
DBClientConnection* mongodb_conn;

int count = 0;

boost::shared_ptr<CompConf> g_cfg;
boost::shared_ptr<ImageLogger> g_imageLogger;

/**
 * Configuration callback for dynamic reconfiguration
 * @param cfg
 */
void configurationCb(iai_image_logging_msgs::CompressedConfig& cfg)
{
  boost::shared_ptr<CompConf> cfg_ptr(new CompConf(cfg));
  g_cfg = cfg_ptr;
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
  ROS_DEBUG_STREAM("Set parameters on topic " << cfg.topic + "/compressed");
  ROS_DEBUG_STREAM("Request " << req.config.ints[0].name << ": " << req.config.ints[0].value);
  ROS_DEBUG_STREAM("Respone " << res.config.ints[0].name << ": " << res.config.ints[0].value);
}

/**
 *
 */
void ImageLogger::onInit()
{
}

/**
 * Starting the main node for image logging
 * @param argc TODO
 * @param argv TODO
 * @return 0 on successful execution
 */
int imageLoggerSetup(int argc, char** argv)
{
  ros::init(argc, argv, "image_logger");

  ros::NodeHandle n;

  // Server for dynamic_reconfigure callback
  dynamic_reconfigure::Server<iai_image_logging_msgs::CompressedConfig> server;
  dynamic_reconfigure::Server<iai_image_logging_msgs::CompressedConfig>::CallbackType f;

  f = boost::bind(&configurationCb, _1);
  server.setCallback(f);

  // Testing client

  //  iai_image_logging_msgs::Setup setup;
  //  ros::M_string header;
  //  header[0] = "header";
  //  ros::ServiceClient setup_client = n.serviceClient<iai_image_logging_msgs::Process>("setup_client",false,header);

  iai_image_logging_msgs::ProcessRequest proc_req;
  iai_image_logging_msgs::ProcessResponse proc_res;

  proc_req.set.db_host = "localhost";
  proc_req.set.topic = "/camera/rgb/image_raw/compressed";
  proc_req.set.format = "png";
  proc_req.set.jpeg_quality = 1;
  proc_req.set.png_level = 9;
  proc_req.set.collection = "db.process_requested_png";

  iai_image_logging_msgs::CompressedConfig proc_cfg;
  proc_cfg.db_host = "localhost";
  proc_cfg.topic = "/camera/rgb/image_raw/compressed";
  proc_cfg.format = "png";
  proc_cfg.jpeg_quality = 1;
  proc_cfg.png_level = 9;
  proc_cfg.collection = "db.process_requested_png";
  configurationCb(proc_cfg);
  ros::Rate r(1.0);

  while (n.ok())
  {
    // setup_client.call(proc_req,proc_res);
    ros::service::call("/preprocessor/process", proc_req, proc_res);
    if (proc_res.success == true)
      ROS_INFO_STREAM("Successfully called service");
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
}
PLUGINLIB_EXPORT_CLASS(image_logger::ImageLogger, nodelet::Nodelet)