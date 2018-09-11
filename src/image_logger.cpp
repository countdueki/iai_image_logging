/**
 * Copyright 2018 by Tammo Wuebbena, University of Bremen
 * Using functionality of the mongodb_log and mongodb_store packages
 * Author: Tammo Wuebbena <ta_wu@tzi.de>
 */

#include "image_logger.h"

/**
 * Callback for dynamic reconfiguration
 * @param cfg Configuration to update parameters of topics to be logged
 */
void mainConfigurationCb(MainConf& cfg)
{
  dynamic_reconfigure::ReconfigureRequest req;
  dynamic_reconfigure::ReconfigureResponse res;

  ProcReq proc_req;
  ProcRes proc_res;

  StrParam db_host, collection, topic, format;
  IntParam jpeg, png, optimize_for, keyframe_frequency, quality;
  DoubleParam target_bitrate;

  db_host.name = "db_host";
  db_host.value = cfg.db_host;

  collection.name = "collection";
  collection.value = cfg.collection;

  topic.name = "topic";
  topic.value = cfg.topic;

  format.name = "format";
  format.value = cfg.format;

  jpeg.name = "jpeg_quality";
  jpeg.value = cfg.jpeg_quality;

  png.name = "png_level";
  png.value = cfg.png_level;

  optimize_for.name = "optimize_for";
  optimize_for.value = cfg.optimize_for;

  target_bitrate.name = "target_bitrate";
  target_bitrate.value = cfg.target_bitrate;

  keyframe_frequency.name = "keyframe_frequency";
  keyframe_frequency.value = cfg.keyframe_frequency;

  quality.name = "quality";
  quality.value = cfg.quality;

  // Set general parameters
  req.config.strs.push_back(db_host);
  req.config.strs.push_back(collection);
  req.config.strs.push_back(topic);

  // Set parameters for compressed images
  req.config.strs.push_back(format);
  req.config.ints.push_back(jpeg);
  req.config.ints.push_back(png);

  // Set parameters for theora video
  req.config.ints.push_back(optimize_for);
  req.config.ints.push_back(keyframe_frequency);
  req.config.ints.push_back(quality);
  req.config.doubles.push_back(target_bitrate);
  ros::service::call(cfg.topic + "/set_parameters", req, res);

  // Call specific preprocessing for different compressions
  switch (cfg.mode)
  {
    case (RAW):
      break;
    case (COMPRESSED):

      proc_req.set.topic = cfg.topic;
      proc_req.set.db_host = cfg.db_host;
      proc_req.set.collection = cfg.collection;
      proc_req.set.format = cfg.format;
      proc_req.set.png_level = cfg.png_level;
      proc_req.set.jpeg_quality = cfg.jpeg_quality;

      ros::service::call("preprocessor/process_compressed", proc_req, proc_res);

      break;
    case (THEORA):

      proc_req.set.topic = cfg.topic;
      proc_req.set.db_host = cfg.db_host;
      proc_req.set.collection = cfg.collection;

      ros::service::call("preprocessor/process_theora", proc_req, proc_res);

      break;
    default:
      break;
  }
}

/**
 * Main Image Logging node responsible for handling configuration
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_logger");
  ros::NodeHandle nh;

  // Server for dynamic_reconfigure callback
  dynamic_reconfigure::Server<MainConf> server;
  dynamic_reconfigure::Server<MainConf>::CallbackType cb_type;

  cb_type = boost::bind(&mainConfigurationCb, _1);
  server.setCallback(cb_type);

  ros::Rate sleep_rate(1.0);

  while (nh.ok())
  {
    ros::spinOnce();
    sleep_rate.sleep();
  }
  return 0;
}
