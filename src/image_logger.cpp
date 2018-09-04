/**
 * Copyright 2018 by Tammo Wuebbena, University of Bremen
 * Using functionality of the mongodb_log and mongodb_store packages
 * Author: Tammo Wuebbena <ta_wu@tzi.de>
 */

#include "image_logger.h"

std::vector<defcon_ptr> g_cfg_multi;


int count = 0;
boost::shared_ptr<ImageLogger> imageLogger(new ImageLogger());
boost::shared_ptr<Configurator> g_cfg(new Configurator());
boost::shared_ptr<Logger> logger(new Logger());
DBClientConnection* mongodb_connection;


/**
 * Callback for the compressed images sent by one camera
 * @param msg compressed image pointer
 */
void imageCb(sensor_msgs::ImageConstPtr msg)
{
}

void printDatabaseEntries(iai_image_logging_msgs::DefaultConfig conf, int sample_size)
{
  if (count != 0 && count % sample_size == 0)
  {
    ROS_INFO_STREAM("Saved " << sample_size << " images to " << conf.collection);
    ROS_INFO_STREAM("Format = " << conf.format);
    ROS_INFO_STREAM("JPEG quality = " << conf.jpeg_quality);
    ROS_INFO_STREAM("PNG level = " << conf.png_level);
  }
}

/**
 * Configuration callback for dynamic reconfiguration
 * @param cfg
 */
void configurationCb(iai_image_logging_msgs::DefaultConfig& cfg)
{
  defcon_ptr input(new iai_image_logging_msgs::DefaultConfig(cfg));
  imageLogger->cfg_list.push_back(input);
  g_cfg->configuration(imageLogger->cfg_list);
}

void matrixFunction()
{
  defcon_ptr conf_ptr;

  conf_ptr = imageLogger->cfg_list[0];
  iai_image_logging_msgs::DefaultConfig conf;

  conf.png_level = conf_ptr->png_level;
  conf.jpeg_quality = conf_ptr->jpeg_quality;
  conf.format = conf_ptr->format;
  conf.collection = conf_ptr->collection;
  conf.topic = conf_ptr->topic;
  conf.db_host = conf_ptr->db_host;
  conf.groups = conf_ptr->groups;
  int sample_size = 100;

  if (count < sample_size)
  {
    conf.format = "jpeg";
    conf.jpeg_quality = 100;
    conf.png_level = 1;
    conf.collection = "db.im_raw_comp_jpeg100_png1";
    configurationCb(conf);
  }

  else if (count < 2 * sample_size)
  {
    conf.format = "jpeg";
    conf.jpeg_quality = 100;
    conf.png_level = 5;

    conf.collection = "db.im_raw_comp_jpeg100_png9";
    configurationCb(conf);
  }

  else if (count < 3 * sample_size)
  {
    conf.format = "png";
    conf.jpeg_quality = 100;
    conf.png_level = 1;

    conf.collection = "db.im_raw_comp_png1_jpeg100";
    configurationCb(conf);
  }

  else if (count < 4 * sample_size)
  {
    conf.format = "png";
    conf.png_level = 1;
    conf.jpeg_quality = 1;

    conf.collection = "db.im_raw_comp_png1_jpeg1";
    configurationCb(conf);
  }

  else if (count >= 4 * sample_size)
  {
    conf.format = "jpeg";
    conf.jpeg_quality = 1;
    conf.collection = "STOPPING WITH MISSING '.'";
    configurationCb(conf);
  }
  printDatabaseEntries(conf, sample_size);
  count++;
}
/**
 * Callback for the compressed images sent by one camera
 * @param msg compressed image pointer
 */
void compressedImageCb(sensor_msgs::CompressedImageConstPtr msg)
{
  // matrixFunction(); // for building test entries
  std::string collection = imageLogger->cfg_list[0]->collection;
  logger->log(msg, g_cfg_multi, mongodb_connection);
}

/**
 * Starting the main node for image logging
 * @param argc TODO
 * @param argv TODO
 * @return 0 on successful execution
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_logger");

  ros::NodeHandle n;

  // Server for dynamic_reconfigure callback
  dynamic_reconfigure::Server<iai_image_logging_msgs::DefaultConfig> server;
  dynamic_reconfigure::Server<iai_image_logging_msgs::DefaultConfig>::CallbackType f;

  f = boost::bind(&configurationCb, _1);
  server.setCallback(f);


  string topic = imageLogger->cfg_list[0]->topic;

  // image_transport sub
  image_transport::ImageTransport it(n);
  image_transport::TransportHints th("compressed");
  image_transport::Subscriber comp_img_sub = it.subscribe(topic, 1, imageCb, ros::VoidPtr(), th);
  //image_transport::Subscriber img_sub = it.subscribe(topic,1, imageCb);

  ros::Subscriber sub_kinect = n.subscribe<sensor_msgs::CompressedImage>(topic + "/compressed", 100, compressedImageCb);
    ros::Subscriber sub_cfg;
  for (defcon_ptr cfg : g_cfg_multi)
  {
      ros::Subscriber sub_cfg = n.subscribe<sensor_msgs::CompressedImage>(cfg->topic + "/compressed", 100, compressedImageCb);


  }
  ros::Rate r(1.0);

  while (n.ok())
  {

          ros::spinOnce();
    r.sleep();
  }

  return 0;
}
