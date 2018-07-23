/**
 * Copyright 2018 by Tammo Wuebbena, University of Bremen
 * Using functionality of the mongodb_log and mongodb_store packages
 * Author: Tammo Wuebbena <ta_wu@tzi.de>
 */

#include "image_logger.h"

DBClientConnection* mongodb_conn;

int count = 0;

iai_image_logging_msgs::DefaultConfig g_cfg;

/**
 * Callback for the compressed images sent by one camera
 * @param msg compressed image pointer
 */
void imageCb(sensor_msgs::ImageConstPtr msg)
{
}

void printDatabaseEntries(iai_image_logging_msgs::DefaultConfig conf, int sample_size)
{
  if ( count != 0 && count % sample_size == 0)
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
  g_cfg = cfg;
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

void matrixFunction()
{
  iai_image_logging_msgs::DefaultConfig conf;
  conf = g_cfg;
  int sample_size = 100;

  if (count < sample_size)
  {
    conf.format = "jpeg";
    conf.jpeg_quality = 100;
    conf.png_level = 1;
    conf.collection = "db.im_raw_comp_jpeg_100";
    configurationCb(conf);
  }

  else if (count < 2 * sample_size)
  {
    conf.format = "jpeg";
    conf.jpeg_quality = 100;
      conf.png_level = 5;

      conf.collection = "db.im_raw_comp_jpeg_80";
    configurationCb(conf);
  }

  else if (count < 3 * sample_size)
  {
      conf.format = "jpeg";
      conf.jpeg_quality = 100;
      conf.png_level = 9;

      conf.collection = "db.im_raw_comp_jpeg_50";
      configurationCb(conf);
  }

  else if (count < 4 * sample_size)
  {
    conf.format = "png";
    conf.png_level = 1;
      conf.jpeg_quality = 100;

      conf.collection = "db.im_raw_comp_png_1";
    configurationCb(conf);
  }

  else if (count < 5 * sample_size)
  {
    conf.format = "png";
    conf.png_level = 1;
      conf.jpeg_quality = 50;

      conf.collection = "db.im_raw_comp_png_2";
    configurationCb(conf);
  }

  else if (count < 6 * sample_size)
  {
      conf.format = "png";
      conf.png_level = 1;
      conf.jpeg_quality = 30;

      conf.collection = "db.im_raw_comp_png_5";
      configurationCb(conf);
  }

  else if (count < 8 * sample_size)
  {
    conf.format = "png";
    conf.png_level = 7;
    conf.collection = "db.im_raw_comp_png_7";
    configurationCb(conf);
  }

  else if (count >= 8 * sample_size)
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
  matrixFunction();

  initialize();
  BSONObjBuilder document;
  std::string collection = g_cfg.collection;

  ROS_DEBUG_STREAM("Saving Image at sec " << msg->header.stamp.sec << " with type " << msg->format);

  Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
  document.append("header", BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
  document.append("format", msg->format);
  document.appendBinData("data", msg->data.size(), BinDataGeneral, (&msg->data[0]));

  add_meta_for_msg<sensor_msgs::CompressedImage>(msg, document);
  mongodb_conn->insert(collection, document.obj());
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

  // Check connection to MongoDB
  string err = string("");
  string db_host = g_cfg.db_host;
  mongodb_conn = new DBClientConnection(/* auto reconnect*/ true);
  if (!mongodb_conn->connect(g_cfg.db_host, err))
  {
    ROS_ERROR("Failed to connect to MongoDB: %s", err.c_str());
  }

  string topic = g_cfg.topic;

  // image_transport sub
  image_transport::ImageTransport it(n);
  image_transport::TransportHints th("compressed");
  image_transport::Subscriber img_sub = it.subscribe(topic, 1, imageCb, ros::VoidPtr(), th);

  ros::Subscriber sub_kinect = n.subscribe<sensor_msgs::CompressedImage>(topic + "/compressed", 100, compressedImageCb);

  ros::Rate r(1.0);

  while (n.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
