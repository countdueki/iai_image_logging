/**
 * Copyright 2018 by Tammo Wuebbena, University of Bremen
 * Using functionality of the mongodb_log and mongodb_store packages
 * Author: Tammo Wuebbena <ta_wu@tzi.de>
 */

#include "image_logger.h"

DBClientConnection* mongodb_conn;

iai_image_logging_msgs::DefaultConfig g_cfg;

/**
 * Callback for the compressed images sent by one camera
 * @param msg compressed image pointer
 */
void imageCb(sensor_msgs::ImageConstPtr msg)
{

}

/**
 * Callback for the compressed images sent by one camera
 * @param msg compressed image pointer
 */
void compressedImageCb(sensor_msgs::CompressedImageConstPtr msg)
{
  initialize();
  BSONObjBuilder document;
  std::string collection = g_cfg.collection;

  ROS_INFO_STREAM("Saving Image at sec " << msg->header.stamp.sec << " with type " << msg->format);

  Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
  document.append("header", BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
  document.append("format", msg->format);
  document.appendBinData("data", msg->data.size(), BinDataGeneral, (&msg->data[0]));

  add_meta_for_msg<sensor_msgs::CompressedImage>(msg, document);
  mongodb_conn->insert(collection, document.obj());
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
  ROS_INFO_STREAM("Set parameters");
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

  ros::Subscriber sub_kinect = n.subscribe<sensor_msgs::CompressedImage>(topic + "/compressed", 1, compressedImageCb);

  ros::Rate r(1.0);

  while (n.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
