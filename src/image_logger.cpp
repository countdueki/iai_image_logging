/**
 * Copyright 2018 by Tammo Wuebbena, University of Bremen
 * Using functionality of the mongodb_log and mongodb_store packages
 * Author: Tammo Wuebbena <ta_wu@tzi.de>
 */

#include "image_logger.h"

DBClientConnection* mongodb_conn;

ImageLogger* iai_image_logger;  // initialize logger with standard topic "/camera/rgb/image_color/compressed"

/**
 * Callback for the compressed images sent by one camera
 * @param msg compressed image pointer
 */
void compressedImageCb(sensor_msgs::CompressedImage::ConstPtr msg)
{
  initialize();
  BSONObjBuilder document;
  std::string collection = iai_image_logger->getCollection();

  ROS_INFO("Saving Image at sec %u, type %s", msg->header.stamp.sec,  msg->format);

  Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
  document.append("header", BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
  document.append("format", msg->format);
  document.appendBinData("data", msg->data.size(), BinDataGeneral, (&msg->data[0]));

  add_meta_for_msg<sensor_msgs::CompressedImage>(msg, document);
  mongodb_conn->insert(collection, document.obj());
}

/**
 * Service for compression manipulation
 * @param req compression format "jpeg" or "png"
 * @param res true on success
 * @return true, always. (needed by ServiceServer)
 */
bool setConfiguration(iai_image_logging_msgs::Configuration::Request& req,
                      iai_image_logging_msgs::Configuration::Response& res)
{

  return true;
}

bool loadConfiguration(iai_image_logging_msgs::Configuration::Request& req,
                       iai_image_logging_msgs::Configuration::Response& res)
{
  return true;
}
/**
 *
 * @param cfg
 * @param level
 */
void configurationCb(iai_image_logging_msgs::DefaultConfig& cfg, uint32_t level)
{
  ROS_INFO_STREAM("Listening to topic: " << cfg.topic);
  ROS_INFO_STREAM("Database host: " << cfg.db_host);
  ROS_INFO_STREAM("Collection: " << cfg.collection);

  iai_image_logger->setTopic(cfg.topic);
  iai_image_logger->setFormat(cfg.format);
  iai_image_logger->setJpegQuality(cfg.jpeg_quality);
  iai_image_logger->setPngLevel(cfg.png_level);
  iai_image_logger->setDbHost(cfg.db_host);
  iai_image_logger->setCollection(cfg.collection);


  // execute

  string format_t = iai_image_logger->getTopic() + "/format";
  string jpeg_t = iai_image_logger->getTopic() + "/jpeg_quality";
  string png_t = iai_image_logger->getTopic() + "/png_level";

  ROS_INFO_STREAM("Format topic: " << format_t);
  ROS_INFO_STREAM("JPEG quality topic: " << jpeg_t);
  ROS_INFO_STREAM("PNG level topic: " << png_t);

  ROS_INFO_STREAM("Format request: " << cfg.format);
  ROS_INFO_STREAM("JPEG quality request: " << cfg.jpeg_quality);
  ROS_INFO_STREAM("PNG level request: " << cfg.png_level);

  ros::param::set(format_t, cfg.format);  // set compression to requested format
  ros::param::set(jpeg_t,cfg.jpeg_quality);
  ros::param::set(png_t,cfg.png_level);

  /*
     string check;
     ros::param::get(topic, check);       // check if format is requested format
     if (check == req.topic)
       res.success = 1;  // Actual success report
  */
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

  ImageLogger il_1;
  iai_image_logger = &il_1;

    // Server for dynamic_reconfigure callback
  dynamic_reconfigure::Server<iai_image_logging_msgs::DefaultConfig> server;
  dynamic_reconfigure::Server<iai_image_logging_msgs::DefaultConfig>::CallbackType f;

  f = boost::bind(&configurationCb, _1, _2);
  server.setCallback(f);

  // Handle mongodb client connection
  string topic = iai_image_logger->getTopic();
  string db_host = iai_image_logger->getDbHost();

  string err = string("");

  // Check connection to MongoDB
  mongodb_conn = new DBClientConnection(/* auto reconnect*/ true);
  if (!mongodb_conn->connect(db_host, err))
  {
    ROS_ERROR("Failed to connect to MongoDB: %s", err.c_str());
    return -1;
  }

  ros::Subscriber sub_kinect = n.subscribe<sensor_msgs::CompressedImage>(topic, 1, compressedImageCb);

  ros::ServiceServer service_cfg = n.advertiseService("image_logger/set_configuration", setConfiguration);
  ros::ServiceServer service_cfg_yaml = n.advertiseService("image_logger/load_configuration", loadConfiguration);

  ros::Rate r(1.0);

  ROS_INFO("%sImage Logger ready!\n", "\x1B[32m");

  while (n.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
