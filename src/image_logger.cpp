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
void compressedImageCb(sensor_msgs::CompressedImageConstPtr msg)
{
  initialize();
  BSONObjBuilder document;
  std::string collection = g_cfg.collection;

  ROS_INFO_STREAM("Saving Image at sec " << msg->header.stamp.sec << " with type " <<  msg->format);

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
  g_cfg = cfg;

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

  f = boost::bind(&configurationCb, _1, _2);
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

  ros::Subscriber sub_kinect = n.subscribe<sensor_msgs::CompressedImage>(topic + "/compressed", 1, compressedImageCb);


  ros::Rate r(1.0);

  ROS_INFO("%sImage Logger ready!\n", "\x1B[32m");

  while (n.ok())
  {

      ros::ServiceServer service_cfg = n.advertiseService("image_logger/set_configuration", setConfiguration);
      ros::ServiceServer service_cfg_yaml = n.advertiseService("image_logger/load_configuration", loadConfiguration);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
