/**
 * Copyright 2018 by Tammo Wuebbena, University of Bremen
 * Using functionality of the mongodb_log and mongodb_store packages
 * Author: Tammo Wuebbena <ta_wu@tzi.de>
 */

#include "image_logger.h"

DBClientConnection* mongodb_conn;

iai_image_logging_msgs::DefaultConfig* g_cfg;

/**
 * Callback for the compressed images sent by one camera
 * @param msg compressed image pointer
 */
void compressedImageCb(sensor_msgs::CompressedImageConstPtr msg)
{

  initialize();
  BSONObjBuilder document;
  std::string collection = g_cfg->collection;

  ROS_INFO_STREAM("Saving Image at sec " << msg->header.stamp.sec << " with type " <<  msg->format);

  Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
  document.append("header", BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
  document.append("format", msg->format);
  document.appendBinData("data", msg->data.size(), BinDataGeneral, (&msg->data[0]));

  add_meta_for_msg<sensor_msgs::CompressedImage>(msg, document);
  mongodb_conn->insert(collection, document.obj());

}

void imageCb(sensor_msgs::ImageConstPtr msg)
{
  ROS_INFO_STREAM("Topic: " << g_cfg->topic);
  ROS_INFO_STREAM("Saving Image at sec " << msg->header.stamp.sec << " with encoding " <<  msg->encoding);
  ROS_INFO_STREAM("Size: " << msg->data.size() /1000 << " KB");


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
  g_cfg = &cfg;
  ROS_INFO_STREAM("Topic: " << cfg.topic);
  ROS_INFO_STREAM("Format: " << cfg.format);
  ROS_INFO_STREAM("JPEG quality: " << cfg.jpeg_quality);
  ROS_INFO_STREAM("PNG level: " << cfg.png_level);
  ROS_INFO_STREAM("DB host: " << cfg.db_host);
  ROS_INFO_STREAM("Collection: " << cfg.collection);

  string param_topic = g_cfg->topic;
  string format_t =  param_topic + "/compressed/format";
  string jpeg_t = param_topic + "/compressed/jpeg_quality";
  string png_t = param_topic + "/compressed/png_level";

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


    // Server for dynamic_reconfigure callback
  dynamic_reconfigure::Server<iai_image_logging_msgs::DefaultConfig> server;
  dynamic_reconfigure::Server<iai_image_logging_msgs::DefaultConfig>::CallbackType f;

  f = boost::bind(&configurationCb, _1, _2);
  server.setCallback(f);

  // Handle mongodb client connection
  string topic = g_cfg->topic;
  string db_host = g_cfg->db_host;

  string err = string("");

  // Check connection to MongoDB
  mongodb_conn = new DBClientConnection(/* auto reconnect*/ true);
  if (!mongodb_conn->connect(db_host, err))
  {
    ROS_ERROR("Failed to connect to MongoDB: %s", err.c_str());
    return -1;
  }

  ros::Subscriber sub_kinect = n.subscribe<sensor_msgs::CompressedImage>(topic + "/compressed", 1, compressedImageCb);

  image_transport::ImageTransport it(n);
  image_transport::TransportHints th("compressed",ros::TransportHints(),n,topic);
  image_transport::SubscriberFilter(it,topic,1,th);
  image_transport::Subscriber img_sub = it.subscribe(topic , 1, imageCb, ros::VoidPtr(), th);

  //ros::ServiceServer service_cfg = n.advertiseService("image_logger/set_configuration", setConfiguration);
  //ros::ServiceServer service_cfg_yaml = n.advertiseService("image_logger/load_configuration", loadConfiguration);

  ros::Rate r(1.0);

  ROS_INFO("%sImage Logger ready!\n", "\x1B[32m");

  while (n.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
