/**
 * Copyright 2018 by Tammo Wuebbena, University of Bremen
 * Using functionality of the mongodb_log and mongodb_store packages
 * Author: Tammo Wuebbena <ta_wu@tzi.de>
 */

#include "image_logger.h"
#include <theora_image_transport/Packet.h>
ImageLogger logger;
MainConfig g_cfg;

unsigned long long compressed_chunk = 0;
unsigned long long theora_chunk = 0;

void imageCb(const sensor_msgs::CompressedImageConstPtr& msg)
{  // matrixFunction(); // for building test entries

  if (logger.getMode() == COMPRESSED)
  {
    mongo::client::initialize();
    mongo::BSONObjBuilder document;
    mongo::Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
    document.append("header", BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
    document.append("format", msg->format);
    document.appendBinData("data", msg->data.size(), mongo::BinDataGeneral, &msg->data);
    std::string type(ros::message_traits::DataType<sensor_msgs::CompressedImage>::value());
    document.append("type", type);
    logger.getClientConnection()->insert(logger.getCollection() + "/compressed", document.obj());
    compressed_chunk += msg->data.size();
  }
  else
  {
    ROS_INFO_STREAM("No compressed images will be logged");
  }
}

void theoraCallback(const theora_image_transport::PacketConstPtr& msg)
{
  if (logger.getMode() == THEORA)
  {
    ROS_DEBUG_STREAM("LOGGER TOPIC: " << logger.getTopic());
    ROS_DEBUG_STREAM("LOGGER COLLECTION: " << logger.getCollection());
    mongo::client::initialize();
    mongo::BSONObjBuilder document;

    mongo::Date_t timestamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
    document.append("header",
                    BSON("seq" << msg->header.seq << "stamp" << timestamp << "frame_id" << msg->header.frame_id));
    document.append("format", "theora");
    document.append("start", msg->b_o_s);
    document.append("end", msg->e_o_s);
    document.append("position", (int)msg->granulepos);
    document.append("packetno", (int)msg->packetno);
    document.appendBinData("data", msg->data.size(), mongo::BinDataGeneral, &msg->data);
    string type(ros::message_traits::DataType<theora_image_transport::Packet>::value());
    document.append("type", type);
    logger.getClientConnection()->insert(logger.getCollection() + "_theora", document.obj());
    theora_chunk += msg->data.size();
  }
  else
  {
    ROS_INFO_STREAM("no theora will be logged");
  }
}

void setCompressedParameters(MainConfig cfg)
{
  dynamic_reconfigure::ReconfigureRequest req;
  dynamic_reconfigure::ReconfigureResponse res;

  StrParam format;
  IntParam jpeg, png;

  format.name = "format";
  format.value = cfg.format;

  jpeg.name = "jpeg_quality";
  jpeg.value = cfg.jpeg_quality;

  png.name = "png_level";
  png.value = cfg.png_level;

  // Set parameters for compressed images
  req.config.strs.push_back(format);
  req.config.ints.push_back(jpeg);
  req.config.ints.push_back(png);

  string new_topic = logger.getTopic() + "/compressed";
  ROS_WARN_STREAM("Called Topic: " << new_topic);
  ros::service::call(logger.getTopic() + "/compressed/set_parameters", req, res);
}

void setTheoraParameters(MainConfig& cfg)
{
  dynamic_reconfigure::ReconfigureRequest req;
  dynamic_reconfigure::ReconfigureResponse res;

  IntParam optimize_for, keyframe_frequency, quality, target_bitrate;

  optimize_for.name = "optimize_for";
  optimize_for.value = cfg.optimize_for;

  target_bitrate.name = "target_bitrate";
  target_bitrate.value = cfg.target_bitrate;

  keyframe_frequency.name = "keyframe_frequency";
  keyframe_frequency.value = cfg.keyframe_frequency;

  quality.name = "quality";
  quality.value = cfg.quality;

  // Set parameters for theora video
  req.config.ints.push_back(optimize_for);
  req.config.ints.push_back(keyframe_frequency);
  req.config.ints.push_back(quality);
  req.config.ints.push_back(target_bitrate);

  string new_topic_theora = logger.getTopic() + "/theora";
  ROS_WARN_STREAM("Called Topic: " << new_topic_theora);
  ros::service::call(logger.getTopic() + "/theora/set_parameters", req, res);
}
/**
 * Callback for dynamic reconfiguration
 * @param cfg Configuration to update parameters of topics to be logged
 */
void mainConfigurationCb(MainConfig& cfg)
{
  // setCompressedParameters(cfg);
  g_cfg = cfg;
  if (logger.getMode() == COMPRESSED)
  {
    setCompressedParameters(cfg);
  }
  else
  {
    setTheoraParameters(cfg);
  }

  ROS_WARN_STREAM("MSG CHUNK THEORA: " << cfg.collection << " with " << theora_chunk << " Bytes");
  ROS_WARN_STREAM("MSG CHUNK COMPRESSED: " << cfg.collection << " with " << compressed_chunk << " Bytes");

  compressed_chunk = 0;
  theora_chunk = 0;
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
  string errmsg;
  if (!logger.getClientConnection()->connect(logger.getDbHost(), errmsg))
  {
    ROS_ERROR("Failed to connect to MongoDB: %s", errmsg.c_str());
    return -1;
  }

  // Server for dynamic_reconfigure callback
  dynamic_reconfigure::Server<MainConfig> server;
  dynamic_reconfigure::Server<MainConfig>::CallbackType cb_type;

  cb_type = boost::bind(&mainConfigurationCb, _1);
  server.setCallback(cb_type);
  ros::Subscriber sub_compressed = nh.subscribe(logger.getTopic() + "/compressed", 1, &imageCb);
  ros::Subscriber sub_theora = nh.subscribe(logger.getTopic() + "/theora", 1, &theoraCallback);

  // 60 enough, 30 is not
  ros::Rate hz_rate(60.0);
  while (nh.ok())
  {
    logger.setTopic(g_cfg.topic);
    logger.setCollection(g_cfg.collection);
    logger.setMode(g_cfg.mode);
    ros::spinOnce();
    hz_rate.sleep();
  }
  return 0;
}
