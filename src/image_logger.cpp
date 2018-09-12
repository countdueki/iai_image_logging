/**
 * Copyright 2018 by Tammo Wuebbena, University of Bremen
 * Using functionality of the mongodb_log and mongodb_store packages
 * Author: Tammo Wuebbena <ta_wu@tzi.de>
 */

#include "image_logger.h"
#include <theora_image_transport/Packet.h>
ImageLogger logger;
unsigned long long compressed_chunk = 0;
unsigned long long theora_chunk = 0;

void imageCb(const sensor_msgs::CompressedImageConstPtr& msg)
{  // matrixFunction(); // for building test entries

    ROS_INFO_STREAM("MSG_SIZE: " << msg->data.size() );

    ROS_INFO_STREAM("Saving Image at sec " << msg->header.stamp.sec << " with type " << "png");
    mongo::client::initialize();
    mongo::BSONObjBuilder document;
    mongo::Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
    document.append("header", BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
    document.append("format", msg->format);
    document.appendBinData("data", (int) msg->data.size(), mongo::BinDataGeneral, &msg->data);
//document.append("stored_type", type);
    logger.getClientConnection()->insert(logger.getCollection(), document.obj());
    compressed_chunk += msg->data.size();
}

/**
 * Callback for dynamic reconfiguration
 * @param cfg Configuration to update parameters of topics to be logged
 */
void mainConfigurationCb(MainConfig& cfg)
{
  dynamic_reconfigure::ReconfigureRequest req;
  dynamic_reconfigure::ReconfigureResponse res;
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
    ros::service::call(logger.getTopic(), req,res);
  ROS_WARN_STREAM("MSG CHUNK THEORA: " << cfg.collection << " with " << theora_chunk << " Bytes");
compressed_chunk = 0;
theora_chunk = 0;
}


void theoraCallback(const theora_image_transport::PacketConstPtr& msg)
{
  mongo::client::initialize();

  mongo::BSONObjBuilder document;
  mongo::Date_t timestamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
  document.append("header", BSON("seq" << msg->header.seq << "stamp" << timestamp << "frame_id" << msg->header.frame_id));
  document.append("format", "theora");
  document.append("b_o_s", msg->b_o_s);
  document.append("e_o_s", msg->e_o_s);
  document.appendBinData("data", (int)msg->data.size(), mongo::BinDataGeneral, &msg->data);
  string type(ros::message_traits::DataType<theora_image_transport::Packet>::value());
  document.append("type", type);
  logger.getClientConnection()->insert(logger.getCollection(), document.obj());
  theora_chunk += msg->data.size();
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
  if (! logger.getClientConnection()->connect(logger.getDbHost(), errmsg)) {
    ROS_ERROR("Failed to connect to MongoDB: %s", errmsg.c_str());
    return -1;
  }
    // Server for dynamic_reconfigure callback
    dynamic_reconfigure::Server<MainConfig> server;
    dynamic_reconfigure::Server<MainConfig>::CallbackType cb_type;

    cb_type = boost::bind(&mainConfigurationCb, _1);
    server.setCallback(cb_type);


  //ros::Subscriber sub_compressed = nh.subscribe(logger.getTopic() , 1, &imageCb);

  ros::Subscriber sub_theora = nh.subscribe("camera/rgb/image_raw/theora" ,1, &theoraCallback);

  ros::Rate sleep_rate(30.0);
  while (nh.ok())
  {
    ros::spinOnce();
    sleep_rate.sleep();
  }
  return 0;
}
