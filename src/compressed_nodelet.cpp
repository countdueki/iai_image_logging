//
// Created by tammo on 25.09.18.
//

#include "compressed_nodelet.h"

PLUGINLIB_EXPORT_CLASS(iai_nodelets::CompressedNodelet, nodelet::Nodelet)

namespace iai_nodelets
{
void CompressedNodelet::create(ros::NodeHandle nh, int mode, string topic, string collection,
                               mongo::DBClientConnection* client_connection)
{
  mode_ = mode;
  topic_ = topic;
  collection_ = collection;
  client_connection_ = client_connection;
}
void CompressedNodelet::setParameters(string topic, string collection, string db_host, int mode)
{
  mode_ = mode;
  topic_ = topic;
  collection_ = collection;
  db_host_ = db_host;
}

/**
 * CompressedImage callback to save compressed images and compressed depth images
 * @param msg compressed image
 */
void CompressedNodelet::compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
  if (mode_ == COMPRESSED || mode_ == COMPRESSED_DEPTH)
  {
    ROS_DEBUG_STREAM("FORMAT: " << msg->format);
    ROS_DEBUG_STREAM("storage TOPIC: " << topic_);
    ROS_DEBUG_STREAM("storage COLLECTION: " << collection_);
    mongo::BSONObjBuilder document;
    mongo::Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
    document.append("header", BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
    document.append("format", msg->format);
    document.appendBinData("data", msg->data.size(), mongo::BinDataGeneral, &msg->data);
    std::string type(ros::message_traits::DataType<sensor_msgs::CompressedImage>::value());
    document.append("type", type);
    document.append("size", (int)msg->data.size());

    if (mode_ == COMPRESSED)
    {
      document.append("mode_", "compressed");
    }
    else if (mode_ == COMPRESSED_DEPTH)
    {
      document.append("mode_", "compressed_depth");
    }
    client_connection_->insert(collection_, document.obj());
  }
  else
  {
    ROS_DEBUG_STREAM("No compressed images will be logged");
  }
}

void CompressedNodelet::onInit()
{
  ros::NodeHandle nh_ = getMTNodeHandle();
  ros::Rate hz_rate(60.0);
  NODELET_INFO("Initializing compressed nodelet...");

  while (nh_.ok())
  {
    NODELET_DEBUG("and compressed again...");

    ros::spinOnce();
    hz_rate.sleep();
  }
}

void CompressedNodelet::setMode(int mode_)
{
  CompressedNodelet::mode_ = mode_;
}
}