//
// Created by tammo on 25.09.18.
//

#include "raw_nodelet.h"

PLUGINLIB_EXPORT_CLASS(iai_nodelets::RawNodelet, nodelet::Nodelet)

namespace iai_nodelets
{
void RawNodelet::create(ros::NodeHandle nh, int mode, string topic, string collection,
                        mongo::DBClientConnection* client_connection)
{
  mode_ = mode;
  topic_ = topic;
  collection_ = collection;
  client_connection_ = client_connection;
}
void RawNodelet::setParameters(string topic, string collection, string db_host, int mode)
{
  mode_ = mode;
  topic_ = topic;
  collection_ = collection;
  db_host_ = db_host;
}
/**
* image callback to save raw and depth images
* @param msg raw and depth images
*/
void RawNodelet::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{  // matrixFunction(); // for building test entries

  NODELET_WARN("RAW_NODELET: INSIDE CALLBACK");
  NODELET_WARN_STREAM("RAW_NODELET_MODE: " << mode_);
  if (mode_ == RAW || mode_ == DEPTH)
  {
    NODELET_WARN("RAW_NODELET: INSIDE IF_CLAUSE");

    ROS_DEBUG_STREAM("msg format: " << msg->encoding);
    ROS_DEBUG_STREAM("storage TOPIC: " << topic_);
    ROS_DEBUG_STREAM("storage COLLECTION: " << collection_);
    mongo::BSONObjBuilder document;
    mongo::Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
    document.append("header", BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
    document.append("encoding", msg->encoding);
    document.append("width", msg->width);
    document.append("height", msg->height);
    document.append("is_bigendian", msg->is_bigendian);
    document.append("step", msg->step);

    document.appendBinData("data", msg->data.size(), mongo::BinDataGeneral, &msg->data);
    std::string type(ros::message_traits::DataType<sensor_msgs::Image>::value());
    document.append("type", type);
    document.append("size", (int)msg->data.size());
    if (mode_ == RAW)
    {
      document.append("mode_", "raw");
    }
    else if (mode_ == DEPTH)
    {
      document.append("mode_", "depth");
    }
    client_connection_->insert(collection_, document.obj());
  }
  else
  {
    ROS_DEBUG_STREAM("No compressed images will be logged");
  }
}

// TODO remove fake callback
void fakeCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_WARN_STREAM("I AM SO FAAAAAAAAAAAAAAAAAAAAAAAAAKE!");
}

void RawNodelet::onInit()
{
  ros::Rate hz_rate(60.0);
  NODELET_INFO("Initializing raw nodelet...");
Subscriber subbi = nh_.subscribe("camera/rgb/image_raw",1,&fakeCallback);
  while (nh_.ok())
  {
    NODELET_DEBUG("and raw again...");

    ros::spinOnce();
    hz_rate.sleep();
  }
}

void RawNodelet::setMode(int mode_)
{
  RawNodelet::mode_ = mode_;
}

    const ros::NodeHandle &RawNodelet::getNh_() const {
      return nh_;
    }
}