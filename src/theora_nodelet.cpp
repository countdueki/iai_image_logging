//
// Created by tammo on 25.09.18.
//

#include "theora_nodelet.h"

PLUGINLIB_EXPORT_CLASS(iai_nodelets::TheoraNodelet, nodelet::Nodelet)

namespace iai_nodelets
{
void TheoraNodelet::setParameters(string topic, string collection, string db_host, int mode)
{
  mode_ = mode;
  topic_ = topic;
  collection_ = collection;
  db_host_ = db_host;
}

void TheoraNodelet::create(ros::NodeHandle nh, int mode, string topic, string collection,
                           mongo::DBClientConnection* client_connection)
{
  mode_ = mode;
  topic_ = topic;
  collection_ = collection;
  client_connection_ = client_connection;
}
/**
* Theora Callback to save video packets
* @param msg theora video
*/
void TheoraNodelet::theoraCallback(const theora_image_transport::PacketConstPtr& msg)
{
  if (mode_ == THEORA)
  {
    ROS_DEBUG_STREAM("Theora called: " << msg->packetno);
    ROS_DEBUG_STREAM("storage TOPIC: " << topic_);
    ROS_DEBUG_STREAM("storage COLLECTION: " << collection_);
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
    document.append("size", (int)msg->data.size());
    document.append("type", type);
    document.append("mode_", "theora");
    client_connection_->insert(collection_, document.obj());
  }
  else
  {
    ROS_DEBUG_STREAM("no theora will be logged");
  }
}
void TheoraNodelet::onInit()
{
  ros::NodeHandle nh_ = getMTNodeHandle();
  ros::Rate hz_rate(60.0);
  NODELET_INFO("Initializing theora nodelet...");

  while (nh_.ok())
  {
    NODELET_DEBUG("and theora again...");

    ros::spinOnce();
    hz_rate.sleep();
  }
}

void TheoraNodelet::setMode(int mode_)
{
  TheoraNodelet::mode_ = mode_;
}
}