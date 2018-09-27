//
// Created by tammo on 25.09.18.
//

#include "iai_nodelet.h"

PLUGINLIB_EXPORT_CLASS(iai_nodelets::IAINodelet, nodelet::Nodelet)

namespace iai_nodelets
{
void IAINodelet::setParameters(string topic, string collection, string db_host, int mode)
{
  mode_ = mode;
  topic_ = topic;
  collection_ = collection;
  db_host_ = db_host;
}

void IAINodelet::create(ros::NodeHandle nh, int mode, string topic, string collection,
                        mongo::DBClientConnection* client_connection)
{
  mode_ = mode;
  topic_ = topic;
  collection_ = collection;
  client_connection_ = client_connection;
}
void IAINodelet::onInit()
{
  ros::NodeHandle nh_;
  ros::Rate hz_rate(1.0);
  NODELET_INFO("Initializing compressed nodelet...");

  while (nh_.ok())
  {
    NODELET_INFO("and compressed again...");

    ros::spinOnce();
    hz_rate.sleep();
  }
}

const ros::NodeHandle& IAINodelet::getNh_() const
{
  return nh_;
}

void IAINodelet::setNh_(const ros::NodeHandle& nh_)
{
  IAINodelet::nh_ = nh_;
}

int IAINodelet::getMode_() const
{
  return mode_;
}

void IAINodelet::setMode_(int mode_)
{
  IAINodelet::mode_ = mode_;
}

const string& IAINodelet::getTopic_() const
{
  return topic_;
}

void IAINodelet::setTopic_(const string& topic_)
{
  IAINodelet::topic_ = topic_;
}

const string& IAINodelet::getCollection_() const
{
  return collection_;
}

void IAINodelet::setCollection_(const string& collection_)
{
  IAINodelet::collection_ = collection_;
}

const string& IAINodelet::getDb_host_() const
{
  return db_host_;
}

void IAINodelet::setDb_host_(const string& db_host_)
{
  IAINodelet::db_host_ = db_host_;
}

mongo::DBClientConnection* IAINodelet::getClient_connection_() const
{
  return client_connection_;
}

void IAINodelet::setClient_connection_(mongo::DBClientConnection* client_connection_)
{
  IAINodelet::client_connection_ = client_connection_;
}

const Subscriber& IAINodelet::getSub_() const
{
  return sub_;
}

void IAINodelet::setSub_(const Subscriber& sub_)
{
  IAINodelet::sub_ = sub_;
}

    void IAINodelet::createSubscriber(string topic) {

    }
}