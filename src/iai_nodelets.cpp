//
// Created by tammo on 25.09.18.
//

#include "iai_nodelets.h"

PLUGINLIB_EXPORT_CLASS(iai_nodelets::RawNodelet, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(iai_nodelets::CompressedNodelet, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(iai_nodelets::TheoraNodelet, nodelet::Nodelet)

namespace iai_nodelets
{
void CompressedNodelet::onInit()
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

void RawNodelet::onInit()
{
  ros::NodeHandle nh_;
  ros::Rate hz_rate(1.0);
  NODELET_INFO("Initializing raw nodelet...");

  while (nh_.ok())
  {
    NODELET_INFO("and raw again...");

    ros::spinOnce();
    hz_rate.sleep();
  }
}

void TheoraNodelet::onInit()
{
  ros::NodeHandle nh_;
  ros::Rate hz_rate(1.0);
  NODELET_INFO("Initializing theora nodelet...");

  while (nh_.ok())
  {
    NODELET_INFO("and theora again...");

    ros::spinOnce();
    hz_rate.sleep();
  }
}
}