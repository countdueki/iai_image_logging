//
// Created by tammo on 25.09.18.
//

#include "compressed_nodelet.h"

PLUGINLIB_EXPORT_CLASS(iai_nodelets::CompressedNodelet, nodelet::Nodelet)

namespace iai_nodelets
{
void CompressedNodelet::onInit()
{
  ros::NodeHandle nh_ = getMTNodeHandle();
  ros::Rate hz_rate(60.0);
  NODELET_INFO("Initializing compressed nodelet...");

  while (nh_.ok())
  {
    NODELET_INFO("and compressed again...");

    ros::spinOnce();
    hz_rate.sleep();
  }
}
}