//
// Created by tammo on 25.09.18.
//

#include "raw_nodelet.h"

PLUGINLIB_EXPORT_CLASS(iai_nodelets::RawNodelet, nodelet::Nodelet)

namespace iai_nodelets
{
void RawNodelet::onInit()
{
  ros::NodeHandle nh_ = getMTNodeHandle();
  ros::Rate hz_rate(60.0);
  NODELET_INFO("Initializing raw nodelet...");

  while (nh_.ok())
  {
    NODELET_INFO("and raw again...");

    ros::spinOnce();
    hz_rate.sleep();
  }
}
}