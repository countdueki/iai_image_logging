//
// Created by tammo on 26.07.18.
//

#include "logger.h"

bool logCb(iai_image_logging_msgs::LogRequestConstPtr& req, iai_image_logging_msgs::LogResponseConstPtr& res)
{
/*  for (iai_image_logging_msgs::DefaultConfig cfg : g_cfg_multi)
  {
    // TODO Move Saving here (callback to multiple topics)
  }*/
}

/**
 * Starting the main node for image logging
 * @param argc TODO
 * @param argv TODO
 * @return 0 on successful execution
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "logger");

  ros::NodeHandle n;

  // ros::ServiceServer ser_log = n.advertiseService("image_logger/log", logCb);
  // ros::Subscriber sub_kinect = n.subscribe<sensor_msgs::CompressedImage>(topic + "/compressed", 100,
  // compressedImageCb);

  ros::Rate r(1.0);

  ROS_INFO_STREAM("LOGGER: I have been called and run!");
  while (n.ok())
  {
    ROS_INFO_STREAM("LOGGER: Me spinniiiiiiing!");
    ros::spinOnce();
    r.sleep();
  }

  return (0);
}
