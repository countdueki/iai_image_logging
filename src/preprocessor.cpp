//
// Created by tammo on 26.07.18.
//

#include "preprocessor.h"

/**
 * Callback for the compressed images sent by one camera
 * @param msg compressed image pointer
 */
void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  // g_image_msg = msg;

  ROS_INFO_STREAM("msg is filled! width: " << msg->width << " height: " << msg->height);
}

/**
 *
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "preprocessor");
  Preprocessor preprocessor;
  ros::Rate sleep_rate = 1;

  while (preprocessor.getNodeHandle().ok())
  {
    ROS_INFO_STREAM("subscribed to topic " << preprocessor.getTopic());
    ros::spinOnce();
    sleep_rate.sleep();
  }

  return 0;
}
