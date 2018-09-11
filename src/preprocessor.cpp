//
// Created by tammo on 26.07.18.
//

#include "preprocessor.h"

/**
 * Preprocessor node responsible for preprocessing images
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "preprocessor");
  Preprocessor preprocessor;
  ros::Rate sleep_rate(1.0);

  while (preprocessor.getNodeHandle().ok())
  {
    ROS_INFO_STREAM("subscribed to topic " << preprocessor.getTopic());
    ros::spinOnce();
    sleep_rate.sleep();
  }

  return 0;
}
