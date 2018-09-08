//
// Created by tammo on 06.09.18.
//

#include "logger.h"

DBClientConnection* mongodb_conn;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "logger");
  Logger logger("preprocessor/images/compressed");

  /*
    ROS_INFO_STREAM("Initiating subscription of compressed image in logger");
     image_transport::ImageTransport it(nh);
     image_transport::TransportHints th("compressed");
     image_transport::Subscriber img_sub = it.subscribe("preprocessor/images", 1, savingImagesCb, ros::VoidPtr(), th);

      ROS_INFO_STREAM("Done initiating subscription of compressed image in logger");


      image_transport::Subscriber log_sub = it.subscribe("/preprocessor/images/compressed", 1, savingImagesCb);
  */

  ros::Rate sleep_rate = 1;
  while (logger.getNodeHandle().ok())
  {
    ros::spinOnce();
    sleep_rate.sleep();
  }
}