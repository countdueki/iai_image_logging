//
// Created by tammo on 13.09.18.
//

#ifndef IAI_IMAGE_LOGGING_STORAGE_H
#define IAI_IMAGE_LOGGING_STORAGE_H

#include <ros/ros.h>
#include "ros/subscribe_options.h"
#include <string>
#include <mongo/client/dbclient.h>
#include <theora_image_transport/Packet.h>
#include <image_transport/image_transport.h>
#include <compressed_image_transport/compressed_subscriber.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/CompressedImage.h>
#include <iai_image_logging_msgs/Update.h>
#include <iai_image_logging_msgs/Delete.h>

// opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <vector>

using std::string;
using ros::Subscriber;
using std::vector;
typedef std::multimap<Subscriber, int> ModeSubscriber;

enum
{
  RAW,
  COMPRESSED,
  THEORA,
  DEPTH,
  COMPRESSED_DEPTH
};

#endif  // IAI_IMAGE_LOGGING_STORAGE_H
