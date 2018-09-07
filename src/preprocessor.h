//
// Created by tammo on 26.07.18.
//

#ifndef IAI_IMAGE_LOGGING_PREPROCESSOR_H
#define IAI_IMAGE_LOGGING_PREPROCESSOR_H
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <iai_image_logging_msgs/Process.h>

class Preprocessor
{
public:
  Preprocessor();

  ~Preprocessor();
};

#endif  // IAI_IMAGE_LOGGING_PREPROCESSOR_H
