//
// Created by tammo on 18.09.18.
//

#ifndef IAI_IMAGE_LOGGING_CAMERA_H
#define IAI_IMAGE_LOGGING_CAMERA_H

#include <ros/ros.h>

class Camera
{
public:
  Camera()
  {
      is_empty = false;
  }

private:
  std::vector<ros::Subscriber> sub_raw_list;
  std::vector<ros::Subscriber> sub_compressed_list;
  std::vector<ros::Subscriber> sub_theora_list;
  bool is_empty;

public:

  void init(ros::Subscriber sub_raw, ros::Subscriber sub_compressed, ros::Subscriber sub_theora)
  {
    sub_raw_list.push_back(sub_raw);
    sub_compressed_list.push_back(sub_compressed);
    sub_theora_list.push_back(sub_theora);
  }

  bool isEmpty()
  {
      return is_empty;
  }
};

#endif  // IAI_IMAGE_LOGGING_CAMERA_H
