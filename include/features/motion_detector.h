//
// Created by tammo on 24.10.18.
//

#ifndef IAI_IMAGE_LOGGING_MOTION_DETECTOR_H
#define IAI_IMAGE_LOGGING_MOTION_DETECTOR_H

#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <tf2/transform_datatypes.h>
class MotionDetector
{
private:
  double threshold;

public:
  MotionDetector() : threshold(0.01){};

  bool detectMotion(const tf::tfMessageConstPtr& tf, tf::tfMessageConstPtr tf_prev, std::string reference,
                    std::string camera)
  {
    geometry_msgs::Transform ref_point, cam_point, ref_point_prev, cam_point_prev;
    double ref_euclid, cam_euclid, ref_euclid_prev, cam_euclid_prev, abs_curr, abs_prev;

    // calculate directional vector for current tf
    for (int i = 0; i < tf->transforms.size(); i++)
    {
      if (tf->transforms.at(i).header.frame_id.find(reference) != std::string::npos)
      {
        ROS_DEBUG_STREAM("found ref");
        ref_point = tf->transforms.at(i).transform;
      }
      if (tf->transforms.at(i).child_frame_id.find(camera) != std::string::npos)
      {
        ROS_DEBUG_STREAM("found cam");

        cam_point = tf->transforms.at(i).transform;
      }
    }

    ref_euclid =
        sqrt(pow(ref_point.translation.x, 2.0) + pow(ref_point.translation.y, 2.0) + pow(ref_point.translation.z, 2.0));
    cam_euclid =
        sqrt(pow(cam_point.translation.x, 2.0) + pow(cam_point.translation.y, 2.0) + pow(cam_point.translation.z, 2.0));

    abs_curr = fabs(ref_euclid - cam_euclid);
    ROS_DEBUG_STREAM("absdiff current ref and cam vector length = " << abs_curr);

    // calculate directional vector for previous tf
    for (int i = 0; i < tf_prev->transforms.size(); i++)
    {
      if (tf_prev->transforms.at(i).header.frame_id.find(reference) != std::string::npos)
      {
        ROS_DEBUG_STREAM("found ref");
        ref_point_prev = tf_prev->transforms.at(i).transform;
      }
      if (tf_prev->transforms.at(i).child_frame_id.find(camera) != std::string::npos)
      {
        ROS_DEBUG_STREAM("found cam");

        cam_point_prev = tf_prev->transforms.at(i).transform;
      }
    }
    ref_euclid_prev = sqrt(pow(ref_point_prev.translation.x, 2.0) + pow(ref_point_prev.translation.y, 2.0) +
                           pow(ref_point_prev.translation.z, 2.0));
    cam_euclid_prev = sqrt(pow(cam_point_prev.translation.x, 2.0) + pow(cam_point_prev.translation.y, 2.0) +
                           pow(cam_point_prev.translation.z, 2.0));

    abs_prev = fabs(ref_euclid_prev - cam_euclid_prev);
    ROS_DEBUG_STREAM("absdiff previous ref and cam vector length = " << abs_prev);

    return (abs_curr <= (abs_prev - threshold) || abs_curr > (abs_prev + threshold));
  }
};
#endif  // IAI_IMAGE_LOGGING_MOTION_DETECTOR_H
