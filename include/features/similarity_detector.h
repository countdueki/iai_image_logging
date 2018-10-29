//
// Created by tammo on 15.10.18.
//

#ifndef IAI_IMAGE_LOGGING_SIMILARITY_DETECTOR_H
#define IAI_IMAGE_LOGGING_SIMILARITY_DETECTOR_H

#include <ros/ros.h>
#include "opencv2/opencv_modules.hpp"
#include <stdio.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d.hpp"

class SimilarityDetector
{
private:
  double threshold;

public:
  SimilarityDetector() : threshold(50.0){};

  bool detectMSE(sensor_msgs::ImageConstPtr prev, sensor_msgs::ImageConstPtr curr)
  {
    try
    {
      cv::Mat prev_grey, curr_grey, diffImage;
      double mse;

      cv_bridge::CvImageConstPtr prev_image = cv_bridge::toCvShare(prev);
      cv_bridge::CvImageConstPtr curr_image = cv_bridge::toCvShare(curr);

      // convert to greyscale
      cv::cvtColor(prev_image->image, prev_grey, CV_BGR2GRAY);
      cv::cvtColor(curr_image->image, curr_grey, CV_BGR2GRAY);

      cv::norm(prev_grey, curr_grey, cv::NORM_L2);

      cv::absdiff(prev_grey, curr_grey, diffImage);

      cv::Mat square_diffImage = diffImage.mul(diffImage);

      square_diffImage.convertTo(square_diffImage, CV_32F);
      cv::Scalar sum = cv::sum(square_diffImage);

      mse = (sum.val[0] + sum.val[1] + sum.val[2] + sum.val[3]) / (prev_grey.channels() * prev_grey.total());

      ROS_DEBUG_STREAM("Mean Squared Error of similar images: " << mse << " (threshold: " << threshold << ")");

      return mse < threshold;
    }
    catch (cv::Exception e)
    {
      ROS_ERROR_STREAM("Error calculating MSE: " << e.what());
    }
  }

  bool detectMSE(sensor_msgs::CompressedImageConstPtr prev, sensor_msgs::CompressedImageConstPtr curr)
  {
    try
    {
      cv::Mat prev_grey, curr_grey, diffImage;
      double mse;

      cv_bridge::CvImageConstPtr prev_image = cv_bridge::toCvCopy(prev);
      cv_bridge::CvImageConstPtr curr_image = cv_bridge::toCvCopy(curr);

      // convert to greyscale
      cv::cvtColor(prev_image->image, prev_grey, CV_BGR2GRAY);
      cv::cvtColor(curr_image->image, curr_grey, CV_BGR2GRAY);

      cv::norm(prev_grey, curr_grey, cv::NORM_L2);

      cv::absdiff(prev_grey, curr_grey, diffImage);

      cv::Mat square_diffImage = diffImage.mul(diffImage);

      square_diffImage.convertTo(square_diffImage, CV_32F);
      cv::Scalar sum = cv::sum(square_diffImage);

      mse = (sum.val[0] + sum.val[1] + sum.val[2] + sum.val[3]) / (prev_grey.channels() * prev_grey.total());

      ROS_DEBUG_STREAM("Mean Squared Error of similar images: " << mse << " (threshold: " << threshold << ")");
      return mse < threshold;
    }
    catch (cv::Exception e)
    {
      ROS_ERROR_STREAM("Error calculating MSE: " << e.what());
    }
  }
};

#endif  // IAI_IMAGE_LOGGING_SIMILARITY_DETECTOR_H
