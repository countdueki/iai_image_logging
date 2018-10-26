//
// Created by tammo on 15.10.18.
//

#ifndef IAI_IMAGE_LOGGING_BLUR_DETECTOR_H
#define IAI_IMAGE_LOGGING_BLUR_DETECTOR_H

#include <ros/ros.h>
// OpenCV
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/rgb_colors.h>

class BlurDetector
{
private:
  const double threshold;
  double result;
  bool blurred;

public:
  BlurDetector() : threshold(12.0){};

  /**
   * Based on the blur filter MIS05
   * @param img
   * @return
   */
  double spatialFrequency(const cv::Mat& img)
  {
    try
    {
      double mean_d, stddev_d, blur_result;
      int ddepth = CV_32F;
      cv::Mat dst, grad_x, grad_y, abs_grad_x, abs_grad_y, grad_norm;
      std::vector<double> mean, stddev;

      // cv::Laplacian(img,dst,ddepth, 1, delta,scale, cv::BORDER_DEFAULT);

      // Apply Sobel filter on both gradients
      cv::Sobel(img, grad_x, ddepth, 1, 0, 3);
      cv::Sobel(img, grad_y, ddepth, 0, 1, 3);

      cv::norm(grad_x, grad_y, cv::NORM_L2);
      // cv::norm( grad_y, cv::NORM_L2);

      cv::addWeighted(grad_x, 0.5, grad_y, 0.5, 0, grad_norm);

      // calculate mean of standard deviations of all channels
      cv::meanStdDev(grad_norm, mean, stddev);
      for (int i = 0; i < stddev.size(); i++)
      {
        stddev_d += stddev.at(i);
      }
      stddev_d = stddev_d / stddev.size();

      std::cout << "Result of std deviation: " << stddev_d << std::endl;
      /* cv::imshow("depth", grad_norm);
       cv::waitKey();*/
      blur_result = stddev_d;

      ROS_WARN_STREAM("Sobel filtered stddev of image: " << blur_result << " (threshold: " << threshold << ")");

      return blur_result;
    }
    catch (cv::Exception e)
    {
      ROS_ERROR_STREAM("Error calculating spatial Frequency: " << e.what());
    }
  }

  bool detectBlur(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImageConstPtr image = cv_bridge::toCvShare(msg);
    cv::Mat grey;
    cv::cvtColor(image->image, grey, CV_BGR2GRAY);

    double result = spatialFrequency(grey);
    blurred = detectBlur(result);

    return blurred;
  }

  bool detectBlur(const sensor_msgs::CompressedImageConstPtr& c_msg)
  {
    cv_bridge::CvImageConstPtr image = cv_bridge::toCvCopy(c_msg);

    cv::Mat grey;
    cv::cvtColor(image->image, grey, CV_BGR2GRAY);

    double result = spatialFrequency(grey);
    blurred = detectBlur(result);

    return blurred;
  }

  bool detectBlur(const double result)
  {
    if (result <= threshold)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
};

#endif  // IAI_IMAGE_LOGGING_BLUR_DETECTOR_H
