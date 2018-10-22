//
// Created by tammo on 15.10.18.
//

#ifndef IAI_IMAGE_LOGGING_BLUR_DETECTOR_H
#define IAI_IMAGE_LOGGING_BLUR_DETECTOR_H

// STL
#include <vector>
#include <list>
#include <string>

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
  const size_t maxHist;
  const size_t minHist;
  const size_t waitBlur;

  std::list<double> history;
  double sum;
  double avg;
  int wasBlurred;

  const size_t maxStatSize;

public:
  std::list<double> results;
  std::list<bool> isBlurred;

  BlurDetector() : threshold(0.01), maxHist(10), minHist(3), waitBlur(5), maxStatSize(500)
  {
    sum = 0;
    avg = 0;
    wasBlurred = 0;
  };

  double funcSobelStdDevOptimized(const cv::Mat& img)
  {
    const size_t height = img.rows - 2;
    const size_t width = img.cols - 2;
    const size_t n = height * width;
    float mean = 0;
    float stdDev = 0;
    cv::Mat mag(height, width, CV_32F);

#pragma omp parallel for
    for (size_t r = 0; r < height; ++r)
    {
      const uint8_t *it11, *it12, *it13, *it21, *it23, *it31, *it32, *it33;
      it11 = img.ptr<uint8_t>(r);
      it12 = it11 + 1;
      it13 = it11 + 2;
      it21 = img.ptr<uint8_t>(r + 1);
      it23 = it21 + 2;
      it31 = img.ptr<uint8_t>(r + 2);
      it32 = it31 + 1;
      it33 = it31 + 2;
      float* itO = mag.ptr<float>(r);
      float localMean = 0;

      for (size_t c = 0; c < width; ++c, ++it11, ++it12, ++it13, ++it21, ++it23, ++it31, ++it32, ++it33, ++itO)
      {
        const int t1 = *it11 - *it33;
        const int t2 = *it13 - *it31;
        const int dx = (t1 - t2 + ((*it21 - *it23) << 1));
        const int dy = (t1 + t2 + ((*it12 - *it32) << 1));
        const float mag = sqrtf(dx * dx + dy * dy);
        localMean += mag;
        *itO = mag;
      }

#pragma omp atomic
      mean += localMean;
    }
    mean /= n;

    for (size_t r = 0; r < height; ++r)
    {
      const float* it = mag.ptr<float>(r);

      for (size_t c = 0; c < width; ++c, ++it)
      {
        const float dev = *it - mean;
        stdDev += (dev * dev);
      }
    }
    stdDev /= n;

    return stdDev / mean;
  }

  bool detectBlur(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImageConstPtr image = cv_bridge::toCvShare(msg);
    cv::Mat grey;
    cv::cvtColor(image->image, grey, CV_BayerGR2GRAY);

    double result = funcSobelStdDevOptimized(grey);
    bool blurred = detectBlur(result);

    results.push_back(result);
    isBlurred.push_back(blurred);

    if (results.size() == maxStatSize)
    {
      results.pop_front();
      isBlurred.pop_front();
    }

    return blurred;
  }

  bool detectBlur(const sensor_msgs::CompressedImageConstPtr& c_msg)
  {
    cv_bridge::CvImageConstPtr image = cv_bridge::toCvCopy(c_msg);

    cv::Mat grey;
    cv::cvtColor(image->image, grey, CV_BayerGR2GRAY);

    double result = funcSobelStdDevOptimized(grey);
    bool blurred = detectBlur(result);

    results.push_back(result);
    isBlurred.push_back(blurred);

    if (results.size() == maxStatSize)
    {
      results.pop_front();
      isBlurred.pop_front();
    }

    return blurred;
  }

  bool detectBlur(const double result)
  {
    bool blurred = history.size() < minHist;

    double diff = avg - result;
    if (diff > threshold * avg)
    {
      wasBlurred = waitBlur;
      blurred = true;
    }

    if (history.size() == maxHist)
    {
      sum -= history.front();
      history.pop_front();
    }
    history.push_back(result);
    sum += result;
    avg = sum / history.size();

    if (!blurred && wasBlurred)
    {
      blurred = true;
      --wasBlurred;
    }

    return blurred;
  }
};

#endif  // IAI_IMAGE_LOGGING_BLUR_DETECTOR_H
