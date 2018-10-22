//
// Created by tammo on 15.10.18.
//

#ifndef IAI_IMAGE_LOGGING_SIMILARITY_DETECTOR_H
#define IAI_IMAGE_LOGGING_SIMILARITY_DETECTOR_H
#include "opencv2/opencv_modules.hpp"
#include <stdio.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d.hpp"

using namespace cv;
class SimilarityDetector
{
public:
  SimilarityDetector(){};

  bool detectBRISK(sensor_msgs::ImageConstPtr prev, sensor_msgs::ImageConstPtr curr)
  {
    cv_bridge::CvImageConstPtr prev_image = cv_bridge::toCvShare(prev);
    cv_bridge::CvImageConstPtr curr_image = cv_bridge::toCvShare(curr);

    cv::Mat src = prev_image->image;
    cv::Mat dst = curr_image->image;

    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;

    Ptr<BRISK> detector = BRISK::create();

    std::vector<KeyPoint> keypoints_1, keypoints_2;

    detector->detect(src, keypoints_1);
    detector->detect(dst, keypoints_2);

    //-- Step 2: Calculate descriptors (feature vectors)

    Mat descriptors_1, descriptors_2;
    Ptr<DescriptorExtractor> extractor = BRISK::create();
    extractor->compute(src, keypoints_1, descriptors_1);
    extractor->compute(dst, keypoints_2, descriptors_2);

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    Ptr<DescriptorMatcher> matcher = BFMatcher::create();
    std::vector<DMatch> matches;
    matcher->match(descriptors_1, descriptors_2, matches);

    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
    //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
    //-- small)
    //-- PS.- radiusMatch can also be used here.
    std::vector<DMatch> good_matches;

    //-- Draw only "good" matches
    Mat img_matches;
    drawMatches(src, keypoints_1, dst, keypoints_2, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    int thresh = 100;

    if ((keypoints_1.size() > (keypoints_2.size() - thresh)) || (keypoints_1.size() < (keypoints_2.size() + thresh)))
    {
      std::cout << keypoints_1.size() << std::endl;
      std::cout << keypoints_2.size() << std::endl;
      return true;
    }
    else
    {
      std::cout << keypoints_1.size() << std::endl;
      std::cout << keypoints_2.size() << std::endl;
      return false;
    }

    /*
    //-- Show detected matches
    imshow("Good Matches", img_matches);

    for (int i = 0; i < (int)good_matches.size(); i++)
    {
      printf("-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx,
             good_matches[i].trainIdx);
    }
       waitKey(0);
 */
  }

  bool detectMSE(sensor_msgs::ImageConstPtr prev, sensor_msgs::ImageConstPtr curr)
  {
    cv_bridge::CvImageConstPtr prev_image = cv_bridge::toCvShare(prev);
    cv_bridge::CvImageConstPtr curr_image = cv_bridge::toCvShare(curr);

    cv::Mat src = prev_image->image;
    cv::Mat dst = curr_image->image;

    cv::Mat diffImage;
    cv::absdiff(src, dst, diffImage);

    cv::Mat square_diffImage = diffImage.mul(diffImage);

    double thresh = 80.0;
    double mse = 0.0;
    square_diffImage.convertTo(square_diffImage, CV_32F);
    Scalar sum = cv::sum(square_diffImage);

    mse = (sum.val[0] + sum.val[1] + sum.val[2] + sum.val[3]) / (src.channels() * src.total());
    if (mse < thresh)
    {
      std::cout << mse << std::endl;
      return true;
    }
    else
    {
      std::cout << mse << std::endl;
      return false;
    }
  }

  bool detectMSE(sensor_msgs::CompressedImageConstPtr prev, sensor_msgs::CompressedImageConstPtr curr)
  {
    cv_bridge::CvImageConstPtr prev_image = cv_bridge::toCvCopy(prev);
    cv_bridge::CvImageConstPtr curr_image = cv_bridge::toCvCopy(curr);

    cv::Mat src = prev_image->image;
    cv::Mat dst = curr_image->image;

    cv::Mat diffImage;
    cv::absdiff(src, dst, diffImage);

    cv::Mat square_diffImage = diffImage.mul(diffImage);

    double thresh = 80.0;
    double mse = 0.0;
    square_diffImage.convertTo(square_diffImage, CV_32F);
    Scalar sum = cv::sum(square_diffImage);

    mse = (sum.val[0] + sum.val[1] + sum.val[2] + sum.val[3]) / (src.channels() * src.total());
    if (mse < thresh)
    {
      // std::cout << mse << std::endl;
      return true;
    }
    else
    {
      // std::cout << mse << std::endl;
      return false;
    }
  }
};

#endif  // IAI_IMAGE_LOGGING_SIMILARITY_DETECTOR_H
