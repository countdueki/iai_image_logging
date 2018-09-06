//
// Created by tammo on 26.07.18.
//

#include "preprocessor.h"
#include <pluginlib/class_list_macros.h>
namespace image_logger
{
sensor_msgs::ImageConstPtr g_image_msg;
std::string g_topic = "";

/**
 * Callback for the compressed images sent by one camera
 * @param msg compressed image pointer
 */
void imageCb(sensor_msgs::ImageConstPtr msg)
{
  g_image_msg = msg;
}

/**
 * Callback for the compressed images sent by one camera
 * @param msg compressed image pointer
 */
void compressedImageCb(sensor_msgs::CompressedImage msg)
{
}

/**
 *
 * @param req
 * @param res
 * @return
 */
bool processCb(iai_image_logging_msgs::ProcessRequest& req, iai_image_logging_msgs::ProcessResponse& res)
{
  // get Images from Topic
  // preprocess Images
  ROS_INFO_STREAM("Requested Topic from ImageLogger: " << req.set.topic);
  g_topic = req.set.topic;

  res.success = true;
  return true;
}

void Preprocessor::onInit()
{
}

/**
 *
 * @param argc
 * @param argv
 * @return
 */
int preprocessorSetup(int argc, char** argv)
{
  // TODO: Use Nodelets

  ros::init(argc, argv, "preprocessor");

  ros::NodeHandle n;

  // string topic = g_cfg->topic;

  image_transport::ImageTransport it(n);
  image_transport::TransportHints th("compressed");
  image_transport::Subscriber img_sub = it.subscribe("/camera/rgb/image_raw", 1, imageCb, ros::VoidPtr(), th);

  ros::ServiceServer process_service = n.advertiseService("/preprocessor/process", processCb);
  ROS_INFO_STREAM("Subscribed to: " << g_topic);

  // do stuff with images
  ros::Publisher pub_comp_img = n.advertise<sensor_msgs::CompressedImage>("/preprocessor/compressed_images", 1);
  ros::Rate r = 1;
  while (n.ok())
  {
    if (g_topic != "")
    {
      // ros::Subscriber sub_kinect = n.subscribe<sensor_msgs::CompressedImage>(g_topic.c_str(), 100,
      // compressedImageCb);
    }
    pub_comp_img.publish(g_image_msg);
    ros::spin();
    r.sleep();
  }

  return 0;
}
}

PLUGINLIB_EXPORT_CLASS(image_logger::Preprocessor, nodelet::Nodelet)