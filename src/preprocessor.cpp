//
// Created by tammo on 26.07.18.
//

#include "preprocessor.h"

sensor_msgs::ImagePtr g_image_msg(new sensor_msgs::Image());
std::string g_topic;

/**
 * Callback for the compressed images sent by one camera
 * @param msg compressed image pointer
 */
void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  g_image_msg->data = msg->data;
  g_image_msg->encoding = msg->encoding;
  g_image_msg->header = msg->header;
  g_image_msg->height = msg->height;
  g_image_msg->is_bigendian = msg->is_bigendian;
  g_image_msg->step = msg->step;
  g_image_msg->width = msg->width;
  // g_image_msg = msg;

  ROS_INFO_STREAM("msg is filled! width: " << msg->width << " height: " << msg->height);
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
  // Set up Subscribers
  ROS_INFO_STREAM("Requested Topic from ImageLogger: " << req.set.topic);
  g_topic = req.set.topic;

  res.success = true;
  return true;
}
/**
 *
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "preprocessor");

  ros::NodeHandle n;
  ros::ServiceServer process_service = n.advertiseService("/preprocessor/process", processCb);

  image_transport::ImageTransport it(n);
  // image_transport::TransportHints th("compressed");
  // image_transport::Subscriber img_sub = it.subscribe("/camera/rgb/image_raw", 1, imageCb, ros::VoidPtr(), th);

  image_transport::Subscriber img_sub = it.subscribe("/camera/rgb/image_raw", 1, imageCb);
  image_transport::Publisher img_pub = it.advertise("preprocessor/images", 1);

  ros::Rate r = 1;
  while (n.ok())
  {
    ROS_INFO_STREAM("g_img_msg is filled! width: " << g_image_msg->width << " height: " << g_image_msg->height);
    img_pub.publish(g_image_msg);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
