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

using std::string;

class Preprocessor
{
public:
  Preprocessor()
  {
    process_service = nodeHandle.advertiseService("/preprocessor/process", &Preprocessor::processCb, this);
    subscriber = nodeHandle.subscribe(topic, 1, &Preprocessor::compressedImageCb, this);
    publisher = nodeHandle.advertise<sensor_msgs::CompressedImage>("preprocessor/images/compressed", 1);
  };

  // not allowed to have a destructor
  //~Preprocessor();

  /**
 * Callback for the compressed images sent by one camera
 * @param msg compressed image pointer
 */
  void compressedImageCb(sensor_msgs::CompressedImage msg)
  {
    publisher.publish(msg);
    ROS_INFO_STREAM("published msg /w format " << msg.format);
  }

  /**
*
*@param req
*@param res
*@return
*/
  bool processCb(iai_image_logging_msgs::ProcessRequest& req, iai_image_logging_msgs::ProcessResponse& res)
  {
    topic = req.set.topic;
    ROS_INFO_STREAM("Requested Topic from ImageLogger: " << req.set.topic);

    ros::service::call("logger/update", req, res);

    res.success = true;
    return true;
  }

private:
  ros::NodeHandle nodeHandle;
  ros::Subscriber subscriber;
  ros::Publisher publisher;
  ros::ServiceServer process_service;
  std::string topic = "camera/rgb/image_raw/compressed";

  // Getter functions

public:
  const ros::NodeHandle& getNodeHandle() const
  {
    return nodeHandle;
  }

  const ros::Subscriber& getSubscriber() const
  {
    return subscriber;
  }

  const ros::Publisher& getPublisher() const
  {
    return publisher;
  }

  const ros::ServiceServer& getProcess_service() const
  {
    return process_service;
  }

  const string& getTopic() const
  {
    return topic;
  }
};

#endif  // IAI_IMAGE_LOGGING_PREPROCESSOR_H
