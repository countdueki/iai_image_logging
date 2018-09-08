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

#include "image_logger.h"

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
    ros::Subscriber new_sub = nodeHandle.subscribe(topic, 1, &Preprocessor::compressedImageCb, this);
    subscriber = new_sub;
    ROS_INFO_STREAM("Requested Topic from ImageLogger: " << req.set.topic);

    ros::service::call("logger/start", req, res);
    res.success = true;
    return true;
  }

private:
  ros::NodeHandle nodeHandle;

public:
  const ros::NodeHandle& getNodeHandle() const
  {
    return nodeHandle;
  }

  void setNodeHandle(const ros::NodeHandle& nodeHandle)
  {
    Preprocessor::nodeHandle = nodeHandle;
  }

  const ros::Subscriber& getSubscriber() const
  {
    return subscriber;
  }

  void setSubscriber(const ros::Subscriber& subscriber)
  {
    Preprocessor::subscriber = subscriber;
  }

  const ros::Publisher& getPublisher() const
  {
    return publisher;
  }

  void setPublisher(const ros::Publisher& publisher)
  {
    Preprocessor::publisher = publisher;
  }

private:
  ros::Subscriber subscriber;
  ros::Publisher publisher;
  ros::ServiceServer process_service;
  std::string topic = "camera/rgb/image_raw/compressed";

public:
  const string& getTopic() const
  {
    return topic;
  }

  void setTopic(const string& topic)
  {
    Preprocessor::topic = topic;
  }
};

#endif  // IAI_IMAGE_LOGGING_PREPROCESSOR_H
