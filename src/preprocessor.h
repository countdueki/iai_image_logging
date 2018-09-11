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
    process_compressed_service =
        nodeHandle.advertiseService("/preprocessor/process_compressed", &Preprocessor::processCompressedCb, this);
    subscriber_compressed = nodeHandle.subscribe(topic, 100, &Preprocessor::compressedImageCb, this);
    publisher = nodeHandle.advertise<sensor_msgs::CompressedImage>("preprocessor/images/compressed", 1);
    // subscriber_theora = nodeHandle.subscribe(topic, 1, &Preprocessor::theoraVideoCb, this);
    // process_theora_service =
    //    nodeHandle.advertiseService("/preprocessor/process_theora", &Preprocessor::processTheoraCb, this);
  };

  // not allowed to have a destructor
  //~Preprocessor();

  /**
 * Callback for the compressed images sent by one camera. For now just tunneling
 * @param msg compressed image
 */
  void compressedImageCb(sensor_msgs::CompressedImage msg)
  {
    publisher.publish(msg);
  }

  /**
   * Process the compressed image from Callback as wished
   * @param req
   * @param res
   * @return true if success, else false
   */
  bool processCompressedCb(iai_image_logging_msgs::ProcessRequest& req, iai_image_logging_msgs::ProcessResponse& res)
  {
    topic = req.set.topic;
    ROS_INFO_STREAM("Requested Compressed Topic from ImageLogger: " << req.set.topic);

    // Call logger to update MongoDB data
    ros::service::call("logger/update", req, res);

    res.success = true;
    return true;
  }

  /**
   * Callback for theora video
   * @param msg
   */
  void theoraVideoCb(sensor_msgs::CompressedImage msg)
  {
    publisher.publish(msg);
  }

  /**
   * Process the compressed theora image as wished
   * @param req
   * @param res
   * @return
   */
  bool processTheoraCb(iai_image_logging_msgs::ProcessRequest& req, iai_image_logging_msgs::ProcessResponse& res)
  {
    topic = req.set.topic;

    ros::service::call("logger/update", req, res);

    res.success = true;
    return true;
  }

private:
  ros::NodeHandle nodeHandle;
  ros::Subscriber subscriber_compressed;
  ros::Subscriber subscriber_theora;
  ros::Publisher publisher;
  ros::ServiceServer process_compressed_service;
  ros::ServiceServer process_theora_service;
  std::string topic = "camera/rgb/image_raw/compressed";

  // Getter functions

public:
  const ros::NodeHandle& getNodeHandle() const
  {
    return nodeHandle;
  }

  const ros::Subscriber& getSubscriber() const
  {
    return subscriber_compressed;
  }

  const ros::Publisher& getPublisher() const
  {
    return publisher;
  }

  const ros::ServiceServer& getProcess_service() const
  {
    return process_compressed_service;
  }

  const string& getTopic() const
  {
    return topic;
  }

  const ros::Subscriber& getSubscriber_theora() const
  {
    return subscriber_theora;
  }

  const ros::ServiceServer& getProcess_theora_service() const
  {
    return process_theora_service;
  }
};

#endif  // IAI_IMAGE_LOGGING_PREPROCESSOR_H
