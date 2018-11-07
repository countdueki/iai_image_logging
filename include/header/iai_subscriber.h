//
// Created by tammo on 19.10.18.
//

#ifndef IAI_IMAGE_LOGGING_STORAGE_SUB_H
#define IAI_IMAGE_LOGGING_STORAGE_SUB_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <ros/callback_queue.h>
#include <theora_image_transport/Packet.h>
#include <mongodb_store/message_store.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <iai_image_logging_msgs/Update.h>
#include <mongo/client/dbclient.h>

#include "../features/blur_detector.h"
#include "../features/similarity_detector.h"
#include "../features/motion_detector.h"

#include <string>

using ros::Subscriber;
using ros::SubscribeOptions;
using ros::NodeHandle;
using ros::CallbackQueue;
using ros::AsyncSpinner;
using std::string;
using mongo::DBClientConnection;

enum
{
  RAW,
  COMPRESSED,
  THEORA,
  DEPTH,
  COMPRESSED_DEPTH
};

class IAISubscriber
{
public:
  explicit IAISubscriber(DBClientConnection& connection)
  {
    nh_.setCallbackQueue(&queue_);
    spinner_ = new AsyncSpinner(1, &queue_);
    client_connection_ = &connection;
    topic_ = "/camera/rgb/image_raw";
    mode_str_ = "raw";
    mode_ = getNumberFromModeString(mode_str_);
    mode_ = 0;
    id_ = generateID(topic_, mode_str_);
    collection_ = "db.standard";  // adds mode and cam# as suffix
    rate_ = 1.0;
    motion_ = false;
    blur_ = false;
    similar_ = false;
    prev_image_ = false;
    motion_detected_ = false;
    blur_detected_ = false;
    similar_detected_ = false;

    // create actual subscriber

    ops_.template init<sensor_msgs::Image>(topic_, 1, boost::bind(&IAISubscriber::imageCallback, this, _1));
    ops_.transport_hints = ros::TransportHints();
    ops_.allow_concurrent_callbacks = true;
    sub_ = nh_.subscribe(ops_);

    // create publisher
    pub_ = nh_.advertise<sensor_msgs::Image>("/" + id_, 1, true);
  }

  IAISubscriber(DBClientConnection& connection, iai_image_logging_msgs::UpdateRequest& req, double rate = 3.0,
                bool motion = false, bool blur = false, bool similar = false)
  {
    nh_.setCallbackQueue(&queue_);
    spinner_ = new AsyncSpinner(1, &queue_);
    client_connection_ = &connection;
    mode_str_ = req.mode;
    mode_ = getNumberFromModeString(mode_str_);
    topic_ = req.topic;  // concatenate base topic and mode string
    id_ = generateID(req.topic, mode_str_);
    collection_ = req.collection;
    rate_ = req.rate;
    motion_ = req.motion;
    blur_ = req.blur;
    similar_ = req.similar;
    tf_base_ = req.tf_base;
    tf_camera_ = req.tf_cam;

    // start condtitions
    prev_image_ = false;
    motion_detected_ = false;
    blur_detected_ = false;
    similar_detected_ = false;
    // create actual subscriber
    createSubscriber(getNumberFromModeString(mode_str_));
    createPublisher(getNumberFromModeString(mode_str_));
    tf_sub_ = nh_.subscribe(req.tf_msg_str, 1, &IAISubscriber::tfCallback, this);
    ROS_WARN_STREAM("tf topic: " << req.tf_msg_str << " | tf_base: " << tf_base_ << " | tf_cam: " << tf_camera_);
  }
  //~IAISubscriber(){};

private:
  NodeHandle nh_;
  CallbackQueue queue_;
  Subscriber sub_, tf_sub_;
  ros::Publisher pub_;
  SubscribeOptions ops_;
  AsyncSpinner* spinner_;
  DBClientConnection* client_connection_;
  string topic_, mode_str_, collection_, id_, tf_base_, tf_camera_;
  tf::tfMessageConstPtr tf_msg_, tf_msg_prev_;

public:
  const string& getTopic() const;

private:
  int mode_;
  double rate_;
  bool motion_, blur_, similar_, motion_detected_, blur_detected_, similar_detected_, prev_image_;

  sensor_msgs::ImageConstPtr sim_prev_, sim_curr_;
  sensor_msgs::CompressedImageConstPtr sim_prev_c_, sim_curr_c_;
  BlurDetector blur_detector;
  SimilarityDetector sim_detector;
  MotionDetector motion_detector;

public:
  void start();

  void destroy();

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& msg);

  void theoraCallback(const theora_image_transport::PacketConstPtr& msg);

  void tfCallback(const tf::tfMessageConstPtr& msg);

  void saveImage(const sensor_msgs::ImageConstPtr& msg);

  void saveCompressedImage(const sensor_msgs::CompressedImageConstPtr& msg);

  void saveTheora(const theora_image_transport::PacketConstPtr& msg);

  void createSubscriber(int mode);

  void createPublisher(int mode);

  int getNumberFromModeString(string mode);

  string generateID(string topic, string mode_str);

  const string& getID() const;

  void setRate_(double rate_);

  bool isMotion_() const;

  void setMotion_(bool motion_);

  bool isBlur_() const;

  void setBlur_(bool blur_);

  bool isSimilar_() const;

  void setSimilar_(bool similar_);

  void show(const sensor_msgs::ImageConstPtr& msg);

  void show(const sensor_msgs::CompressedImageConstPtr& msg);

  string addIdentifier(string collection);
};

#endif  // IAI_IMAGE_LOGGING_STORAGE_SUB_H
