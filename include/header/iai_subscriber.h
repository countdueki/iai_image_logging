//
// Created by tammo on 19.10.18.
//

#ifndef IAI_IMAGE_LOGGING_STORAGE_SUB_H
#define IAI_IMAGE_LOGGING_STORAGE_SUB_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <ros/callback_queue.h>
#include <theora_image_transport/Packet.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <iai_image_logging_msgs/Update.h>
#include <mongo/client/dbclient.h>

#include "../features/blur_detector.h"
#include "../features/similarity_detector.h"
#include "../features/motion_detector.h"
#include <boost/date_time/posix_time/posix_time.hpp>
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
    if (req.rate <= 0.0)
      rate_ = 30.0;
    motion_ = req.motion;
    blur_ = req.blur;
    similar_ = req.similar;
    tf_msg_str_ = req.tf_msg_str;
    tf_base_ = req.tf_base;
    tf_camera_ = req.tf_cam;

    // start condtitions
    prev_image_ = false;
    prev_tf_ = false;
    motion_detected_ = false;
    blur_detected_ = false;
    similar_detected_ = false;
    // create actual subscriber
    createSubscriber(getNumberFromModeString(mode_str_));
    createPublisher(getNumberFromModeString(mode_str_));
    ROS_DEBUG_STREAM("tf topic: " << req.tf_msg_str << " | tf_base: " << tf_base_ << " | tf_cam: " << tf_camera_);
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
  string topic_, mode_str_, collection_, id_, tf_msg_str_, tf_base_, tf_camera_;
  tf::tfMessageConstPtr tf_msg_, tf_msg_prev_;

public:
  const string& getTopic() const;

private:
  int mode_;
  double rate_;
  bool motion_, blur_, similar_, motion_detected_, blur_detected_, similar_detected_, prev_image_, prev_tf_;

  sensor_msgs::ImageConstPtr sim_prev_, sim_curr_;
  sensor_msgs::CompressedImageConstPtr sim_prev_c_, sim_curr_c_;
  BlurDetector blur_detector;
  SimilarityDetector sim_detector;
  MotionDetector motion_detector;

public:
    /**
     * Start the Asynchronous spinner for the subscriber
     */
  void start();

  /**
   * Shut the subscriber down
   */
  void destroy();

  /**
   * Callback method for raw images
   * @param msg raw image received
   */
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  /**
   * Callback method for compressed images
   * @param msg compressed image received
   */
  void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& msg);

  /**
   * Callback method for theora video streams
   * @param msg theora packet received
   */
  void theoraCallback(const theora_image_transport::PacketConstPtr& msg);

  /**
   * Callback method for transforms
   * @param msg transform received
   */
  void tfCallback(const tf::tfMessageConstPtr& msg);

  /**
   * Saves a raw image to the mongoDB
   * @param msg raw image
   */
  void saveImage(const sensor_msgs::ImageConstPtr& msg);

  /**
   * Saves a compressed image to the mongoDB
   * @param msg compressed image
   */
  void saveCompressedImage(const sensor_msgs::CompressedImageConstPtr& msg);

  /**
   * Saves a theora packet to the mongoDB
   * @param msg theora packet
   */
  void saveTheora(const theora_image_transport::PacketConstPtr& msg);

  /**
   * Creates a subscriber defined by a certain mode.
   * @param mode raw, compressed, depthCompressed or theora
   */
  void createSubscriber(int mode);

  /**
   * Creates a publisher, advertising those images that were saved
   * @param mode raw, compressed, depthCompressed or theora
   */
  void createPublisher(int mode);

  /**
   *  Transforms a mode string to a mode number
   * @param mode raw, compressed, depthCompressed or theora
   * @return enum int that represents the compression mode
   */
  int getNumberFromModeString(string mode);

  /**
   * Generates a unique ID from topic and mode
   * @param topic topic used for ID
   * @param mode_str mode used for ID
   * @return unique subscriber-ID
   */
  string generateID(string topic, string mode_str);

  const string& getID() const;

  void setRate_(double rate_);

  bool isMotion_() const;

  void setMotion_(bool motion_);

  bool isBlur_() const;

  void setBlur_(bool blur_);

  bool isSimilar_() const;

  void setSimilar_(bool similar_);
};

#endif  // IAI_IMAGE_LOGGING_STORAGE_SUB_H
