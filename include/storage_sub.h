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

#include "blur_detector.h"
#include "similarity_detector.h"

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

class StorageSub
{
public:
  explicit StorageSub(DBClientConnection& connection)
  {
    nh_.setCallbackQueue(&queue_);
    spinner_ = new AsyncSpinner(1, &queue_);
    client_connection_ = &connection;
    topic_ = "/camera/rgb/image_raw";
    cam_ = 0;
    mode_ = 0;
    id_ = "00_ID";
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

    ops_.template init<sensor_msgs::Image>(topic_, 1, boost::bind(&StorageSub::imageCallback, this, _1));
    ops_.transport_hints = ros::TransportHints();
    ops_.allow_concurrent_callbacks = true;
    sub_ = nh_.subscribe(ops_);

    // create publisher
    pub_ = nh_.advertise<sensor_msgs::Image>("/" + id_, 1, true);
  }

  StorageSub(DBClientConnection& connection, iai_image_logging_msgs::UpdateRequest& req, double rate = 3.0,
             bool motion = false, bool blur = false, bool similar = false)
  {
    nh_.setCallbackQueue(&queue_);
    spinner_ = new AsyncSpinner(1, &queue_);
    client_connection_ = &connection;
    topic_ = req.topic + getModeString(req.mode);
    cam_ = req.cam_no;
    mode_ = req.mode;
    id_ = generateID(req.topic, getModeString(req.mode));
    collection_ = req.collection;  // addIdentifier(req.collection);  // adds mode and cam# as suffix
    rate_ = rate;
    motion_ = motion;
    blur_ = true;
    similar_ = true;
    prev_image_ = false;
    motion_detected_ = false;
    blur_detected_ = false;
    similar_detected_ = false;
    // create actual subscriber
    createSubscriber(mode_);
    createPublisher(mode_);
  }
  //~StorageSub(){};

private:
  NodeHandle nh_;
  CallbackQueue queue_;
  Subscriber sub_;
  ros::Publisher pub_;
  SubscribeOptions ops_;
  AsyncSpinner* spinner_;
  DBClientConnection* client_connection_;
  string topic_, collection_, id_;
  int cam_, mode_;
  double rate_;
  bool motion_, blur_, similar_, motion_detected_, blur_detected_, similar_detected_, prev_image_;
  sensor_msgs::ImageConstPtr sim_prev_, sim_curr_;
  sensor_msgs::CompressedImageConstPtr sim_prev_c_, sim_curr_c_;
  BlurDetector blur_detector;
  SimilarityDetector sim_detector;

public:
  void start();

  void destroy();

  void saveImage(const sensor_msgs::ImageConstPtr& msg);

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  void saveCompressedImage(const sensor_msgs::CompressedImageConstPtr& msg);

  void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& msg);

  void saveTheora(const theora_image_transport::PacketConstPtr& msg);

  void theoraCallback(const theora_image_transport::PacketConstPtr& msg);

  void createSubscriber(int mode);

  void createPublisher(int mode);

  string addIdentifier(string collection);

  string getModeString(int mode);

  string generateID(string topic, string mode_str);

  void show(const sensor_msgs::ImageConstPtr& msg);

  void show(const sensor_msgs::CompressedImageConstPtr& msg);

  const string& getTopic() const;

  int getCam() const;

  const string& getID() const;
};

#endif  // IAI_IMAGE_LOGGING_STORAGE_SUB_H
