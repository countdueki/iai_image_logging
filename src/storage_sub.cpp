//
// Created by tammo on 12.10.18.
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
int count = 0;
using ros::Subscriber;
using ros::SubscribeOptions;
using ros::NodeHandle;
using ros::CallbackQueue;
using ros::AsyncSpinner;
using std::string;
using mongo::DBClientConnection;
sensor_msgs::ImageConstPtr prev, curr;

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

    // create actual subscriber

    ops_.template init<sensor_msgs::Image>(topic_, 1, boost::bind(&StorageSub::imageCallback, this, _1));
    ops_.transport_hints = ros::TransportHints();
    ops_.allow_concurrent_callbacks = true;
    sub_ = nh_.subscribe(ops_);
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
    id_ = std::to_string(req.cam_no) + std::to_string(req.mode) + "_" + "_ID";
    collection_ = req.collection;  // addIdentifier(req.collection);  // adds mode and cam# as suffix
    rate_ = rate;
    motion_ = motion;
    blur_ = blur;
    similar_ = similar;

    // create actual subscriber
    createSubscriber(mode_);
  }
  //~StorageSub(){};

private:
  NodeHandle nh_;
  CallbackQueue queue_;
  Subscriber sub_;
  SubscribeOptions ops_;
  AsyncSpinner* spinner_;
  DBClientConnection* client_connection_;
  string topic_, collection_, id_;
  int cam_, mode_;
  double rate_;
  bool motion_, blur_, similar_;
  BlurDetector blur_detector;
  SimilarityDetector sim_detector;

public:

    void start()
    {
        spinner_->start();
        // queue_.callAvailable();
    }
    void destroy()
    {
        sub_.shutdown();
    }

    void saveImage(const sensor_msgs::ImageConstPtr& msg)
  {
    mongo::BSONObjBuilder document;
    mongo::Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
    document.append("header", BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
    document.append("encoding", msg->encoding);
    document.append("width", msg->width);
    document.append("height", msg->height);
    document.append("is_bigendian", msg->is_bigendian);
    document.append("step", msg->step);

    document.appendBinData("data", msg->data.size(), mongo::BinDataGeneral, &msg->data[0]);
    std::string type(ros::message_traits::DataType<sensor_msgs::Image>::value());
    document.append("type", type);
    document.append("size", (int)msg->data.size());
    if (mode_ == RAW)
    {
      document.append("mode_", "raw");
    }
    else if (mode_ == DEPTH)
    {
      document.append("mode_", "depth");
    }
    // TODO check failure on second raw connection
    client_connection_->insert(collection_, document.obj());
  }

  /**
   * image callback to save raw and depth images
   * @param msg raw and depth images
   */
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    ros::Rate r(rate_);

    // Similarity Detector test
      //if (similar_){
          if (count == 0){
              prev = msg;
              saveImage(msg);
              count++;
              ROS_ERROR_STREAM("added prev");

          } else if (count == 1){

              curr = msg;
            ROS_ERROR_STREAM("added curr");

            if (sim_detector.detect(prev, curr)){
                  count = 0;
              } else {
                  prev = curr;
                  saveImage(msg);
                  count = 1;
              };
          }
      //}

      // Blur detector test
 /*     if (!blur_detector.detectBlur(msg)){
        ROS_WARN_STREAM("saving raw image");

        saveImage(msg);
      } else {
        ROS_ERROR_STREAM("I saw you moving you fool!");
      }*/
    r.sleep();
  }

  void saveCompressedImage(const sensor_msgs::CompressedImageConstPtr& msg)
  {
    try
    {
      mongo::BSONObjBuilder document;
      mongo::Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
      document.append("header",
                      BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
      document.append("format", msg->format);
      document.appendBinData("data", msg->data.size(), mongo::BinDataGeneral, &msg->data[0]);
      std::string type(ros::message_traits::DataType<sensor_msgs::CompressedImage>::value());
      document.append("type", type);
      document.append("size", (int)msg->data.size());

      if (mode_ == COMPRESSED)
      {
        document.append("mode_", "compressed");
      }
      else if (mode_ == COMPRESSED_DEPTH)
      {
        document.append("mode_", "compressed_depth");
      }
      client_connection_->insert(collection_, document.obj());
    }
    catch (std::bad_alloc& ba)
    {
      ROS_ERROR_STREAM("bad_alloc caught: " << ba.what());
    }
  }

  /**
   * CompressedImage callback to save compressed images and compressed depth images
   * @param msg compressed image
   */
  void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
  {
    ros::Rate r(rate_);

    // Blur detector test
/*    if (!blur_detector.detectBlur(msg)){
      ROS_WARN_STREAM("saving compressed image");

      saveCompressedImage(msg);
    } else {
      ROS_ERROR_STREAM("I saw you moving you compressed fool!");
    }   */
    r.sleep();
  }

  void saveTheora(const theora_image_transport::PacketConstPtr& msg)
  {
    mongo::BSONObjBuilder document;

    mongo::Date_t timestamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
    document.append("header",
                    BSON("seq" << msg->header.seq << "stamp" << timestamp << "frame_id" << msg->header.frame_id));
    document.append("format", "theora");
    document.append("start", msg->b_o_s);
    document.append("end", msg->e_o_s);
    document.append("position", (int)msg->granulepos);
    document.append("packetno", (int)msg->packetno);
    document.appendBinData("data", msg->data.size(), mongo::BinDataGeneral, &msg->data[0]);
    string type(ros::message_traits::DataType<theora_image_transport::Packet>::value());
    document.append("size", (int)msg->data.size());
    document.append("type", type);
    document.append("mode_", "theora");
    client_connection_->insert(collection_, document.obj());
  }

  /**
   * Theora Callback to save video packets
   * @param msg theora video
   */
  void theoraCallback(const theora_image_transport::PacketConstPtr& msg)
  {
    ROS_WARN_STREAM("saving theora image");
    ros::Rate r(rate_);
    saveTheora(msg);
    r.sleep();
  }

  void createSubscriber(int mode)
  {
    switch (mode)
    {
      case (RAW):
      case (DEPTH):
        ops_.template init<sensor_msgs::Image>(topic_, 1, boost::bind(&StorageSub::imageCallback, this, _1));
        ops_.transport_hints = ros::TransportHints();
        ops_.allow_concurrent_callbacks = true;
        sub_ = nh_.subscribe(ops_);
        break;

      case (COMPRESSED):
      case (COMPRESSED_DEPTH):
        ops_.template init<sensor_msgs::CompressedImage>(topic_, 1,
                                                         boost::bind(&StorageSub::compressedImageCallback, this, _1));
        ops_.transport_hints = ros::TransportHints();
        ops_.allow_concurrent_callbacks = true;
        sub_ = nh_.subscribe(ops_);
        break;

      case (THEORA):
        ops_.template init<theora_image_transport::Packet>(topic_, 1,
                                                           boost::bind(&StorageSub::theoraCallback, this, _1));
        ops_.transport_hints = ros::TransportHints();
        ops_.allow_concurrent_callbacks = true;
        sub_ = nh_.subscribe(ops_);
        break;

      default:
        break;
    }
  }

  // Getter, Setter and helper
public:
  int getCam() const
  {
    return cam_;
  }

  const string& getTopic() const
  {
    return topic_;
  }

  string addIdentifier(string collection)
  {
    switch (mode_)
    {
      case (RAW):
        collection = collection + "_raw" + "_cam_" + std::to_string(cam_);
        break;
      case (COMPRESSED):
        collection = collection + "_compressed" + "_cam_" + std::to_string(cam_);
        break;
      case (THEORA):
        collection = collection + "_theora" + "_cam_" + std::to_string(cam_);
        break;
      case (DEPTH):
        collection = collection + "_depth" + "_cam_" + std::to_string(cam_);
        break;
      case (COMPRESSED_DEPTH):
        collection = collection + "_compressedDepth" + "_cam_" + std::to_string(cam_);
        break;
      default:
        collection = collection;
        break;
    }
  }
  string getModeString(int mode)
  {
    switch (mode)
    {
      case (RAW):
        return "";
      case (COMPRESSED):
        return "/compressed";
      case (THEORA):
        return "/theora";
      case (DEPTH):
        return "";
      case (COMPRESSED_DEPTH):
        return "/compressedDepth";
      default:
        break;
    }
  }
};

#endif  // IAI_IMAGE_LOGGING_STORAGE_SUB_H
