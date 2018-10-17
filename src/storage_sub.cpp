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
    blur_ = true;
    similar_ = true;
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
    id_ = std::to_string(req.cam_no) + std::to_string(req.mode) +  "_" + topic_;
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

      mongo::BSONObjBuilder meta;

      ros::Time now = ros::Time::now();
      mongo::Date_t nowDate((now.sec * 1000.0) + (now.nsec / 1000000.0));
      meta.append("inserted_at", nowDate);

      meta.append("stored_type", type);
      meta.append("topic", topic_);

      size_t slashIndex = type.find('/');
      std::string package = type.substr(0, slashIndex);
      std::string name = type.substr(slashIndex + 1, type.length() - slashIndex - 1);
      std::string pythonName = package + ".msg._" + name + "." + name;
      meta.append("stored_class", pythonName);

      document.append("_meta", meta.obj());

    client_connection_->insert(collection_, document.obj());
  }

  /**
   * image callback to save raw and depth images
   * @param msg raw and depth images
   */
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    ros::Rate r(rate_);
    if (motion_){
      if (motion_detected_) {
        ROS_DEBUG_STREAM("motion detected");
        return;
      }
    }
    if (blur_){
      blur_detected_ = blur_detector.detectBlur(msg);
      if (blur_detected_){
        ROS_WARN_STREAM("blur detected");
          return;

      }
    }
    if (similar_){
      if (prev_image_){
          sim_curr_ = msg;
        similar_detected_ = sim_detector.detectMSE(sim_prev_, sim_curr_);
        if (similar_detected_){
          ROS_WARN_STREAM("similar image detected");

          sim_prev_ = sim_curr_;
            return;


        } else if (!motion_detected_ && !blur_detected_){
          sim_prev_ = sim_curr_;
        }
      } else {
        sim_prev_ = msg;
        prev_image_ = true;
      }
    }
    if (!motion_detected_ && !blur_detected_ && !similar_detected_ ){
        saveImage(msg);
        pub_.publish(msg);
        //show(msg);
    }
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

        mongo::BSONObjBuilder meta;

        ros::Time now = ros::Time::now();
        mongo::Date_t nowDate((now.sec * 1000.0) + (now.nsec / 1000000.0));
        meta.append("inserted_at", nowDate);

        meta.append("stored_type", type);
        meta.append("topic", topic_);

        size_t slashIndex = type.find('/');
        std::string package = type.substr(0, slashIndex);
        std::string name = type.substr(slashIndex + 1, type.length() - slashIndex - 1);
        std::string pythonName = package + ".msg._" + name + "." + name;
        meta.append("stored_class", pythonName);

        document.append("_meta", meta.obj());
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

      if (motion_){
          if (motion_detected_) {
              ROS_DEBUG_STREAM("motion detected");
              return;
          }
      }
      if (blur_){
          blur_detected_ = blur_detector.detectBlur(msg);
          if (blur_detected_) {
              ROS_WARN_STREAM("blur detected");
              return;
          }
      }
      if (similar_){
          if (prev_image_){
              sim_curr_c_ = msg;
              similar_detected_ = sim_detector.detectMSE(sim_prev_c_, sim_curr_c_);
              if (similar_detected_){
                  ROS_WARN_STREAM("similar image detected");
                  sim_prev_c_ = sim_curr_c_;
                  return;
              } else if (!motion_detected_ && !blur_detected_){
                  sim_prev_c_ = sim_curr_c_;
              }
          } else {
              sim_prev_c_ = msg;
              prev_image_ = true;
          }
      }
      if (!motion_detected_ && !blur_detected_ && !similar_detected_ ){
          saveCompressedImage(msg);
          pub_.publish(msg);
          //show(msg);

      }

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

    void createPublisher(int mode)
    {
        switch (mode)
        {
            case (RAW):
            case (DEPTH):

                pub_ = nh_.advertise<sensor_msgs::Image>("/" + id_, 1, true);
                break;

            case (COMPRESSED):
            case (COMPRESSED_DEPTH):
                pub_ = nh_.advertise<sensor_msgs::CompressedImage>("/" + id_, 1, true);

                break;

            case (THEORA):
                pub_ = nh_.advertise<theora_image_transport::Packet>("/" + id_, 1, true);
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

  void show(const sensor_msgs::ImageConstPtr& msg){
      cv_bridge::CvImagePtr img;
      img = cv_bridge::toCvCopy(msg);
      cv::imshow("Image recorded",img->image);
      waitKey(1);
  }
    void show(const sensor_msgs::CompressedImageConstPtr& msg){
            cv::namedWindow("Compressed image recorded");
          cv_bridge::CvImageConstPtr img;
          img = cv_bridge::toCvCopy(msg);
          cv::imshow("Compressed image recorded",img->image);
          waitKey(1);


    }
};

#endif  // IAI_IMAGE_LOGGING_STORAGE_SUB_H
