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
    StorageSub(iai_image_logging_msgs::UpdateRequest& req, iai_image_logging_msgs::UpdateResponse& res)
    {
        // stuff;
    }
  StorageSub(DBClientConnection& client_connection, string topic, string collection, int cam = 0, int mode = 0,
             string id_string = "_standard", int rate = 30, bool motion = false, bool blur = false,
             bool similar = false)
  {
    nh_.setCallbackQueue(&queue_);
    spinner_ = new AsyncSpinner(1, &queue_);

    client_connection_ = &client_connection;

    topic_ = topic;
    cam_ = cam;
    mode_ = mode;
    id_ = std::to_string(cam) + std::to_string(mode) + "_" + id_string;
    collection_ = addIdentifier(collection);  // adds mode and cam# as suffix
    rate_ = rate;
    motion_ = motion;
    blur_ = blur;
    similar_ = similar;

    // create actual subscriber
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
        ops_.template init<sensor_msgs::CompressedImage>(topic_ + "/compressed", 1,
                                                         boost::bind(&StorageSub::compressedImageCallback, this, _1));
        ops_.transport_hints = ros::TransportHints();
        ops_.allow_concurrent_callbacks = true;
        sub_ = nh_.subscribe(ops_);
        break;

      case (THEORA):
        ops_.template init<theora_image_transport::Packet>(topic_ + "/theora", 1,
                                                           boost::bind(&StorageSub::theoraCallback, this, _1));
        ops_.transport_hints = ros::TransportHints();
        ops_.allow_concurrent_callbacks = true;
        sub_ = nh_.subscribe(ops_);
        break;

      default:
        break;
    }
    if (spinner_->canStart()) spinner_->start();
  }
  StorageSub(iai_image_logging_msgs::UpdateRequest& req, iai_image_logging_msgs::UpdateResponse& res,
             DBClientConnection& client_connection, string id_string)
  {
    StorageSub(client_connection, req.topic, req.collection, req.cam_no, req.mode, id_string);
  }
  StorageSub(DBClientConnection& client_connection)
  {
    StorageSub(client_connection, "camera/rgb/image_raw", "db.standard");
  }

  void destroy()
  {
    sub_.shutdown();
  }

  ~StorageSub(){};
private:
  NodeHandle nh_;
  CallbackQueue queue_;
  Subscriber sub_;
  SubscribeOptions ops_;
  AsyncSpinner* spinner_;
  DBClientConnection* client_connection_;
  string topic_, collection_, id_;
  int cam_, mode_, rate_;
public:
    int getCam() const {
      return cam_;
    }

private:
    bool motion_, blur_, similar_;

public:
  int getMode() const
  {
    return mode_;
  }

  const string& getTopic() const
  {
    return topic_;
  }

public:
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
    client_connection_->insert(collection_, document.obj());
  }

  /**
   * image callback to save raw and depth images
   * @param msg raw and depth images
   */
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    saveImage(msg);
    sleep(rate_);
  }

  void saveCompressedImage(const sensor_msgs::CompressedImageConstPtr& msg)
  {
    mongo::BSONObjBuilder document;
    mongo::Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
    document.append("header", BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
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

  /**
   * CompressedImage callback to save compressed images and compressed depth images
   * @param msg compressed image
   */
  void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
  {
    saveCompressedImage(msg);
    sleep(rate_);
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
    saveTheora(msg);
    sleep(rate_);
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
};

#endif  // IAI_IMAGE_LOGGING_STORAGE_SUB_H