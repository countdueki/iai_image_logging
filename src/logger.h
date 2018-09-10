//
// Created by tammo on 06.09.18.
//

#ifndef IAI_IMAGE_LOGGING_LOGGER_H
#define IAI_IMAGE_LOGGING_LOGGER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <mongo/client/dbclient.h>
#include <mongodb_store/util.h>
#include <mongodb_store/message_store.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <iai_image_logging_msgs/Process.h>

using std::string;
using mongo::client::initialize;
using mongo::BSONObjBuilder;
using mongo::Date_t;
using mongo::DBClientConnection;
using mongo::BinDataGeneral;
using mongodb_store::add_meta_for_msg;

class Logger
{
public:
    Logger(std::string topic)
    {
      // Check connection to MongoDB
      string err = string("");
      if (!dbClientConnection->connect(db_host, err))
      {
        ROS_ERROR("Failed to connect to MongoDB: %s", err.c_str());
      }

      subscriber_compressed = nodeHandle.subscribe(topic, 1, &Logger::saveCompressedImagesCb, this);
      //subscriber_theora = nodeHandle.subscribe(topic, 1, &Logger::saveTheoraVideoCb, this);

      serviceServer = nodeHandle.advertiseService("logger/update", &Logger::update,
                                                  this);  // TODO how does 'this' fix the problem of non-static reference?
    };


    Logger(int method)
    {
      // TODO update method  topic selection
      // Check connection to MongoDB
      string err = string("");
      if (!dbClientConnection->connect(db_host, err))
      {
        ROS_ERROR("Failed to connect to MongoDB: %s", err.c_str());
      }
      string topic = "";
      subscriber_compressed = nodeHandle.subscribe(topic, 1, &Logger::saveCompressedImagesCb, this);
      subscriber_theora = nodeHandle.subscribe(topic, 1, &Logger::saveTheoraVideoCb, this);

      serviceServer = nodeHandle.advertiseService("logger/update", &Logger::update,
                                                  this);  // TODO how does 'this' fix the problem of non-static reference?
    };
  void saveImagesCb(const sensor_msgs::ImagePtr& msg)
  {
    initialize();
    BSONObjBuilder document;

    Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
    document.append("header", BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
    document.append("encoding", msg->encoding);
    document.appendBinData("data", msg->data.size(), BinDataGeneral, (&msg->data[0]));

    add_meta_for_msg<sensor_msgs::Image>(msg, document);
    dbClientConnection->insert(collection, document.obj());
  }

  void saveTheoraVideoCb(const sensor_msgs::ImagePtr& msg)
  {
    initialize();
    BSONObjBuilder document;

    Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
    document.append("header", BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
    document.append("encoding", msg->encoding);
    document.appendBinData("data", msg->data.size(), BinDataGeneral, (&msg->data[0]));

    add_meta_for_msg<sensor_msgs::Image>(msg, document);
    dbClientConnection->insert(collection, document.obj());
  }

  void saveCompressedImagesCb(const sensor_msgs::CompressedImagePtr& msg)
  {  // matrixFunction(); // for building test entries

    initialize();
    BSONObjBuilder document;

    ROS_DEBUG_STREAM("Saving Image at sec " << msg->header.stamp.sec << " with type " << msg->format);

    Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
    document.append("header", BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
    document.append("format", msg->format);
    document.appendBinData("data", msg->data.size(), BinDataGeneral, (&msg->data[0]));

    add_meta_for_msg<sensor_msgs::CompressedImage>(msg, document);
    dbClientConnection->insert(collection, document.obj());
  }

  bool update(iai_image_logging_msgs::ProcessRequest& req, iai_image_logging_msgs::ProcessResponse& res)
  {
    collection = req.set.collection;
    db_host = req.set.db_host;

    res.success = true;
    return true;
  }

private:
  ros::NodeHandle nodeHandle;
  ros::Subscriber subscriber_compressed;
  ros::Subscriber subscriber_theora;
  ros::ServiceServer serviceServer;
  DBClientConnection* dbClientConnection = new DBClientConnection(/* auto reconnect*/ true);
  std::string db_host = "localhost";
  std::string collection = "db.standard_collection";

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

  const ros::ServiceServer& getServiceServer() const
  {
    return serviceServer;
  }

  DBClientConnection* getDbClientConnection() const
  {
    return dbClientConnection;
  }

  const string& getDb_host() const
  {
    return db_host;
  }

  const string& getCollection() const
  {
    return collection;
  }

  const ros::Subscriber getSubscriber_theora() const
  {
    return subscriber_theora;
  }
};
#endif  // IAI_IMAGE_LOGGING_LOGGER_H
