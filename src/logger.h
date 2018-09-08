//
// Created by tammo on 06.09.18.
//

#ifndef IAI_IMAGE_LOGGING_LOGGER_H
#define IAI_IMAGE_LOGGING_LOGGER_H

#include <ros/ros.h>
#include <iai_image_logging_msgs/CompressedConfig.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <mongo/client/dbclient.h>
#include <mongodb_store/util.h>
#include <mongodb_store/message_store.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include "image_logger.h"

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

    subscriber = nodeHandle.subscribe(topic, 1, &Logger::savingCompressedImagesCb, this);
    serviceServer = nodeHandle.advertiseService("logger/start", &Logger::startLogger,
                                                this);  // TODO how does 'this' fix the problem of non-static reference?
  };

private:
  ros::NodeHandle nodeHandle;
  ros::Subscriber subscriber;
  ros::ServiceServer serviceServer;
  DBClientConnection* dbClientConnection = new DBClientConnection(/* auto reconnect*/ true);

public:
  DBClientConnection* getDbClientConnection() const
  {
    return dbClientConnection;
  }

  void setDbClientConnection(DBClientConnection* dbClientConnection)
  {
    Logger::dbClientConnection = dbClientConnection;
  }

private:
  std::string db_host = "localhost";

public:
  const string& getDb_host() const
  {
    return db_host;
  }

  void setDb_host(const string& db_host)
  {
    Logger::db_host = db_host;
  }

private:
  std::string collection = "db.standard_collection";

public:
  const ros::NodeHandle& getNodeHandle() const
  {
    return nodeHandle;
  }

  void setNodeHandle(const ros::NodeHandle& nodeHandle)
  {
    Logger::nodeHandle = nodeHandle;
  }

  void savingCompressedImagesCb(const sensor_msgs::CompressedImagePtr& msg)
  {  // matrixFunction(); // for building test entries

    ROS_INFO_STREAM("cb logger");
    initialize();
    BSONObjBuilder document;

    ROS_DEBUG_STREAM("Saving Image at sec " << msg->header.stamp.sec << " with type " << msg->format);

    Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
    document.append("header", BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
    document.append("format", msg->format);
    document.appendBinData("data", msg->data.size(), BinDataGeneral, (&msg->data[0]));

    add_meta_for_msg<sensor_msgs::CompressedImage>(msg, document);
    ROS_INFO_STREAM("now saving");
    dbClientConnection->insert(collection, document.obj());
  }

  bool startLogger(iai_image_logging_msgs::ProcessRequest& req, iai_image_logging_msgs::ProcessResponse& res)
  {
    collection = req.set.collection;
    db_host = req.set.db_host;

    res.success = true;
    return true;
  }
};
#endif  // IAI_IMAGE_LOGGING_LOGGER_H
