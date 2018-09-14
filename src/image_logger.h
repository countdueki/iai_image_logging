/**
 * Copyright 2018 by Tammo Wuebbena, University of Bremen
 * Author: Tammo Wuebbena <ta_wu@tzi.de>
 */

#ifndef IAI_IMAGE_LOGGING_IMAGE_LOGGER_H
#define IAI_IMAGE_LOGGING_IMAGE_LOGGER_H

#include <ros/ros.h>

#include <boost/smart_ptr/shared_ptr.hpp>
#include <dynamic_reconfigure/server.h>

#include <iai_image_logging_msgs/Configuration.h>
#include <iai_image_logging_msgs/ConfigurationYAML.h>
#include <iai_image_logging_msgs/CompressedConfig.h>
#include <iai_image_logging_msgs/TheoraConfig.h>
#include <iai_image_logging_msgs/MainConfig.h>
#include <iai_image_logging_msgs/Process.h>
#include <mongo/client/dbclient.h>
#include <sensor_msgs/CompressedImage.h>
#include <iostream>

typedef iai_image_logging_msgs::MainConfig MainConfig;

typedef dynamic_reconfigure::StrParameter StrParam;
typedef dynamic_reconfigure::IntParameter IntParam;
typedef dynamic_reconfigure::DoubleParameter DoubleParam;

using std::string;

enum
{
  RAW,
  COMPRESSED,
  THEORA
};
class ImageLogger
{
public:
  ImageLogger()
  {
    clientConnection = new mongo::DBClientConnection(true);
    dbHost = "localhost";
    collection = "db.standard";
    topic = "camera/rgb/image_raw";
  }

private:
  string topic;
  string dbHost;
  string collection;
  int mode;
  mongo::DBClientConnection* clientConnection;

public:
  const string& getTopic() const
  {
    return topic;
  }

  void setTopic(const string& topic)
  {
    ImageLogger::topic = topic;
  }

  const string& getDbHost() const
  {
    return dbHost;
  }

  void setDbHost(const string& dbHost)
  {
    ImageLogger::dbHost = dbHost;
  }

  const string& getCollection() const
  {
    return collection;
  }

  void setCollection(const string& collection)
  {
    ImageLogger::collection = collection;
  }

  int getMode() const
  {
    return mode;
  }

  void setMode(int mode)
  {
    ImageLogger::mode = mode;
  }

  mongo::DBClientConnection* getClientConnection() const
  {
    return clientConnection;
  }

  void setClientConnection(mongo::DBClientConnection* clientConnection)
  {
    ImageLogger::clientConnection = clientConnection;
  }
};

#endif  // IAI_IMAGE_LOGGING_IMAGE_LOGGER_H
