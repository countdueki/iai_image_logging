//
// Created by tammo on 13.09.18.
//

#ifndef IAI_IMAGE_LOGGING_STORAGE_H
#define IAI_IMAGE_LOGGING_STORAGE_H

#include <ros/ros.h>
#include <string>
#include <mongo/client/dbclient.h>
#include <theora_image_transport/Packet.h>
#include <sensor_msgs/CompressedImage.h>
#include <iai_image_logging_msgs/Update.h>

// opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <vector>

using std::string;
enum
{
  RAW,
  COMPRESSED,
  THEORA
};
class Storage
{
public:
  Storage()
  {
    clientConnection = new mongo::DBClientConnection(true);
    dbHost = "localhost";
    collection = "db.standard";
    topic = "camera/rgb/image_raw";
    mode = 1;
  }

private:
  string topic;
  string dbHost;
  string collection;
  int mode;

public:
  int getMode() const
  {
    return mode;
  }

  void setMode(int mode)
  {
    Storage::mode = mode;
  }

private:
  mongo::DBClientConnection* clientConnection;

public:
  const string& getTopic() const
  {
    return topic;
  }

  void setTopic(const string& topic)
  {
    Storage::topic = topic;
  }

  const string& getDbHost() const
  {
    return dbHost;
  }

  void setDbHost(const string& dbHost)
  {
    Storage::dbHost = dbHost;
  }

  const string& getCollection() const
  {
    return collection;
  }

  void setCollection(const string& collection)
  {
    Storage::collection = collection;
  }

  mongo::DBClientConnection* getClientConnection() const
  {
    return clientConnection;
  }

  void setClientConnection(mongo::DBClientConnection* clientConnection)
  {
    Storage::clientConnection = clientConnection;
  }
};

#endif  // IAI_IMAGE_LOGGING_STORAGE_H
