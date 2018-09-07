/**
 * Copyright 2018 by Tammo Wuebbena, University of Bremen
 * Author: Tammo Wuebbena <ta_wu@tzi.de>
 */

#ifndef IAI_IMAGE_LOGGING_IMAGE_LOGGER_H
#define IAI_IMAGE_LOGGING_IMAGE_LOGGER_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <dynamic_reconfigure/server.h>

#include <iai_image_logging_msgs/Configuration.h>
#include <iai_image_logging_msgs/ConfigurationYAML.h>
#include <iai_image_logging_msgs/CompressedConfig.h>
#include <iai_image_logging_msgs/Process.h>
#include <iostream>
#include <boost/smart_ptr/shared_ptr.hpp>

#include <mongo/client/dbclient.h>
#include <mongodb_store/util.h>
#include <mongodb_store/message_store.h>

#include <nodelet/nodelet.h>

using std::string;
using mongo::client::initialize;
using mongo::BSONObjBuilder;
using mongo::Date_t;
using mongo::DBClientConnection;
using mongo::BinDataGeneral;
using mongodb_store::add_meta_for_msg;

typedef iai_image_logging_msgs::CompressedConfig CompConf;

namespace image_logger
{
class ImageLogger : public nodelet::Nodelet
{
private:
  // General Parameters
  string topic_;
  string format_;
  int jpeg_quality_;

  int png_level_;
  string db_host_;
  string collection_;

  // Depth parameters
  // double_t depth_max_;
  // double_t depth_quantization_;

  // Theora parameters
  int keyframe_frequency_;
  // int optimize_for_;
  int t_quality_;
  uint32_t target_bitrate_;

public:
  const string& getTopic() const;

  void setTopic(const string& topic);

  const string& getDbHost() const;

  void setDbHost(const string& db_host_);

  const string& getCollection() const;

  void setCollection(const string& collection_);

  const string& getFormat() const;

  void setFormat(const string& format_);

  int getJpegQuality() const;

  void setJpegQuality(int jpeg_quality);

  int getPngLevel() const;

  void setPngLevel(int png_level);

public:
  ImageLogger(string topic) : topic_(topic)
  {
    topic_ = topic;
  }

  ImageLogger()
  {
    // TODO enter defaults
    topic_ = "/camera/rgb/image_raw";
    db_host_ = "localhost";
    collection_ = "db.image_color_compressed";
  }

  ~ImageLogger();


  virtual void onInit();


};

const string& ImageLogger::getTopic() const
{
  return topic_;
}

void ImageLogger::setTopic(const string& topic)
{
  ImageLogger::topic_ = topic;
}

const string& ImageLogger::getDbHost() const
{
  return db_host_;
}

void ImageLogger::setDbHost(const string& db_host)
{
  ImageLogger::db_host_ = db_host;
}

const string& ImageLogger::getCollection() const
{
  return collection_;
}

void ImageLogger::setCollection(const string& collection)
{
  ImageLogger::collection_ = collection;
}

const string& ImageLogger::getFormat() const
{
  return format_;
}

void ImageLogger::setFormat(const string& format)
{
  ImageLogger::format_ = format;
}

int ImageLogger::getJpegQuality() const
{
  return jpeg_quality_;
}

void ImageLogger::setJpegQuality(int jpeg_quality)
{
  ImageLogger::jpeg_quality_ = jpeg_quality;
}

int ImageLogger::getPngLevel() const
{
  return png_level_;
}

void ImageLogger::setPngLevel(int png_level)
{
  ImageLogger::png_level_ = png_level;
}
}
#endif  // IAI_IMAGE_LOGGING_IMAGE_LOGGER_H
