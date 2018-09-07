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
#include <compressed_image_transport/CompressedSubscriberConfig.h>
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
  Logger();
  ~Logger();
};
#endif  // IAI_IMAGE_LOGGING_LOGGER_H
