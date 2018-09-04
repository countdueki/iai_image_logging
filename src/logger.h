//
// Created by tammo on 26.07.18.
//

#ifndef IAI_IMAGE_LOGGING_LOGGER_H
#define IAI_IMAGE_LOGGING_LOGGER_H

#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <iai_image_logging_msgs/DefaultConfig.h>
#include <iai_image_logging_msgs/Log.h>
#include "image_logger_types.h"


#include <mongo/client/dbclient.h>
#include <mongodb_store/util.h>
#include <mongodb_store/message_store.h>

using std::vector;

using mongo::client::initialize;
using mongo::BSONObjBuilder;
using mongo::Date_t;
using mongo::DBClientConnection;
using mongo::BinDataGeneral;
using mongodb_store::add_meta_for_msg;
using std::string;


class Logger
{
public:
    Logger(){};
    void log(sensor_msgs::CompressedImageConstPtr msg, std::vector<defcon_ptr> cfg_multi, DBClientConnection *mongodb_conn);

};

#endif  // IAI_IMAGE_LOGGING_LOGGER_H
