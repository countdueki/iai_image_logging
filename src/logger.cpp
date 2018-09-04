//
// Created by tammo on 26.07.18.
//

#include "logger.h"


std::vector<defcon_ptr> cfg_global;
DBClientConnection *mongodb_conn;
void Logger::log(sensor_msgs::CompressedImageConstPtr msg, std::vector<defcon_ptr> cfg_multi, DBClientConnection *mongodb_conn) {

    // Check connection to MongoDB
    string err = string("");
    string db_host = cfg_global[0]->db_host;
    mongodb_conn = new DBClientConnection(/* auto reconnect*/ true);
    if (!mongodb_conn->connect(cfg_global[0]->db_host, err))
    {
        ROS_ERROR("Failed to connect to MongoDB: %s", err.c_str());
    }

  initialize();
  BSONObjBuilder document;

    cfg_global = cfg_multi;
  ROS_DEBUG_STREAM("Saving Image at sec " << msg->header.stamp.sec << " with type " << msg->format);

  Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
  document.append("header", BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
  document.append("format", msg->format);
  document.appendBinData("data", msg->data.size(), BinDataGeneral, (&msg->data[0]));

  add_meta_for_msg<sensor_msgs::CompressedImage>(msg, document);
  mongodb_conn->insert(cfg_multi[0]->collection, document.obj());
}
