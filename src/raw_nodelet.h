//
// Created by tammo on 25.09.18.
//

#ifndef IAI_IMAGE_LOGGING_RAW_NODELET_H
#define IAI_IMAGE_LOGGING_RAW_NODELET_H
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "iai_resources.h"

namespace iai_nodelets
{
class RawNodelet : public nodelet::Nodelet
{
public:

  virtual void create(ros::NodeHandle nh, int mode, string topic, string collection,
                      mongo::DBClientConnection* client_connection);
  virtual void setParameters(string topic, string collection, string db_host, int mode);

  virtual void onInit();
  virtual void imageCallback(const sensor_msgs::ImageConstPtr& msg);

private:
  ros::NodeHandle nh_;
public:
    const ros::NodeHandle &getNh_() const;

private:
    int mode_;

public:
  void setMode(int mode_);

private:
  string topic_;
  string collection_;
  string db_host_;
  mongo::DBClientConnection* client_connection_;
};
}

#endif  // IAI_IMAGE_LOGGING_RAW_NODELET_H
