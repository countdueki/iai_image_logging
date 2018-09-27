//
// Created by tammo on 25.09.18.
//

#ifndef IAI_IMAGE_LOGGING_THEORA_NODELET_H
#define IAI_IMAGE_LOGGING_THEORA_NODELET_H
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "iai_resources.h"
#include "iai_nodelet.h"

namespace iai_nodelets
{
class TheoraNodelet : public iai_nodelets::IAINodelet
{
public:
  virtual void createSubscriber(string topic);

  virtual void setParameters(string topic, string collection, string db_host, int mode);
  virtual void create(ros::NodeHandle nh, int mode, string topic, string collection,
                      mongo::DBClientConnection* client_connection);

  virtual void onInit();
  virtual void theoraCallback(const theora_image_transport::PacketConstPtr& msg);

private:
  ros::NodeHandle nh_;
  Subscriber sub_;

  int mode_;
  string topic_;
  string collection_;
  string db_host_;

  mongo::DBClientConnection* client_connection_;
};
}

#endif  // IAI_IMAGE_LOGGING_THEORA_NODELET_H
