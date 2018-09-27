//
// Created by tammo on 25.09.18.
//

#ifndef IAI_IMAGE_LOGGING_IAI_NODELET_H
#define IAI_IMAGE_LOGGING_IAI_NODELET_H
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "iai_resources.h"

namespace iai_nodelets
{
class IAINodelet : public nodelet::Nodelet
{
public:
  virtual void setParameters(string topic, string collection, string db_host, int mode);
  virtual void create(ros::NodeHandle nh, int mode, string topic, string collection,
                      mongo::DBClientConnection* client_connection);
  virtual void createSubscriber(string topic);

  virtual void onInit();

public:
  const ros::NodeHandle& getNh_() const;

  void setNh_(const ros::NodeHandle& nh_);

  int getMode_() const;

  void setMode_(int mode_);

  const string& getTopic_() const;

  void setTopic_(const string& topic_);

  const string& getCollection_() const;

  void setCollection_(const string& collection_);

  const string& getDb_host_() const;

  void setDb_host_(const string& db_host_);

  mongo::DBClientConnection* getClient_connection_() const;

  void setClient_connection_(mongo::DBClientConnection* client_connection_);

  const Subscriber& getSub_() const;

  void setSub_(const Subscriber& sub_);

private:
  ros::NodeHandle nh_;
  int mode_;
  string topic_;
  string collection_;
  string db_host_;
  mongo::DBClientConnection* client_connection_;
  Subscriber sub_;
};
}

#endif  // IAI_IMAGE_LOGGING_IAI_NODELET_H
