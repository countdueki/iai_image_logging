//
// Created by tammo on 13.09.18.
//

#ifndef IAI_IMAGE_LOGGING_STORAGE_H
#define IAI_IMAGE_LOGGING_STORAGE_H

#include <ros/ros.h>
#include <ros/console.h>

#include <iostream>
#include <mongo/client/dbclient.h>
#include <iai_image_logging_msgs/Update.h>
#include <iai_image_logging_msgs/Insert.h>
#include <iai_image_logging_msgs/Remove.h>
#include <iai_image_logging_msgs/Behave.h>
#include "../../src/iai_subscriber.cpp"

#include <iai_image_logging_msgs/MainConfig.h>
#include <dynamic_reconfigure/client.h>

using std::string;
using std::vector;

using mongo::DBClientConnection;
typedef vector<IAISubscriber*> StorageSubVector;
using dynamic_reconfigure::ReconfigureRequest;
using dynamic_reconfigure::ReconfigureResponse;

typedef dynamic_reconfigure::StrParameter StrParam;
typedef dynamic_reconfigure::IntParameter IntParam;
typedef dynamic_reconfigure::DoubleParameter DoubleParam;
class IAIConfigurator
{
public:
  static IAIConfigurator& Instance()
  {
    static boost::shared_ptr<IAIConfigurator> instance(new IAIConfigurator);
    return *instance;
  }

private:
  IAIConfigurator()
  {
    string node_name = "iai_configurator";
    db_host_ = "localhost";
    // establish MongoDB connection
    string errmsg;
    if (!client_connection_->connect(db_host_, errmsg))
    {
      ROS_ERROR("Failed to connect to MongoDB: %s", errmsg.c_str());
    }
    mongo::client::initialize();

    // start storage services
    update_config = nh_.advertiseService(node_name + "/update", &IAIConfigurator::update, this);
    add_service = nh_.advertiseService(node_name + "/insert", &IAIConfigurator::insert, this);
    del_service = nh_.advertiseService(node_name + "/remove", &IAIConfigurator::remove, this);
    behave_service = nh_.advertiseService(node_name + "/behave", &IAIConfigurator::behave, this);
  }

  IAIConfigurator(const IAIConfigurator& old);

  const IAIConfigurator& operator=(const IAIConfigurator& old);
  // ~IAIConfigurator(){}

  string db_host_;
  ros::NodeHandle nh_;

private:
  ros::ServiceServer update_config, add_service, del_service, behave_service;
  StorageSubVector subs_;
  DBClientConnection* client_connection_ = new DBClientConnection(true);

public:
  void updateCamera(iai_image_logging_msgs::UpdateRequest& req, iai_image_logging_msgs::UpdateResponse& res);
  bool update(iai_image_logging_msgs::UpdateRequest& req, iai_image_logging_msgs::UpdateResponse& res);
  bool insert(iai_image_logging_msgs::InsertRequest &req, iai_image_logging_msgs::InsertResponse &res);
  void insertionConfigurator(iai_image_logging_msgs::InsertRequest &req, iai_image_logging_msgs::InsertResponse &res,
            iai_image_logging_msgs::UpdateRequest& ureq, iai_image_logging_msgs::UpdateResponse& ures);
  bool remove(iai_image_logging_msgs::RemoveRequest &req, iai_image_logging_msgs::RemoveResponse &res);
  bool behave(iai_image_logging_msgs::BehaveRequest& req, iai_image_logging_msgs::BehaveResponse& res);

  const NodeHandle& getNodeHandle() const;

  const StorageSubVector& getSubscribers() const;
};

#endif  // IAI_IMAGE_LOGGING_STORAGE_H
