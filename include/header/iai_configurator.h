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

/**
 * Constructor (singleton) handles the connection to the mongoDB database and set up for the services to
 * insert and remove IAISubscribers as well as change their behavior
 */
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
  /**
   * Service that updates or inserts a new Subscriber in @property subs_. It is usually called by @function insert. A
   * new IAISubscriber pointer is created and then compared to the existing subscribers in the list.
   * If its ID (@property iai_id_) exists, the current subscriber will be destroyed and the new one takes its place.
   * If it does not exist, a new IAISubscriber will be added.
   *
   * @param req Information for the new subscriber
   * @param res success true if successful, false else
   * @return true, if update was successful, false else
   */
  bool update(iai_image_logging_msgs::UpdateRequest& req, iai_image_logging_msgs::UpdateResponse& res);

  /**
   * Service function, that inserts or updates a IAISubscriber. Calls @function insertionConfigurator to fill the more
   * complex UpdateRequest with information from the simpler @arg InsertRequest. When the configuration was succesful,
   * @function update is called.
   * @param req InsertRequest with information about topic and behavior
   * @param res success true if successful, false else
   * @return true, if update was succesfuls, false else
   */
  bool insert(iai_image_logging_msgs::InsertRequest& req, iai_image_logging_msgs::InsertResponse& res);

  /**
    * Service, that removes an IAISubscriber by ID (iai_id)
    * @param req ID of subscriber, that should be removed
    * @param res success true, if IAISubscriber was removed, false else
    * @return
    */
  bool remove(iai_image_logging_msgs::RemoveRequest& req, iai_image_logging_msgs::RemoveResponse& res);

  /**
   * Service, that changes the behavior of an IAISubscriber by ID (iai_id)
   * @param req ID of subscriber, that should be changed with information about behavioral changes
   * @param res success true, if IAISubscriber was changed, false else
   * @return
   */
  bool behave(iai_image_logging_msgs::BehaveRequest& req, iai_image_logging_msgs::BehaveResponse& res);

  /**
   * Converts or fills an UpdateRequest from an InsertRequest
   * @param req Information about the subscriber, that is to be created
   * @param res success true, if filling was successful, false else
   * @param ureq UpdateRequest that is to be filled
   * @param ures success true, if update was successful, false else
   */
  void insertionConfigurator(iai_image_logging_msgs::InsertRequest& req, iai_image_logging_msgs::InsertResponse& res,
                             iai_image_logging_msgs::UpdateRequest& ureq, iai_image_logging_msgs::UpdateResponse& ures);

  /**
   * Sends parameters for the IAISubscriber created (by @function update) to @class IAIUpdater which in turn handles
   * the update of the camera topics requested.
   * @param req UpdateRequest with topic and parameter information
   * @param res success, if sending was succesful, false else
   */
  void updateCamera(iai_image_logging_msgs::UpdateRequest& req, iai_image_logging_msgs::UpdateResponse& res);

  /**
   * Returns the corresponding integer to the requested string for mode evaluation
   * @param mode information about topic (raw, compressed, theora or compressedDepth)
   * @return mode integer from enum that corresponds to requested string
   */
  int getNumberFromModeString(string mode);

  /**
   * Returns the Nodehandle of this Configurator
   * @return ros::NodeHandle of this Configurator
   */
  const NodeHandle& getNodeHandle() const;

  /**
   * Returns the list of IAISubscribers of this node, being a vector of IAISubscriber pointers
   * @return list of IAISubscribers
   */
  const StorageSubVector& getSubscribers() const;
};

#endif  // IAI_IMAGE_LOGGING_STORAGE_H
