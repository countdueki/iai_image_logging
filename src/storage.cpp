//
// Created by tammo on 13.09.18.
//
#include "storage.h"

#include <ros/ros.h>

#include <string>
#include <vector>
#include <map>
#include <mongo/client/dbclient.h>
#include <iai_image_logging_msgs/Update.h>
#include <iai_image_logging_msgs/Delete.h>
#include "storage_sub.h"

using std::string;
using std::vector;
using std::map;

using mongo::DBClientConnection;
typedef vector<StorageSub*> StorageSubVector;
typedef vector<StorageSubVector*> CameraVector;

class Storage
{
public:
  static Storage& Instance()
  {
    static std::shared_ptr<Storage> instance(new Storage);
    return *instance;
  }

private:
  Storage()
  {
    db_host_ = "localhost";
    // establish MongoDB connection
    string errmsg;
    if (!client_connection_->connect(db_host_, errmsg))
    {
      ROS_ERROR("Failed to connect to MongoDB: %s", errmsg.c_str());
    }
    mongo::client::initialize();

    cams_.push_back(new StorageSubVector);

    // start storage services
    update_config = nh_.advertiseService("storage/update", &Storage::update, this);
    add_service = nh_.advertiseService("storage/add", &Storage::addConfig, this);
    del_service = nh_.advertiseService("storage/del", &Storage::delConfig, this);
  }

  Storage(const Storage& old);
  const Storage& operator=(const Storage& old);
  // ~Storage(){}

  string db_host_;

  ros::NodeHandle nh_;
  ros::ServiceServer update_config;
  ros::ServiceServer add_service;
  ros::ServiceServer del_service;
  StorageSubVector subs_;
  CameraVector cams_;
  mongo::DBClientConnection* client_connection_ = new mongo::DBClientConnection(true);

public:
  /**
   * Service: Updates the requested topic of a 'camera' in a list of 'cameras' (a vector of a mmap<Subscriber,mode>)
   * If the requested topic is not found, it is added. If the camera isn't found, it is added
   * @param req Topic and database info to update
   * @param res success bool
   * @return true, if update was successfull, false else
   */
  bool update(iai_image_logging_msgs::UpdateRequest& req, iai_image_logging_msgs::UpdateResponse& res)
  {
    try {

      db_host_ = req.db_host;
      bool found_cam = false;
      int found_cam_no = 0;
      string cur_topic = "";
      string req_topic = "";

      if (req.cam_no > cams_.size())
      {
        ROS_ERROR_STREAM("Cameras have to be added in order. Camera " << req.cam_no << " cannot be added.");
        ROS_ERROR_STREAM("Please add camera with index " << cams_.size());
      }
      else
      {
        // assigns sub_ to required callback
        // createSubscriber(req.topic, req.mode);
        StorageSub storage_sub(req, res, client_connection_, "_ID_STRING");
        // iterate over camera list for existing entries
        for (int idx = 0; idx < cams_.size(); idx++)
        {
          if (idx == req.cam_no)
          {
            ROS_DEBUG_STREAM("Updating camera");
            found_cam = true;
            // iterate over list of Subscribers in camera
            for (int cam_idx = 0; cam_idx < cams_.at(idx)->size(); cam_idx++)
            {
              // neater topic names
              cur_topic = cams_.at(idx)->at(cam_idx)->getTopic() + getModeString(cams_.at(idx)->at(cam_idx)->getMode());
              req_topic = req.topic + getModeString(req.mode);

              // if we find topic, then update
              if (cur_topic.find(req_topic) != string::npos && found_cam)
              {
                ROS_DEBUG_STREAM("Updating topic");

                // shutdown and delete old subscriber
                cams_.at(idx)->at(cam_idx)->destroy();
                // TODO delete sub from camera list

                // add updated Subscriber
                cams_.at(req.cam_no)->push_back(&storage_sub);
                return true;
              }
            }
          }
        }

        if (!found_cam)
        {
          // add new cam
          ROS_WARN_STREAM("Adding new Camera");
          cams_.push_back(new StorageSubVector);
          return true;
        }
        else
        {
          // add new subscriber
          ROS_WARN_STREAM("Adding new Subscriber");
          cams_.at(req.cam_no)->push_back(&storage_sub);
          return true;
        }
      }
    } catch (ros::Exception e){
      ROS_ERROR_STREAM(e.what());
      return false;

    }

  }

  /**
   * Adding a Configiguration (i.e. camera + topic and parameters)
   * @param req requested configuration
   * @param res success bool
   * @return true if success, false else
   */
  bool addConfig(iai_image_logging_msgs::UpdateRequest& req, iai_image_logging_msgs::UpdateResponse& res)
  {
    if (cams_.size() < req.cam_no)
    {
      ROS_ERROR_STREAM("Camera does not exist");
    }
    else
    {
      // TODO update kinect topic parameters
      update(req, res);
      res.success = true;
      return true;
    }
  }

  /**
  * Deleting a Configiguration (i.e. camera + topic)
  * @param req requested configuration
  * @param res success bool
  * @return true if success, false else
  */
  bool delConfig(iai_image_logging_msgs::DeleteRequest& req, iai_image_logging_msgs::DeleteResponse& res)
  {
    if (cams_.size() < req.cam_no)
     {
       ROS_ERROR_STREAM("Camera does not exist");
     }
     else
     {
       for (int idx = 0; idx < cams_.at(req.cam_no)->size() ; idx++)
       {

         string cur_topic = cams_.at(req.cam_no)->at(idx)->getTopic() + getModeString(cams_.at(req.cam_no)->at(idx)->getMode());
         string req_topic = req.topic + getModeString(req.mode);

         if (cur_topic.find(req_topic) != string::npos) {

           // TODO: Check for integrity of subscriber list in camera
           ROS_WARN_STREAM("Shutting down sub");
           cams_.at(req.cam_no)->at(idx)->destroy();
           ROS_WARN_STREAM("Found fitting topic and mode, erasing...");
           //TODO: erase subscriber
         }

         if (cams_.at(req.cam_no)->empty())

           // TODO delete camera
           ROS_WARN_STREAM("Camera order may be broken. be careful!");

       }
       res.success = true;
       return true;
     }
  }

  /**
   * simple helper function. Returns string suffix requested by mode
   * @param mode RAW, COMPRESSED, etc.
   * @return suffix string by mode
   */
  string getModeString(int mode)
  {
    switch (mode)
    {
      case (RAW):
        return "";
      case (COMPRESSED):
        return "/compressed";
      case (THEORA):
        return "/theora";
      case (DEPTH):
        return "";
      case (COMPRESSED_DEPTH):
        return "/compressed";
      default:
        break;
    }
  }

  /**
  * Get the storage ros::Nodehandle
  *@return Nodehandle
  */
  const ros::NodeHandle& getNh() const
  {
    return nh_;
  }

  /**
   * initialize storage node. Now needed for startup not to break.
   */
  void init()
  {
    try {

      ROS_WARN_STREAM("initialize...");
      StorageSub sub(client_connection_);
      StorageSubVector sub_vect;
      sub_vect.push_back(&sub);
      cams_.push_back(&sub_vect);
      ROS_WARN_STREAM("initilization done");
    } catch (ros::Exception e){
      ROS_ERROR_STREAM(e.what());
    }

  }
};
/**
 * Start storage node
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "storage");

  // create storage and initialize
  Storage* storage = &Storage::Instance();
  ROS_ERROR_STREAM("ERROR NOT HERE");

  storage->init();
  ros::spin();
  return 0;
}