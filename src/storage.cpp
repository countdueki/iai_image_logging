//
// Created by tammo on 13.09.18.
//
#include "storage.h"
#include <iai_image_logging_msgs/MainConfig.h>
#include <dynamic_reconfigure/client.h>

using std::string;
using std::vector;
using std::map;

using mongo::DBClientConnection;
typedef vector<StorageSub*> StorageSubVector;

class Storage
{
public:
  static Storage& Instance()
  {
    static boost::shared_ptr<Storage> instance(new Storage);
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

    cams_size_ = 0;
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
  DBClientConnection* client_connection_ = new DBClientConnection(true);
  int cams_size_;

public:
  void updateCamera(iai_image_logging_msgs::UpdateRequest& req, iai_image_logging_msgs::UpdateResponse& res)
  {
    dynamic_reconfigure::ReconfigureRequest req_;
    dynamic_reconfigure::ReconfigureResponse res_;
    dynamic_reconfigure::StrParameter format;
    format.name = "format";
    format.value = req.format;
    req_.config.strs.push_back(format);
    ros::service::call("image_logger/set_parameters", req_, res_);  // WORKS!
    // ros::service::call("image_logger/updateCamera",req,res);
  }

  /**
         * Service: Updates the requested topic of a 'camera' in a list of 'cameras' (a vector of a
   * mmap<Subscriber,mode>)
         * If the requested topic is not found, it is added. If the camera isn't found, it is added
         * @param req Topic and database info to update
         * @param res success bool
         * @return true, if update was successfull, false else
         */
  bool update(iai_image_logging_msgs::UpdateRequest& req, iai_image_logging_msgs::UpdateResponse& res)
  {
    try
    {
      db_host_ = req.db_host;
      string cur_topic = "";
      string req_topic = "";

      if (req.cam_no > cams_size_)
      {
        ROS_ERROR_STREAM("Cameras have to be added in order. Camera " << req.cam_no << " cannot be added.");
        ROS_ERROR_STREAM("Please add camera with index " << cams_size_);
      }
      else
      {
        // assigns sub_ to required callback
        // createSubscriber(req.topic, req.mode);
        StorageSub* storage_sub = new StorageSub(*client_connection_, req);

        // iterate over camera list for existing entries
        for (auto it = subs_.begin(); it != subs_.end(); ++it)
        {
          if ((*it)->getCam() == req.cam_no)
          {
            ROS_DEBUG_STREAM("Found Camera");

            // neater topic names
            cur_topic = (*it)->getTopic();
            req_topic = req.topic + getModeString(req.mode);
            ROS_DEBUG_STREAM("current_topic: " << cur_topic);
            ROS_DEBUG_STREAM("requested topic: " << req_topic);

            // if we find topic, then update
            if (cur_topic.find(req_topic) != string::npos)
            {
              ROS_DEBUG_STREAM("Found topic. delete old...");

              // shutdown and delete old subscriber
              (*it)->destroy();
              subs_.erase(it);
              ROS_DEBUG_STREAM("...update camera topic...");
              // updateCamera(req,res);
              ROS_DEBUG_STREAM("...and add new");

              // add updated Subscriber
              subs_.push_back(storage_sub);
              ROS_INFO_STREAM("Updated Subscriber");
              return true;
            }
          }
        }
        // add new subscriber
        ROS_DEBUG_STREAM("Adding new Subscriber");
        // updateCamera(req,res);
        subs_.push_back(storage_sub);
        cams_size_++;
        ROS_DEBUG_STREAM("size of subs_: " << subs_.size());
        ROS_INFO_STREAM("Added Subscriber");
        return true;
      }
    }
    catch (std::bad_alloc& ba)
    {
      ROS_ERROR_STREAM("bad_alloc caught: " << ba.what());
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
    if (cams_size_ < req.cam_no)
    {
      ROS_ERROR_STREAM("Camera does not exist");
    }
    else
    {
      updateCamera(req, res);
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
    /* if (cams_size_ < req.cam_no)
      {
        ROS_ERROR_STREAM("Camera does not exist");
      }
      else
      {
        for (int idx = 0; idx < cams_.at(req.cam_no)->size() ; idx++)
        {

          string cur_topic = cams_.at(req.cam_no)->at(idx)->getTopic() +
      getModeString(cams_.at(req.cam_no)->at(idx)->getMode());
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
      }*/
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
        return "/compressedDepth";
      default:
        break;
    }
  }

  /**
   * initialize storage node. Now needed for startup not to break.
   */
  void init()
  {
    try
    {
      ROS_WARN_STREAM("initialize...");
      StorageSub* sub = new StorageSub(*client_connection_);
      subs_.push_back(sub);
      cams_size_++;
      ROS_WARN_STREAM("initilization done");
      ROS_ERROR_STREAM("topic initialized with: " << sub->getTopic());
      ROS_ERROR_STREAM("size: " << subs_.size());
    }
    catch (std::bad_alloc& ba)
    {
      ROS_ERROR_STREAM("bad_alloc caught: " << ba.what());
    }
  }

  const StorageSubVector& getSubs() const
  {
    return subs_;
  }

  const NodeHandle& getNodeHandle() const
  {
    return nh_;
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
  storage->init();
  // ros::Rate r(20.0);
  while (storage->getNodeHandle().ok())
  {
    for (auto s : storage->getSubs())
    {
      s->start();
      ROS_DEBUG_STREAM("Topic: " << s->getTopic());
    }
    ROS_DEBUG_STREAM("storage spinned");
    ros::spinOnce();
    // r.sleep();
  }
  return 0;
}