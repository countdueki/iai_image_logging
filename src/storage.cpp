//
// Created by tammo on 13.09.18.
//
#include "storage.h"
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

    collection_.emplace_back("db.standard");
    topic_ = "camera/rgb/image_raw";
    mode_ = 1;

    // establish MongoDB connection
    string errmsg;
    if (!client_connection_->connect(db_host_, errmsg))
    {
      ROS_ERROR("Failed to connect to MongoDB: %s", errmsg.c_str());
    }
    mongo::client::initialize();

    // start storage services
    update_config = nh_.advertiseService("storage/update", &Storage::update, this);
    add_service = nh_.advertiseService("storage/add", &Storage::addConfig, this);
    del_service = nh_.advertiseService("storage/del", &Storage::delConfig, this);
  }

  Storage(const Storage& old);
  const Storage& operator=(const Storage& old);
  // ~Storage(){}

  string topic_;
  string db_host_;
  vector<string> collection_, collection_compressed_, collection_theora_;
  int mode_;

  ros::NodeHandle nh_, nh_r_, nh_c_, nh_t_;
  ros::ServiceServer update_config;
  ros::ServiceServer add_service;
  ros::ServiceServer del_service;
  StorageSubVector subs_;
  map<int, StorageSubVector> cams_;
  mongo::DBClientConnection* client_connection_ = new mongo::DBClientConnection(true);

  ros::Spinner* async_spinner_r;

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
    topic_ = req.topic;
    db_host_ = req.db_host;
    mode_ = req.mode;

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
      /* // set collection name
       switch (req.mode)
       {
         case (RAW):
           if (collection_.empty() || collection_.size() <= req.cam_no)
           {
             collection_.push_back(req.collection);
           }
           else
           {
             collection_.at(req.cam_no) = req.collection;
           }
           break;
         case (COMPRESSED):
           if (collection_compressed_.empty() || collection_compressed_.size() <= req.cam_no)
           {
             collection_compressed_.push_back(req.collection);
           }
           else
           {
             collection_compressed_.at(req.cam_no) = req.collection;
           }
           break;
         case (THEORA):
           if (collection_theora_.empty() || collection_theora_.size() <= req.cam_no)
           {
             collection_theora_.push_back(req.collection);
           }
           else
           {
             collection_theora_.at(req.cam_no) = req.collection;
           }
           break;
         case (DEPTH):
           if (collection_.empty() || collection_.size() <= req.cam_no)
           {
             collection_.push_back(req.collection);
           }
           else
           {
             collection_.at(req.cam_no) = req.collection;
           }
           break;
         case (COMPRESSED_DEPTH):
           if (collection_compressed_.empty() || collection_compressed_.size() <= req.cam_no)
           {
             collection_compressed_.push_back(req.collection);
           }
           else
           {
             collection_compressed_.at(req.cam_no) = req.collection;
           }
           break;
         default:
           break;
       }
 */
      // assigns sub_ to required callback
      //createSubscriber(req.topic, req.mode);
      StorageSub storage_sub(req,res, client_connection_, "_ID_STRING");

      // iterate over camera list for existing entries
      for (int idx = 0; idx < cams_.size(); idx++)
      {
        if (idx == req.cam_no)
        {
          // iterate over list of Subscribers in camera
          for (auto it = cams_.begin(); it != cams_.end(); it++)
          {
            if (it->first == req.cam_no)
            {
              ROS_DEBUG_STREAM("Updating camera");
              found_cam = true;

              // iterate over subscriber list of camera
              for (auto sub_it = it->second.begin(); sub_it != it->second.end(); sub_it++)
              {
                // neater topic names
                cur_topic = sub_it->getTopic() + getModeString(sub_it->getMode());
                req_topic = req.topic + getModeString(req.mode);

                if (cur_topic.find(req_topic) != string::npos && found_cam)
                {
                  ROS_DEBUG_STREAM("Updating topic");

                  // shutdown and delete old subscriber
                  auto pos = cams_.at(req.cam_no).begin();
                  while (pos->getMode() != req.mode && pos != cams_.at(req.cam_no).end())
                    ++pos;
                  pos->destroy();
                  cams_.at(req.cam_no).erase(pos);

                  // add updated Subscriber
                  cams_.at(req.cam_no).push_back(storage_sub);
                  return true;
                }
              }
            }
          }
        }
      }

      if (!found_cam)
      {
        // add new cam
        ROS_WARN_STREAM("Adding new Camera");
        cams_.insert(std::make_pair(req.cam_no,storage_sub));
        return true;
      }
      else
      {
        // add new subscriber
        ROS_WARN_STREAM("Adding new Subscriber");
        cams_.at(req.cam_no).push_back(storage_sub);
        return true;
      }
    }

    return false;
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
      for (auto it = cams_.at(req.cam_no).begin(); it != cams_.at(req.cam_no).end();it++)
      {

        string cur_topic = it->getTopic() + getModeString(it->getMode());
        string req_topic = req.topic + getModeString(req.mode);

        if (cur_topic.find(req_topic) != string::npos) {

          // TODO: Check for integrity of subscriber list in camera
          ROS_WARN_STREAM("Shutting down sub");
          it->destroy();
          ROS_WARN_STREAM("Found fitting topic and mode, erasing...");
          cams_.at(req.cam_no).erase(it);
        }

        if (cams_.at(req.cam_no).empty())

          // TODO delete camera
          ROS_WARN_STREAM("Camera order may be broken. be careful!");

      }
      res.success = true;
      return true;
    }
  }

  /**
   * Helper function to create Subscriber by topic and mode
   * @param topic images to subscribe to
   * @param mode compression method / raw method
   */
  void createSubscriber(string topic, int mode)
  {
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
        break;
      case (COMPRESSED):
        return "/compressed";
        break;
      case (THEORA):
        return "/theora";
        break;
      case (DEPTH):
        return "";
        break;
      case (COMPRESSED_DEPTH):
        return "/compressed";
        break;
      default:
        break;
    }
  }

  /**
 *  Get the storage ros::Nodehandle
 * @return Nodehandle
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
    ROS_WARN_STREAM("initialize...");
    StorageSub sub(client_connection_);
    cams_.at(0).push_back(sub);
    ROS_WARN_STREAM("initilization done");
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
  ros::spin();
  return 0;
}