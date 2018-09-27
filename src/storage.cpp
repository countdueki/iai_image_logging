//
// Created by tammo on 13.09.18.
//

#include "storage.h"
#include "iai_sub.h"
#include "iai_raw_sub.h"
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
    collection_ = "db.standard";
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

    raw_nodelet_.create(topic_,collection_,db_host_,mode_,client_connection_);
  }

  Storage(const Storage& old);
  const Storage& operator=(const Storage& old);
  // ~Storage(){}

  string topic_;
  string db_host_;
  string collection_;
  int mode_;

  ros::NodeHandle nh_;
  ros::ServiceServer update_config;
  ros::ServiceServer add_service;
  ros::ServiceServer del_service;
  ros::Subscriber sub_;
  vector<IAISub> nodelet_list_;
  vector<vector<IAISub>> camera_list_;
  mongo::DBClientConnection* client_connection_ = new mongo::DBClientConnection(true);

  RawSub raw_nodelet_;

public:
  void updateNodelet(iai_image_logging_msgs::UpdateRequest req)
  {
    collection_ = req.collection;
    topic_ = req.topic;
    db_host_ = req.db_host;
    mode_ = req.mode;

    raw_nodelet_.create(topic_,collection_,db_host_,mode_,client_connection_);



  }
  /**
   * Service: Updates the requested topic of a 'camera' in a list of 'cameras' (a vector of a mmap<Subscriber,mode>)
   * If the requested topic is not found, it is added. If the camera isn't found, it is added
   * @param req Topic and database info to update
   * @param res success bool
   * @return true, if update was successfull, false else
   */
  bool update(iai_image_logging_msgs::UpdateRequest& req, iai_image_logging_msgs::UpdateResponse& res)
  {
    ROS_WARN_STREAM("EXECUTING UPDATE");
    updateNodelet(req);

    bool found_cam = false;
    int found_cam_no = 0;
    string current_topic = "";
    string requested_topic = "";
      std::vector<RawSub> nodelets;
      nodelets.push_back(raw_nodelet_);
    if (req.cam_no > camera_list_.size())
    {
      ROS_ERROR_STREAM("Cameras have to be added in order. Camera " << req.cam_no << " cannot be added.");
      ROS_ERROR_STREAM("Please add camera with index " << camera_list_.size());
    }
    else
    {
      // assigns sub_ to required callback
      createSubscriber(req.topic, req.mode,req,res);

      // iterate over camera list for existing entries
      for (int idx = 0; idx < camera_list_.size(); idx++)
      {
        if (idx == req.cam_no)
        {
          ROS_DEBUG_STREAM("Updating camera");

          found_cam = true;

          // iterate over list of Subscribers in camera
          for (auto it = camera_list_.at(idx).begin(); it != camera_list_.at(idx).end(); it++)
          {
            current_topic = it->getTopic_() + getModeString(it->getMode_());
            requested_topic = req.topic + getModeString(req.mode);
            if (current_topic.find(requested_topic) != string::npos && found_cam)
            {
              ROS_DEBUG_STREAM("Updating topic");

              // shutdown and delete old subscriber
              auto pos = camera_list_.at(idx).begin();
              while (pos->getMode_() != req.mode && pos != camera_list_.at(idx).end())
                ++pos;
              ros::Subscriber temp_sub = pos->getSub_();
              temp_sub.shutdown();
              camera_list_.at(idx).erase(pos);
              // add updated Subscriber

                  camera_list_.at(idx).push_back(raw_nodelet_);

              return true;
            }
          }
        }
      }

      if (!found_cam)
      {
        // add new cam
        ROS_WARN_STREAM("Adding new Camera");
        std::vector<RawSub> nodelet_vector;

                  nodelet_vector.push_back(raw_nodelet_);
        return true;
      }
      else
      {
        // add new subscriber
        ROS_WARN_STREAM("Adding new Subscriber");

                               camera_list_.at(req.cam_no).push_back(raw_nodelet_);

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
    if (camera_list_.size() < req.cam_no)
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
  bool delConfig(iai_image_logging_msgs::UpdateRequest& req, iai_image_logging_msgs::UpdateResponse& res)
  {
    if (camera_list_.size() < req.cam_no)
    {
      ROS_ERROR_STREAM("Camera does not exist");
    }
    else
    {
      for (auto it = camera_list_.at(req.cam_no).begin(); it != camera_list_.at(req.cam_no).end(); it++)
      {
        if (it->getTopic_().find(req.topic) != string::npos && it->getMode_() == req.mode)
        {
          ROS_WARN_STREAM("Shutting down sub");
          Subscriber temp_sub = it->getSub_();
          temp_sub.shutdown();
          ROS_WARN_STREAM("Found fitting topic and mode, erasing...");
          camera_list_.at(req.cam_no).erase(it);
        }
        if (camera_list_.at(req.cam_no).empty())
        {
          // TODO delete camera
          ROS_ERROR_STREAM("Camera has to be deleted!");
        }
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
  void createSubscriber(string topic, int mode,iai_image_logging_msgs::UpdateRequest& req, iai_image_logging_msgs::UpdateResponse& res)
  {

       raw_nodelet_.createSubscriber(topic);
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
    raw_nodelet_.createSubscriber(topic_);
   nodelet_list_.push_back(raw_nodelet_);
    camera_list_.push_back(nodelet_list_);
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

  ros::Rate hz_rate(1.0);
  while (storage->getNh().ok())
  {
    ros::spinOnce();

    hz_rate.sleep();
  }

  return 0;
}