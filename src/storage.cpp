//
// Created by tammo on 13.09.18.
//

#include "storage.h"
int g_count = 0;
int g_count_comp = 0;
ros::CallbackQueue cb_queue_r_, cb_queue_c_, cb_queue_t_;

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

    nh_r_.setCallbackQueue(&cb_queue_r_);
    nh_c_.setCallbackQueue(&cb_queue_c_);
    nh_t_.setCallbackQueue(&cb_queue_t_);
  }

  Storage(const Storage& old);
  const Storage& operator=(const Storage& old);
  // ~Storage(){}

  string topic_;
  string db_host_;
  vector<string> collection_, collection_compressed_, collection_theora_;
  int mode_;

  ros::NodeHandle nh_,nh_r_,nh_c_,nh_t_;
  ros::ServiceServer update_config;
  ros::ServiceServer add_service;
  ros::ServiceServer del_service;
  ros::Subscriber sub_;
  ModeSubscriber sub_list_;
  ros::SubscribeOptions ops_r_, ops_c_;
  vector<ModeSubscriber> camera_list_;
  mongo::DBClientConnection* client_connection_ = new mongo::DBClientConnection(true);

    ros::Spinner* async_spinner_r;
public:

    void saveImage(const sensor_msgs::ImageConstPtr& msg)
    {
        mongo::BSONObjBuilder document;

        ROS_DEBUG_STREAM("raw collection size " << collection_.size());
        for (int i = 0; i < collection_.size(); i++)
        {
            ROS_DEBUG_STREAM("msg format: " << msg->encoding);
            ROS_DEBUG_STREAM("storage TOPIC: " << topic_);
            ROS_DEBUG_STREAM("storage COLLECTION: " << collection_.at(i));
            mongo::Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
            document.append("header",
                            BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
            document.append("encoding", msg->encoding);
            document.append("width", msg->width);
            document.append("height", msg->height);
            document.append("is_bigendian", msg->is_bigendian);
            document.append("step", msg->step);

            document.appendBinData("data", msg->data.size(), mongo::BinDataGeneral, &msg->data[0]);
            std::string type(ros::message_traits::DataType<sensor_msgs::Image>::value());
            document.append("type", type);
            document.append("size", (int)msg->data.size());
            if (mode_ == RAW)
            {
                document.append("mode_", "raw");
            }
            else if (mode_ == DEPTH)
            {
                document.append("mode_", "depth");
            }
            client_connection_->insert(collection_.at(i), document.obj());
        }
    }
  /**
   * image callback to save raw and depth images
   * @param msg raw and depth images
   */
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
      ROS_WARN_STREAM("we welcome our G #" << g_count);
      saveImage(msg);
      g_count++;
      sleep(1);



  }

  void saveCompressedImage(const sensor_msgs::CompressedImageConstPtr& msg)
  {
      ROS_DEBUG_STREAM("compressed collection size " << collection_compressed_.size());

      for (int i = 0; i < collection_compressed_.size(); i++)
      {
          ROS_DEBUG_STREAM("FORMAT: " << msg->format);
          ROS_DEBUG_STREAM("storage TOPIC: " << topic_);
          ROS_DEBUG_STREAM("storage COLLECTION: " << collection_.at(i));
          mongo::BSONObjBuilder document;
          mongo::Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
          document.append("header",
                          BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
          document.append("format", msg->format);
          document.appendBinData("data", msg->data.size(), mongo::BinDataGeneral, &msg->data[0]);
          std::string type(ros::message_traits::DataType<sensor_msgs::CompressedImage>::value());
          document.append("type", type);
          document.append("size", (int)msg->data.size());

          if (mode_ == COMPRESSED)
          {
              document.append("mode_", "compressed");
          }
          else if (mode_ == COMPRESSED_DEPTH)
          {
              document.append("mode_", "compressed_depth");
          }
          client_connection_->insert(collection_compressed_.at(i), document.obj());
      }
  }
  /**
   * CompressedImage callback to save compressed images and compressed depth images
   * @param msg compressed image
   */
  void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {

          ROS_WARN_STREAM("we welcome our compressed G #" << g_count_comp);
          saveCompressedImage(msg);
          g_count_comp++;
sleep(3);
  }

  /**
   * Theora Callback to save video packets
   * @param msg theora video
   */
  void theoraCallback(const theora_image_transport::PacketConstPtr& msg)
  {
    ROS_DEBUG_STREAM("theora collection size " << collection_theora_.size());

    for (int i = 0; i < collection_theora_.size(); i++)
    {
      ROS_DEBUG_STREAM("Theora called: " << msg->packetno);
      ROS_DEBUG_STREAM("storage TOPIC: " << topic_);
      ROS_DEBUG_STREAM("storage COLLECTION: " << collection_.at(i));
      mongo::BSONObjBuilder document;

      mongo::Date_t timestamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
      document.append("header",
                      BSON("seq" << msg->header.seq << "stamp" << timestamp << "frame_id" << msg->header.frame_id));
      document.append("format", "theora");
      document.append("start", msg->b_o_s);
      document.append("end", msg->e_o_s);
      document.append("position", (int)msg->granulepos);
      document.append("packetno", (int)msg->packetno);
      document.appendBinData("data", msg->data.size(), mongo::BinDataGeneral, &msg->data[0]);
      string type(ros::message_traits::DataType<theora_image_transport::Packet>::value());
      document.append("size", (int)msg->data.size());
      document.append("type", type);
      document.append("mode_", "theora");
      client_connection_->insert(collection_theora_.at(i), document.obj());
    }
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
    topic_ = req.topic;
    db_host_ = req.db_host;
    mode_ = req.mode;

    bool found_cam = false;
    int found_cam_no = 0;
    string current_topic = "";
    string requested_topic = "";

    if (req.cam_no > camera_list_.size())
    {
      ROS_ERROR_STREAM("Cameras have to be added in order. Camera " << req.cam_no << " cannot be added.");
      ROS_ERROR_STREAM("Please add camera with index " << camera_list_.size());
    }
    else
    {
      // set collection name
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

      // assigns sub_ to required callback
      createSubscriber(req.topic, req.mode);

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
            current_topic = it->first.getTopic() + getModeString(it->second);
            requested_topic = req.topic + getModeString(req.mode);
            if (current_topic.find(requested_topic) != string::npos && found_cam)
            {
              ROS_DEBUG_STREAM("Updating topic");

              // shutdown and delete old subscriber
              auto pos = camera_list_.at(idx).begin();
              while (pos->second != req.mode && pos != camera_list_.at(idx).end())
                ++pos;
              ros::Subscriber temp_sub = pos->first;
              temp_sub.shutdown();
              camera_list_.at(idx).erase(pos);
              // add updated Subscriber
              camera_list_.at(idx).insert(std::make_pair(sub_, req.mode));
              return true;
            }
          }
        }
      }

      if (!found_cam)
      {
        // add new cam
        ROS_WARN_STREAM("Adding new Camera");
        ModeSubscriber sub_map;
        sub_map.insert(std::make_pair(sub_, req.mode));
        camera_list_.push_back(sub_map);
        return true;
      }
      else
      {
        // add new subscriber
        ROS_WARN_STREAM("Adding new Subscriber");
        camera_list_.at(req.cam_no).insert(std::make_pair(sub_, req.mode));
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
  bool delConfig(iai_image_logging_msgs::DeleteRequest& req, iai_image_logging_msgs::DeleteResponse& res)
  {
    if (camera_list_.size() < req.cam_no)
    {
      ROS_ERROR_STREAM("Camera does not exist");
    }
    else
    {
      for (auto it = camera_list_.at(req.cam_no).begin(); it != camera_list_.at(req.cam_no).end(); it++)
      {
        ROS_WARN_STREAM("requested topic: " << req.topic);
        ROS_WARN_STREAM("requested mode: " << req.mode);
        ROS_WARN_STREAM("requested cam: " << req.cam_no);


        ROS_WARN_STREAM("Current topic: " << it->first.getTopic());
        ROS_WARN_STREAM("Current mode: " << it->second);
        ROS_WARN_STREAM("Current cam: " << req.cam_no);


        if (it->first.getTopic().find(req.topic) != string::npos && it->second == req.mode)
        {
          // TODO: Check for integrity of subscriber list in camera
          ROS_WARN_STREAM("Shutting down sub");
          Subscriber temp_sub = it->first;
          temp_sub.shutdown();
          ROS_WARN_STREAM("Found fitting topic and mode, erasing...");
          camera_list_.at(req.cam_no).erase(it);
        }
        if (camera_list_.at(req.cam_no).empty())
        {
          // TODO delete camera
          ROS_WARN_STREAM("Camera order may be broken. be careful!");
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
  void createSubscriber(string topic, int mode)
  {
    switch (mode)
    {
      case (RAW):
          ops_r_.template init<sensor_msgs::Image>(topic_, 1,boost::bind(&Storage::imageCallback, this, _1));
            ops_r_.transport_hints = ros::TransportHints();
            ops_r_.allow_concurrent_callbacks = true;
            sub_ = nh_r_.subscribe(ops_r_);
            break;
      case (COMPRESSED):
          ops_c_.template init<sensor_msgs::CompressedImage>(topic_+ "/compressed", 1,boost::bind(&Storage::compressedImageCallback, this, _1));
            ops_c_.transport_hints = ros::TransportHints();
            ops_c_.allow_concurrent_callbacks = true;
            sub_ = nh_c_.subscribe(ops_c_);
        break;
      case (THEORA):
        sub_ = nh_.subscribe(topic + "/theora", 1, &Storage::theoraCallback, this);
        break;
      case (DEPTH):
        sub_ = nh_.subscribe(topic, 1, &Storage::imageCallback, this);
        break;
      case (COMPRESSED_DEPTH):
        sub_ = nh_.subscribe(topic + "/compressed", 1, &Storage::compressedImageCallback, this);
        break;
      default:
        break;
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
      ops_r_.template init<sensor_msgs::Image>(topic_, 1,boost::bind(&Storage::imageCallback, this, _1));
      ops_r_.transport_hints = ros::TransportHints();
      ops_r_.allow_concurrent_callbacks = true;
      sub_ = nh_r_.subscribe(ops_r_);

      sub_list_.insert(std::make_pair(sub_, RAW));
    camera_list_.push_back(sub_list_);
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
  ros::AsyncSpinner a_r(1,&cb_queue_r_);
  ros::AsyncSpinner a_c(1,&cb_queue_c_);
  a_r.start();
  a_c.start();
  //cb_queue_r_.callAvailable();

  ros::spin();
  return 0;
}