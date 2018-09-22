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
    collection_ = "db.standard";
    topic_ = "camera/rgb/image_raw";

    mode_ = 1;
    // maybe init camera_list_ here
    string errmsg;
    if (!client_connection_->connect(db_host_, errmsg))
    {
      ROS_ERROR("Failed to connect to MongoDB: %s", errmsg.c_str());
    }
    mongo::client::initialize();

    update_config_ = nh_.advertiseService("storage/update", &Storage::update, this);

    add_service_ = nh_.advertiseService("storage/add", &Storage::addConfig, this);
    del_service_ = nh_.advertiseService("storage/del", &Storage::delConfig, this);
  }

  Storage(const Storage& old);
  const Storage& operator=(const Storage& old);
  // ~Storage(){}

  string topic_;
  string db_host_;
  string collection_;
  int mode_;

  ros::NodeHandle nh_;
  ros::ServiceServer update_config_;
  ros::ServiceServer add_service_;
  ros::ServiceServer del_service_;
  ros::Subscriber sub_;
  vector<Subscriber> sub_list_;
  vector<vector<Subscriber>> camera_list_;
  mongo::DBClientConnection* client_connection_ = new mongo::DBClientConnection(true);

public:
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {  // matrixFunction(); // for building test entries

    ROS_DEBUG_STREAM("msg format: " << msg->encoding);
    ROS_DEBUG_STREAM("storage TOPIC: " << topic_);
    ROS_DEBUG_STREAM("storage COLLECTION: " << collection_);
    mongo::BSONObjBuilder document;
    mongo::Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
    document.append("header", BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
    document.append("encoding", msg->encoding);
    document.append("width", msg->width);
    document.append("height", msg->height);
    document.append("is_bigendian", msg->is_bigendian);
    document.append("step", msg->step);

    document.appendBinData("data", msg->data.size(), mongo::BinDataGeneral, &msg->data);
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
    client_connection_->insert(collection_, document.obj());
  }

  void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
  {
    ROS_DEBUG_STREAM("FORMAT: " << msg->format);
    ROS_DEBUG_STREAM("storage TOPIC: " << topic_);
    ROS_DEBUG_STREAM("storage COLLECTION: " << collection_);
    mongo::BSONObjBuilder document;
    mongo::Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
    document.append("header", BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
    document.append("format", msg->format);
    document.appendBinData("data", msg->data.size(), mongo::BinDataGeneral, &msg->data);
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
    client_connection_->insert(collection_, document.obj());
  }

  void theoraCallback(const theora_image_transport::PacketConstPtr& msg)
  {
    ROS_DEBUG_STREAM("Theora called: " << msg->packetno);
    ROS_DEBUG_STREAM("storage TOPIC: " << topic_);
    ROS_DEBUG_STREAM("storage COLLECTION: " << collection_);
    mongo::BSONObjBuilder document;

    mongo::Date_t timestamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
    document.append("header",
                    BSON("seq" << msg->header.seq << "stamp" << timestamp << "frame_id" << msg->header.frame_id));
    document.append("format", "theora");
    document.append("start", msg->b_o_s);
    document.append("end", msg->e_o_s);
    document.append("position", (int)msg->granulepos);
    document.append("packetno", (int)msg->packetno);
    document.appendBinData("data", msg->data.size(), mongo::BinDataGeneral, &msg->data);
    string type(ros::message_traits::DataType<theora_image_transport::Packet>::value());
    document.append("size", (int)msg->data.size());
    document.append("type", type);
    document.append("mode_", "theora");
    client_connection_->insert(collection_, document.obj());
  }
    void createSubscriber(string topic, int mode) {
      switch (mode) {
        case (RAW):
          sub_ = nh_.subscribe(topic, 1, &Storage::imageCallback, this);
              break;
        case (COMPRESSED):

          sub_ = nh_.subscribe(topic + "/compressed", 1, &Storage::compressedImageCallback, this);

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


    bool update(iai_image_logging_msgs::UpdateRequest& req, iai_image_logging_msgs::UpdateResponse& res)
  {
    collection_ = req.collection;
    topic_ = req.topic;
    db_host_ = req.db_host;
    mode_ = req.mode;

    bool allow_new_cam = false;
    bool allow_new_sub = false;

    if (req.cam_no > camera_list_.size())
    {
      ROS_ERROR_STREAM("Cameras have to be added in order. Camera " << req.cam_no << " cannot be added.");
      ROS_ERROR_STREAM("Please add camera with index " << camera_list_.size());
    }
    else
    {
      // assigns sub_ to required callback
      createSubscriber(req.topic, req.mode);


      // Loop camera list for existing entries
      for (int index = 0; index < camera_list_.size(); index++)
      {
        // search camera
        if (index == req.cam_no && !camera_list_.at(index).empty())
        {
          // search topic_ in existing camera
          for (int sub_index = 0; sub_index < camera_list_.at(index).size(); sub_index++)
          {
            string current_topic = camera_list_.at(index).at(sub_index).getTopic();

            // update existing topic_ in existing camera
            if (current_topic.find(req.topic) != std::string::npos)
            {
              ROS_WARN_STREAM("Updating Subscriber");
              camera_list_.at(index).at(sub_index) = sub_;
            }
            else
            {
              allow_new_sub = true;
            }
          }
        }
        // camera has to be added sequentially
        else if (req.cam_no == camera_list_.size())
        {
            // search for topic_ in every camera
          for (int sub_idx = 0; sub_idx < camera_list_.size(); sub_idx++)
          {
            string current_topic = camera_list_.at(index).at(sub_idx).getTopic();
            if (current_topic.find(sub_.getTopic()) != std::string::npos)
            {
              ROS_ERROR_STREAM("Topic is already subscribed to by camera: " << index);
              allow_new_cam = false;
            }
            else
            {
              allow_new_cam = true;
            }
          }
          /* requested topic_ not found in any camera
             and requested camera not found
             add new camera and subscriber */
          if (allow_new_cam && allow_new_sub)
          {
            ROS_WARN_STREAM("Adding new Camera");
            vector<Subscriber> sub_vector;
            sub_vector.push_back(sub_);
            camera_list_.push_back(sub_vector);
          }
          /* requested topic_ not found in any camera.
             add new subscriber */
          else
          {
            ROS_WARN_STREAM("Adding new Subscriber");
            vector<Subscriber> sub_vector;
            sub_vector.push_back(sub_);
            camera_list_.at(index) = sub_vector;
          }
        }
      }
    }
    res.success = true;

    return true;
  }

  void init()
  {
    sub_ = nh_.subscribe("", 1, &Storage::imageCallback, this);
    sub_list_.push_back(sub_);
    camera_list_.push_back(sub_list_);
  }

  bool addConfig(iai_image_logging_msgs::UpdateRequest& req, iai_image_logging_msgs::UpdateResponse& res)
  {
    if (camera_list_.size() < req.cam_no)
    {
      ROS_ERROR_STREAM("Camera does not exist");
    }
    else
    {
      update(req, res);
    }
  }

  bool delConfig(iai_image_logging_msgs::UpdateRequest& req, iai_image_logging_msgs::UpdateResponse& res)
  {
    if (camera_list_.size() < req.cam_no)
    {
      ROS_ERROR_STREAM("Camera does not exist");
    }
    else
    {
      createSubscriber(req.topic,req.mode);
    }
  }

  const ros::NodeHandle& getNodeHandle() const
  {
    return nh_;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "storage");
  Storage* storage = &Storage::Instance();

  storage->init();
  ros::Rate hz_rate(60.0);
  while (storage->getNodeHandle().ok())
  {
    ros::spinOnce();

    hz_rate.sleep();
  }

  return 0;
}