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
    dbHost = "localhost";
    collection = "db.standard";
    topic = "camera/rgb/image_raw";

    mode = 1;
    // maybe init camera_list here
    string errmsg;
    if (!clientConnection->connect(dbHost, errmsg))
    {
      ROS_ERROR("Failed to connect to MongoDB: %s", errmsg.c_str());
    }
    mongo::client::initialize();

    update_config = nh.advertiseService("storage/update", &Storage::update, this);

    add_service = nh.advertiseService("storage/add", &Storage::addConfig, this);
    del_service = nh.advertiseService("storage/del", &Storage::delConfig, this);
  }

  Storage(const Storage& old);
  const Storage& operator=(const Storage& old);
  // ~Storage(){}

  string topic;
  string dbHost;
  string collection;
  int mode;

  ros::NodeHandle nh;
  ros::ServiceServer update_config;
  ros::ServiceServer add_service;
  ros::ServiceServer del_service;
  ros::Subscriber sub_;
  vector<Subscriber> sub_list_;
  vector<vector<Subscriber>> camera_list;
  mongo::DBClientConnection* clientConnection = new mongo::DBClientConnection(true);

public:
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {  // matrixFunction(); // for building test entries

    if (mode == RAW || mode == DEPTH)
    {
      ROS_DEBUG_STREAM("msg format: " << msg->encoding);
      ROS_DEBUG_STREAM("storage TOPIC: " << topic);
      ROS_DEBUG_STREAM("storage COLLECTION: " << collection);
      mongo::BSONObjBuilder document;
      mongo::Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
      document.append("header",
                      BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
      document.append("encoding", msg->encoding);
      document.append("width", msg->width);
      document.append("height", msg->height);
      document.append("is_bigendian", msg->is_bigendian);
      document.append("step", msg->step);

      document.appendBinData("data", msg->data.size(), mongo::BinDataGeneral, &msg->data);
      std::string type(ros::message_traits::DataType<sensor_msgs::Image>::value());
      document.append("type", type);
      document.append("size", (int)msg->data.size());
      if (mode == RAW)
      {
        document.append("mode", "raw");
      }
      else if (mode == DEPTH)
      {
        document.append("mode", "depth");
      }
      clientConnection->insert(collection, document.obj());
    }
    else
    {
      ROS_DEBUG_STREAM("No compressed images will be logged");
    }
  }

  void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
  {
    if (mode == COMPRESSED || mode == COMPRESSED_DEPTH)
    {
      ROS_DEBUG_STREAM("FORMAT: " << msg->format);
      ROS_DEBUG_STREAM("storage TOPIC: " << topic);
      ROS_DEBUG_STREAM("storage COLLECTION: " << collection);
      mongo::BSONObjBuilder document;
      mongo::Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
      document.append("header",
                      BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
      document.append("format", msg->format);
      document.appendBinData("data", msg->data.size(), mongo::BinDataGeneral, &msg->data);
      std::string type(ros::message_traits::DataType<sensor_msgs::CompressedImage>::value());
      document.append("type", type);
      document.append("size", (int)msg->data.size());

      if (mode == COMPRESSED)
      {
        document.append("mode", "compressed");
      }
      else if (mode == COMPRESSED_DEPTH)
      {
        document.append("mode", "compressed_depth");
      }
      clientConnection->insert(collection, document.obj());
    }
    else
    {
      ROS_DEBUG_STREAM("No compressed images will be logged");
    }
  }

  void theoraCallback(const theora_image_transport::PacketConstPtr& msg)
  {
    if (mode == THEORA)
    {
      ROS_DEBUG_STREAM("Theora called: " << msg->packetno);
      ROS_DEBUG_STREAM("storage TOPIC: " << topic);
      ROS_DEBUG_STREAM("storage COLLECTION: " << collection);
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
      document.append("mode", "theora");
      clientConnection->insert(collection, document.obj());
    }
    else
    {
      ROS_DEBUG_STREAM("no theora will be logged");
    }
  }

  bool update(iai_image_logging_msgs::UpdateRequest& req, iai_image_logging_msgs::UpdateResponse& res)
  {
    collection = req.collection;
    topic = req.topic;
    dbHost = req.db_host;
    mode = req.mode;

    // TODO better existence check for camera_list
    if (req.cam_no > camera_list.size())
    {
      ROS_ERROR_STREAM("Cameras have to be added in order. Camera " << req.cam_no << " cannot be added.");
      ROS_ERROR_STREAM("Please add camera with index " << camera_list.size());
    }
    else
    {
      switch (mode)
      {
        case (RAW):
          sub_ = nh.subscribe(req.topic, 1, &Storage::imageCallback, this);
          break;
        case (COMPRESSED):

          sub_ = nh.subscribe(req.topic + "/compressed", 1, &Storage::compressedImageCallback, this);

          break;
        case (THEORA):
          sub_ = nh.subscribe(req.topic + "/theora", 1, &Storage::theoraCallback, this);
          break;
        default:
          break;
      }


      for (int index = 0; index < camera_list.size(); index++)
      {
        ROS_INFO_STREAM("INDEX START: " << index);

        if (index == req.cam_no && !camera_list.at(index).empty())
        {
           for (int sub_index = 0; sub_index < camera_list.at(index).size(); sub_index++){
               ROS_INFO_STREAM("Required Topic: " << req.topic);
               ROS_INFO_STREAM("Topic in List: " << camera_list.at(index).at(sub_index).getTopic());

               if (camera_list.at(index).at(sub_index).getTopic().find(req.topic) != std::string::npos)
          {
            ROS_WARN_STREAM("Updating Subscriber");
            camera_list.at(index).at(sub_index) = sub_;
          } else {
              ROS_WARN_STREAM("Adding new Subscriber");
              vector<Subscriber> sub_vector;
              sub_vector.push_back(sub_);
              camera_list.at(index) = sub_vector;
          }

          }
        }
        else if (req.cam_no == camera_list.size())
        {
          ROS_WARN_STREAM("Adding new Camera");
          vector<Subscriber> sub_vector;
          sub_vector.push_back(sub_);
          camera_list.push_back(sub_vector);

        }
        ROS_INFO_STREAM("INDEX: " << index);

      }
    }
    res.success = true;

    return true;
  }

  void init()
  {
    sub_ = nh.subscribe(topic, 1, &Storage::imageCallback, this);
    sub_list_.push_back(sub_);
    camera_list.push_back(sub_list_);
  }

  bool addConfig(iai_image_logging_msgs::UpdateRequest& req, iai_image_logging_msgs::UpdateResponse& res)
  {
    if (camera_list.size() < req.cam_no)
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
    if (camera_list.size() < req.cam_no)
    {
      ROS_ERROR_STREAM("Camera does not exist");
    }
    else
    {
      // camera_list.at(req.cam_no)->del(req);
    }
  }

public:
  int getMode() const
  {
    return mode;
  }

  void setMode(int mode)
  {
    Storage::mode = mode;
  }

  const string& getTopic() const
  {
    return topic;
  }

  void setTopic(const string& topic)
  {
    Storage::topic = topic;
  }

  const string& getDbHost() const
  {
    return dbHost;
  }

  void setDbHost(const string& dbHost)
  {
    Storage::dbHost = dbHost;
  }

  const string& getCollection() const
  {
    return collection;
  }

  void setCollection(const string& collection)
  {
    Storage::collection = collection;
  }

  mongo::DBClientConnection* getClientConnection() const
  {
    return clientConnection;
  }

  void setClientConnection(mongo::DBClientConnection* clientConnection)
  {
    Storage::clientConnection = clientConnection;
  }
  const ros::NodeHandle& getNh() const
  {
    return nh;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "storage");
  Storage* storage = &Storage::Instance();

  storage->init();
  ros::Rate hz_rate(60.0);
  while (storage->getNh().ok())
  {
    ros::spinOnce();

    hz_rate.sleep();
  }

  return 0;
}