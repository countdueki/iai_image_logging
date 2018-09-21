//
// Created by tammo on 13.09.18.
//

#include "storage.h"
class Storage
{
public:
  static Storage &Instance()
  {
      static boost::shared_ptr<Storage> instance (new Storage);
      return *instance;
  }
private:

    Storage(){
        dbHost = "localhost";
        collection = "db.standard";
        topic = "camera/rgb/image_raw";

        mode = 1;
        // maybe init cams here
        cams.add(new Camera());
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

  Storage(const Storage &old);
  const Storage &operator=(const Storage &old);
 // ~Storage(){}

  string topic;
  string dbHost;
  string collection;
  int mode;

  ros::NodeHandle nh;
  ros::ServiceServer update_config;
  ros::ServiceServer add_service;
  ros::ServiceServer del_service;
  ros::Subscriber sub_raw;
  ros::Subscriber sub_compressed;
  ros::Subscriber sub_theora;

  CompositeCamera cams;

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


    //TODO better existence check for cams
    if (req.cam_no > cams.size())
    {
      ROS_ERROR_STREAM("Cameras have to be added in order. Camera " << req.cam_no << " cannot be added.");
      ROS_ERROR_STREAM("Please add camera with index " << cams.size());
    }
    else
    {
      switch(mode){
          case (RAW):
            sub_raw = nh.subscribe(req.topic, 1, &Storage::imageCallback, this);

              for (int i = 0; i < cams.size(); i++)
              {
                if (cams.at(i)->getNo() == req.cam_no)
                {
                  cams.at(req.cam_no)->updateRaw(sub_raw);
                }
                else
                {
                  cams.add(new Camera(req.cam_no));
                  cams.at(req.cam_no)->updateRaw(sub_raw);
                }
              }
              break;
          case (COMPRESSED):
            ROS_ERROR_STREAM("1");

              sub_compressed = nh.subscribe(req.topic + "/compressed", 1, &Storage::compressedImageCallback, this);

              for (int i = 0; i < cams.size(); i++)
              {
                ROS_ERROR_STREAM("2");
                ROS_WARN_STREAM("CAMS SIZE: " << cams.size());

                if (cams.at(i)->getNo() == req.cam_no)
                {
                  ROS_ERROR_STREAM("3");

                  cams.at(req.cam_no)->updateCompressed(sub_compressed);
                }
                else
                {
                  ROS_ERROR_STREAM("4");

                  cams.add(new Camera(req.cam_no));
                  cams.at(req.cam_no)->updateCompressed(sub_compressed);
                }
              }
              break;
          case (THEORA):
            sub_theora = nh.subscribe(req.topic + "/theora", 1, &Storage::theoraCallback, this);

              for (int i = 0; i < cams.size(); i++)
              {
                if (cams.at(i)->getNo() == req.cam_no)
                {
                  cams.at(req.cam_no)->updateTheora(sub_theora);
                }
                else
                {
                  cams.add(new Camera(req.cam_no));
                  cams.at(req.cam_no)->updateTheora(sub_theora);
                }
              }
              break;
        default:
          break;

      }

      res.success = true;

      return true;
    }
  }

  void init()
  {
          sub_raw = nh.subscribe(topic, 1, &Storage::imageCallback, this);
          sub_compressed = nh.subscribe(topic + "/compressed", 1, &Storage::compressedImageCallback, this);
          sub_theora = nh.subscribe(topic + "/theora", 1, &Storage::theoraCallback, this);

          cams.init(sub_raw, sub_compressed, sub_theora);

  }

  bool addConfig(iai_image_logging_msgs::UpdateRequest& req, iai_image_logging_msgs::UpdateResponse& res)
  {
    if (cams.size() < req.cam_no)
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
    if (cams.size() < req.cam_no)
    {
      ROS_ERROR_STREAM("Camera does not exist");
    }
    else
    {
      cams.at(req.cam_no)->del(req);
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