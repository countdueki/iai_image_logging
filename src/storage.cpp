//
// Created by tammo on 13.09.18.
//

#include "storage.h"

class Storage
{
public:
  Storage()
  {
    dbHost = "localhost";
    collection = "db.standard";
    topic = "camera/rgb/image_raw";
    mode = 1;
    cams.push_back(cam);

    string errmsg;
    if (!clientConnection->connect(dbHost, errmsg))
    {
      ROS_ERROR("Failed to connect to MongoDB: %s", errmsg.c_str());
    }
    mongo::client::initialize();

    update_config = nh.advertiseService("storage/update", &Storage::update, this);
  }

private:
  string topic;
  string dbHost;
  string collection;
  int mode;

  ros::NodeHandle nh;
  ros::ServiceServer update_config;
  ros::Subscriber sub_raw;
  ros::Subscriber sub_compressed;
  ros::Subscriber sub_theora;

  Camera cam;
  std::vector<Camera> cams;

  mongo::DBClientConnection* clientConnection = new mongo::DBClientConnection(true);

public:
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {  // matrixFunction(); // for building test entries

    if (mode == RAW || mode == DEPTH)
    {
      // ROS_WARN_STREAM("msg format: " << msg->encoding);
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
      ROS_WARN_STREAM("FORMAT: " << msg->format);
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
      ROS_WARN_STREAM("Theora called: " << msg->packetno);
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
    collection = req.set.collection;
    topic = req.set.topic;
    dbHost = req.set.db_host;
    mode = req.set.mode;

    sub_raw = nh.subscribe(req.set.topic, 1, &Storage::imageCallback, this);
    sub_compressed = nh.subscribe(req.set.topic + "/compressed", 1, &Storage::compressedImageCallback, this);
    sub_theora = nh.subscribe(req.set.topic + "/theora", 1, &Storage::theoraCallback, this);

    // TODO: Better existence and elements check
    cams.push_back(cam);

    ROS_WARN_STREAM("the crucial part");
    cams[req.set.cam_no].init(sub_raw, sub_compressed, sub_theora);
    ROS_WARN_STREAM("all set");

    res.success = true;
    ROS_INFO_STREAM("UPDATE CALLED, MODE SET TO: " << mode);

    return true;
  }

  void init()
  {
    sub_raw = nh.subscribe(topic, 1, &Storage::imageCallback, this);
    sub_compressed = nh.subscribe(topic + "/compressed", 1, &Storage::compressedImageCallback, this);
    sub_theora = nh.subscribe(topic + "/theora", 1, &Storage::theoraCallback, this);

    cams[0].init(sub_raw, sub_compressed, sub_theora);
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
  Storage storage;

  storage.init();
  ros::Rate hz_rate(60.0);
  while (storage.getNh().ok())
  {
    ros::spinOnce();

    hz_rate.sleep();
  }

  return 0;
}