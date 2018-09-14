//
// Created by tammo on 13.09.18.
//

#include "storage.h"

Storage storage;

void writeImage(const sensor_msgs::CompressedImageConstPtr& msg)
{
  ROS_INFO_STREAM("Format: " << msg->format);
  std::vector<int> compression_params;
  compression_params.push_back(CV_LOAD_IMAGE_COLOR);
  compression_params.push_back(1);
  cv_bridge::CvImagePtr cv_image;
  // cv_image->encoding = sensor_msgs::image_encodings::BAYER_GRBG8;
  cv_image = cv_bridge::toCvCopy(msg);
  cv::imshow("DAYUM", cv_image->image);

  cv::imwrite("test.jpg", cv_image->image);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{  // matrixFunction(); // for building test entries

    // TODO: fix saving of raw images
  if (storage.getMode() == RAW)
  {
    ROS_DEBUG_STREAM("storage TOPIC: " << storage.getTopic());
    ROS_DEBUG_STREAM("storage COLLECTION: " << storage.getCollection());
    mongo::client::initialize();
    mongo::BSONObjBuilder document;
    mongo::Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
    document.append("header", BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
    document.append("encoding", msg->encoding);
    document.append("width", msg->width);
    document.append("height", msg->height);
    document.append("is_bigendian", msg->is_bigendian);

    document.appendBinData("data", msg->data.size(), mongo::BinDataGeneral, &msg->data);
    std::string type(ros::message_traits::DataType<sensor_msgs::CompressedImage>::value());
    document.append("type", type);
    document.append("size", (int)msg->data.size());
    storage.getClientConnection()->insert(storage.getCollection(), document.obj());
  }
  else
  {
    ROS_DEBUG_STREAM("No compressed images will be logged");
  }
}

void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{  // matrixFunction(); // for building test entries

  if (storage.getMode() == COMPRESSED)
  {
    ROS_DEBUG_STREAM("storage TOPIC: " << storage.getTopic());
    ROS_DEBUG_STREAM("storage COLLECTION: " << storage.getCollection());
    mongo::client::initialize();
    mongo::BSONObjBuilder document;
    mongo::Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
    document.append("header", BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
    document.append("format", msg->format);
    document.appendBinData("data", msg->data.size(), mongo::BinDataGeneral, &msg->data);
    std::string type(ros::message_traits::DataType<sensor_msgs::CompressedImage>::value());
    document.append("type", type);
    document.append("size", (int)msg->data.size());
    storage.getClientConnection()->insert(storage.getCollection(), document.obj());
  }
  else
  {
    ROS_DEBUG_STREAM("No compressed images will be logged");
  }
}

void theoraCallback(const theora_image_transport::PacketConstPtr& msg)
{
  if (storage.getMode() == THEORA)
  {
    ROS_DEBUG_STREAM("storage TOPIC: " << storage.getTopic());
    ROS_DEBUG_STREAM("storage COLLECTION: " << storage.getCollection());
    mongo::client::initialize();
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
    storage.getClientConnection()->insert(storage.getCollection(), document.obj());
  }
  else
  {
    ROS_DEBUG_STREAM("no theora will be logged");
  }
}

bool update(iai_image_logging_msgs::UpdateRequest& req, iai_image_logging_msgs::UpdateResponse& res)
{
  storage.setCollection(req.set.collection);
  storage.setTopic(req.set.topic);
  storage.setDbHost(req.set.db_host);
  storage.setMode(req.set.mode);
  res.success = true;
  ROS_INFO_STREAM("UPDATE CALLED, MODE SET TO: " << storage.getMode());

  return true;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "storage");
  ros::NodeHandle nh;

  string errmsg;
  if (!storage.getClientConnection()->connect(storage.getDbHost(), errmsg))
  {
    ROS_ERROR("Failed to connect to MongoDB: %s", errmsg.c_str());
    return -1;
  }

  ros::Subscriber sub_raw = nh.subscribe(storage.getTopic(), 1, &imageCallback);
  ros::Subscriber sub_compressed = nh.subscribe(storage.getTopic() + "/compressed", 1, &compressedImageCallback);
  ros::Subscriber sub_theora = nh.subscribe(storage.getTopic() + "/theora", 1, &theoraCallback);

  ros::ServiceServer update_config = nh.advertiseService("storage/update", &update);

  ros::Rate hz_rate(60.0);
  while (nh.ok())
  {
    ros::spinOnce();

    hz_rate.sleep();
  }

  return 0;
}