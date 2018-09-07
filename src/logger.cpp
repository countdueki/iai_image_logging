//
// Created by tammo on 06.09.18.
//

#include "logger.h"
CompConfPtr g_cfg(new CompConf());

DBClientConnection* mongodb_conn;

void savingImagesCb(const sensor_msgs::CompressedImageConstPtr& msg)
{  // matrixFunction(); // for building test entries
  initialize();
  BSONObjBuilder document;
  std::string collection = g_cfg->collection;

  ROS_DEBUG_STREAM("Saving Image at sec " << msg->header.stamp.sec << " with type " << msg->format);

  Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
  document.append("header", BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
  document.append("format", msg->format);
  document.appendBinData("data", msg->data.size(), BinDataGeneral, (&msg->data[0]));

  add_meta_for_msg<sensor_msgs::CompressedImage>(msg, document);
  mongodb_conn->insert(collection, document.obj());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "logger");
  ros::NodeHandle nh;

  g_cfg->format = "jpeg";
  g_cfg->jpeg_quality = 100;
  g_cfg->png_level = 1;
  g_cfg->collection = "db.direct_compressed";
  g_cfg->db_host = "localhost";

  // Check connection to MongoDB
  string err = string("");
  string db_host = g_cfg->db_host;
  mongodb_conn = new DBClientConnection(/* auto reconnect*/ true);
  if (!mongodb_conn->connect(g_cfg->db_host, err))
  {
    ROS_ERROR("Failed to connect to MongoDB: %s", err.c_str());
  }

  // image_transport::ImageTransport it(nh);
  // image_transport::TransportHints th("compressed");
  // image_transport::Subscriber img_sub = it.subscribe("preprocessor/images", 1, savingImagesCb, ros::VoidPtr(), th);
  // image_transport::Subscriber log_sub = it.subscribe("/preprocessor/images/compressed", 1, savingImagesCb);

  ros::Subscriber sub_img = nh.subscribe("/camera/rgb/image_raw/compressed", 1, savingImagesCb);
  ros::Rate r = 1;
  while (nh.ok())
  {
    ros::spinOnce();
    r.sleep();
  }
}