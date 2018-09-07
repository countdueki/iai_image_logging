//
// Created by tammo on 06.09.18.
//

#include "logger.h"

DBClientConnection* mongodb_conn;

/**
 *
 * @param conf
 * @param sample_size
 */
void printDatabaseEntries(iai_image_logging_msgs::CompressedConfig conf, int sample_size)
{
  if (count != 0 && count % sample_size == 0)
  {
    ROS_INFO_STREAM("Saved " << sample_size << " images to " << conf.collection);
    ROS_INFO_STREAM("Format = " << conf.format);
    ROS_INFO_STREAM("JPEG quality = " << conf.jpeg_quality);
    ROS_INFO_STREAM("PNG level = " << conf.png_level);
  }
}

/**
 *
 */
void matrixFunction()
{
  boost::shared_ptr<CompConf> conf;
  conf = g_cfg;
  int sample_size = 100;

  if (count < sample_size)
  {
    conf->format = "jpeg";
    conf->jpeg_quality = 100;
    conf->png_level = 1;
    conf->collection = "db.im_raw_comp_jpeg100_png1";
    configurationCb(*conf);
  }

  else if (count < 2 * sample_size)
  {
    conf->format = "jpeg";
    conf->jpeg_quality = 100;
    conf->png_level = 5;
    conf->collection = "db.im_raw_comp_jpeg100_png9";
    configurationCb(*conf);
  }

  else if (count < 3 * sample_size)
  {
    conf->format = "png";
    conf->jpeg_quality = 100;
    conf->png_level = 1;
    conf->collection = "db.im_raw_comp_png1_jpeg100";
    configurationCb(*conf);
  }

  else if (count < 4 * sample_size)
  {
    conf->format = "png";
    conf->png_level = 1;
    conf->jpeg_quality = 1;
    conf->collection = "db.im_raw_comp_png1_jpeg1";
    configurationCb(*conf);
  }

  else if (count >= 4 * sample_size)
  {
    conf->format = "jpeg";
    conf->jpeg_quality = 1;
    conf->collection = "STOPPING WITH MISSING '.'";
    configurationCb(*conf);
  }
  printDatabaseEntries(*conf, sample_size);
  count++;
}

// TODO new type with sensor or ask dynamic server
void savingImagesCb(const sensor_msgs::ImageConstPtr& msg)
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

  // Check connection to MongoDB
  string err = string("");
  string db_host = g_cfg->db_host;
  mongodb_conn = new DBClientConnection(/* auto reconnect*/ true);
  if (!mongodb_conn->connect(g_cfg->db_host, err))
  {
    ROS_ERROR("Failed to connect to MongoDB: %s", err.c_str());
  }

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber img_sub = it.subscribe("/preprocessor/images", 1, savingImagesCb);

  ros::Rate r = 1;
  while (n.ok())
  {
    ros::spin();
    r.sleep();
  }
}