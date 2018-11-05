//
// Created by tammo on 12.10.18.
//

#include "../include/header/iai_subscriber.h"

void IAISubscriber::start()
{
  if (spinner_->canStart())
    spinner_->start();
  // queue_.callAvailable();
}
void IAISubscriber::destroy()
{
  sub_.shutdown();
  // delete this;
}

void IAISubscriber::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_WARN_STREAM("motion blur similar: " << motion_ << blur_ << similar_);

    ros::Rate r(rate_);
  if (motion_)
  {
      motion_detected_ = motion_detector.detectMotion(tf_msg_, tf_msg_prev_, tf_base_, tf_camera_);
    if (motion_detected_)
    {
      ROS_WARN_STREAM("motion detected");
      return;
    }
  }
  if (blur_)
  {
    blur_detected_ = blur_detector.detectBlur(msg);
    if (blur_detected_)
    {
      ROS_DEBUG_STREAM("blur detected");
      return;
    }
  }
  if (similar_)
  {
    if (prev_image_)
    {
      sim_curr_ = msg;
      similar_detected_ = sim_detector.detectMSE(sim_prev_, sim_curr_);
      if (similar_detected_)
      {
        ROS_DEBUG_STREAM("similar image detected");

        sim_prev_ = sim_curr_;
        return;
      }
      else if (!motion_detected_ && !blur_detected_)
      {
        sim_prev_ = sim_curr_;
      }
    }
    else
    {
      sim_prev_ = msg;
      prev_image_ = true;
    }
  }
  if (!motion_detected_ && !blur_detected_ && !similar_detected_)
  {
    saveImage(msg);
    pub_.publish(msg);
    // show(msg);
  }
  r.sleep();
}

void IAISubscriber::compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
  ros::Rate r(rate_);
  ROS_DEBUG_STREAM("motion blur similar: " << motion_ << blur_ << similar_);
  if (motion_)
  {
      motion_detected_ = motion_detector.detectMotion(tf_msg_, tf_msg_prev_, tf_base_, tf_camera_);
      if (motion_detected_)
      {
          ROS_WARN_STREAM("motion detected");
          return;
      }
  }
  if (blur_)
  {
    blur_detected_ = blur_detector.detectBlur(msg);
    if (blur_detected_)
    {
      ROS_WARN_STREAM("blur detected");
      return;
    }
  }
  if (similar_)
  {
    if (prev_image_)
    {
      sim_curr_c_ = msg;
      similar_detected_ = sim_detector.detectMSE(sim_prev_c_, sim_curr_c_);
      if (similar_detected_)
      {
        ROS_WARN_STREAM("similar image detected");
        sim_prev_c_ = sim_curr_c_;
        return;
      }
      else if (!motion_detected_ && !blur_detected_)
      {
        sim_prev_c_ = sim_curr_c_;
      }
    }
    else
    {
      sim_prev_c_ = msg;
      prev_image_ = true;
    }
  }
  if (!motion_detected_ && !blur_detected_ && !similar_detected_)
  {
    saveCompressedImage(msg);
    pub_.publish(*msg);
    // show(msg);
  }

  r.sleep();
}

void IAISubscriber::theoraCallback(const theora_image_transport::PacketConstPtr& msg)
{
    ROS_WARN_STREAM("saving theora image");
  ros::Rate r(rate_);
  saveTheora(msg);
  r.sleep();
}

void IAISubscriber::tfCallback(const tf::tfMessageConstPtr& msg){
    if (tf_msg_.use_count() == 0){
        tf_msg_ = msg;
    } else
    {
        tf_msg_prev_ = tf_msg_;
        tf_msg_ = msg;
    }
}

void IAISubscriber::saveImage(const sensor_msgs::ImageConstPtr& msg)
{
  mongo::BSONObjBuilder document;
  mongo::Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
  document.append("header", BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
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

  mongo::BSONObjBuilder meta;

  ros::Time now = ros::Time::now();
  mongo::Date_t nowDate((now.sec * 1000.0) + (now.nsec / 1000000.0));
  meta.append("inserted_at", nowDate);

  meta.append("stored_type", type);
  meta.append("topic", topic_);

  size_t slashIndex = type.find('/');
  std::string package = type.substr(0, slashIndex);
  std::string name = type.substr(slashIndex + 1, type.length() - slashIndex - 1);
  std::string pythonName = package + ".msg._" + name + "." + name;
  meta.append("stored_class", pythonName);

  document.append("_meta", meta.obj());

  client_connection_->insert(collection_, document.obj());
}

void IAISubscriber::saveCompressedImage(const sensor_msgs::CompressedImageConstPtr& msg)
{
  try
  {
    mongo::BSONObjBuilder document;
    document.append("iai_id", id_);
    mongo::Date_t stamp = msg->header.stamp.sec * 1000.0 + msg->header.stamp.nsec / 1000000.0;
    document.append("header", BSON("seq" << msg->header.seq << "stamp" << stamp << "frame_id" << msg->header.frame_id));
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

    mongo::BSONObjBuilder meta;

    ros::Time now = ros::Time::now();
    mongo::Date_t nowDate((now.sec * 1000.0) + (now.nsec / 1000000.0));
    meta.append("inserted_at", nowDate);

    meta.append("stored_type", type);
    meta.append("topic", topic_);

    size_t slashIndex = type.find('/');
    std::string package = type.substr(0, slashIndex);
    std::string name = type.substr(slashIndex + 1, type.length() - slashIndex - 1);
    std::string pythonName = package + ".msg._" + name + "." + name;
    meta.append("stored_class", pythonName);

    document.append("_meta", meta.obj());
    client_connection_->insert(collection_, document.obj());
  }

  catch (std::bad_alloc& ba)
  {
    ROS_ERROR_STREAM("bad_alloc caught: " << ba.what());
  }
}

void IAISubscriber::saveTheora(const theora_image_transport::PacketConstPtr& msg)
{
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

  mongo::BSONObjBuilder meta;

  ros::Time now = ros::Time::now();
  mongo::Date_t nowDate((now.sec * 1000.0) + (now.nsec / 1000000.0));
  meta.append("inserted_at", nowDate);

  meta.append("stored_type", type);
  meta.append("topic", topic_);

  size_t slashIndex = type.find('/');
  std::string package = type.substr(0, slashIndex);
  std::string name = type.substr(slashIndex + 1, type.length() - slashIndex - 1);
  std::string pythonName = package + ".msg._" + name + "." + name;
  meta.append("stored_class", pythonName);

  document.append("_meta", meta.obj());
  client_connection_->insert(collection_, document.obj());
}

void IAISubscriber::createSubscriber(int mode)
{
    tf_sub_ = nh_.subscribe("/tf",1,&IAISubscriber::tfCallback, this);

  switch (mode)
  {
    case (RAW):
    case (DEPTH):
      ops_.template init<sensor_msgs::Image>(topic_, 1, boost::bind(&IAISubscriber::imageCallback, this, _1));
      ops_.transport_hints = ros::TransportHints();
      ops_.allow_concurrent_callbacks = true;
      sub_ = nh_.subscribe(ops_);
      break;

    case (COMPRESSED):
    case (COMPRESSED_DEPTH):
      ops_.template init<sensor_msgs::CompressedImage>(topic_ + "/compressed", 1,
                                                       boost::bind(&IAISubscriber::compressedImageCallback, this, _1));
      ops_.transport_hints = ros::TransportHints();
      ops_.allow_concurrent_callbacks = true;
      sub_ = nh_.subscribe(ops_);
      break;

    case (THEORA):
      ops_.template init<theora_image_transport::Packet>(topic_ + "/theora", 1,
                                                         boost::bind(&IAISubscriber::theoraCallback, this, _1));
      ops_.transport_hints = ros::TransportHints();
      ops_.allow_concurrent_callbacks = true;
      sub_ = nh_.subscribe(ops_);
      break;

    default:
      break;
  }
}

void IAISubscriber::createPublisher(int mode)
{
  switch (mode)
  {
    case (RAW):
    case (DEPTH):

      pub_ = nh_.advertise<sensor_msgs::Image>("/" + id_, 1, true);
      break;

    case (COMPRESSED):
    case (COMPRESSED_DEPTH):
      pub_ = nh_.advertise<sensor_msgs::CompressedImage>("/" + id_, 1, true);

      break;

    case (THEORA):
      pub_ = nh_.advertise<theora_image_transport::Packet>("/" + id_, 1, true);
      break;

    default:
      break;
  }
}

string IAISubscriber::generateID(string topic, string mode_str)
{
  string result;
  // parse topic
  std::string delimiter = "/";

  size_t pos = 0;
  if (topic.at(0) == '/')
    topic.erase(0, 1);

  while ((pos = topic.find(delimiter)) != std::string::npos)
  {
    if (pos == topic.length() - 1)
    {
      topic.erase(pos, 1);
    }
    else
    {
      topic.replace(pos, 1, "__");
    }
  }

  // add mode string
  if (mode_str.at(0) == '/')
  {
    ROS_DEBUG_STREAM("modestring starts with: " << mode_str.at(0));
    mode_str.erase(0, 1);
  }
  result += topic + "__" + mode_str;

  return result;
}

int IAISubscriber::getNumberFromModeString(string mode)
{
  if (mode.find("raw") != std::string::npos)
    return RAW;
  if (mode.find("compressed") != std::string::npos)
    return COMPRESSED;
  if (mode.find("theora") != std::string::npos)
    return THEORA;
  if (mode.find("depth") != std::string::npos)
    return DEPTH;
  if (mode.find("compressedDepth") != std::string::npos)
    return COMPRESSED_DEPTH;
}

string IAISubscriber::getModeString(int mode)
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
      return "/compressedDepth";
    default:
      break;
  }
}

const string& IAISubscriber::getID() const
{
  return id_;
}

const string& IAISubscriber::getTopic() const
{
  return topic_;
}

void IAISubscriber::setRate_(double rate_)
{
  IAISubscriber::rate_ = rate_;
}

bool IAISubscriber::isMotion_() const
{
  return motion_;
}

void IAISubscriber::setMotion_(bool motion_)
{
  IAISubscriber::motion_ = motion_;
}

bool IAISubscriber::isBlur_() const
{
  return blur_;
}

void IAISubscriber::setBlur_(bool blur_)
{
  IAISubscriber::blur_ = blur_;
}

bool IAISubscriber::isSimilar_() const
{
  return similar_;
}

void IAISubscriber::setSimilar_(bool similar_)
{
  IAISubscriber::similar_ = similar_;
}

void IAISubscriber::show(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr img;
  img = cv_bridge::toCvCopy(msg);
  cv::imshow("Image recorded", img->image);
  cv::waitKey(1);
}
void IAISubscriber::show(const sensor_msgs::CompressedImageConstPtr& msg)
{
  cv::namedWindow("Compressed image recorded");
  cv_bridge::CvImageConstPtr img;
  img = cv_bridge::toCvCopy(msg);
  cv::imshow("Compressed image recorded", img->image);
  cv::waitKey(1);
}

string IAISubscriber::addIdentifier(string collection)
{ /**
   switch (mode_)
   {
     case (RAW):
       collection = collection + "_raw" + "_cam_" + std::to_string(cam_);
       break;
     case (COMPRESSED):
       collection = collection + "_compressed" + "_cam_" + std::to_string(cam_);
       break;
     case (THEORA):
       collection = collection + "_theora" + "_cam_" + std::to_string(cam_);
       break;
     case (DEPTH):
       collection = collection + "_depth" + "_cam_" + std::to_string(cam_);
       break;
     case (COMPRESSED_DEPTH):
       collection = collection + "_compressedDepth" + "_cam_" + std::to_string(cam_);
       break;
     default:
       collection = collection;
       break;
   }**/
}
