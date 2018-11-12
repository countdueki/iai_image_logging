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
  tf_sub_.shutdown();
  // delete this;
}

void IAISubscriber::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ros::Rate r(rate_);
  if (motion_)
  {
    motion_detected_ = motion_detector.detectMotion(tf_msg_, tf_msg_prev_, tf_base_, tf_camera_);
    if (motion_detected_)
    {
      ROS_WARN("motion detected");
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
    ROS_DEBUG_STREAM("saving raw image");
    saveImage(msg);
    //pub_.publish(msg);
    // show(msg);
  }
  r.sleep();
}

void IAISubscriber::compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
  ros::Rate r(rate_);
  if (motion_)
  {
    motion_detected_ = motion_detector.detectMotion(tf_msg_, tf_msg_prev_, tf_base_, tf_camera_);
    if (motion_detected_)
    {
      ROS_DEBUG_STREAM("motion detected");
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
    ROS_DEBUG_STREAM("saving compressed image");
    saveCompressedImage(msg);
    //pub_.publish(*msg);
    // show(msg);
  }

  r.sleep();
}

void IAISubscriber::theoraCallback(const theora_image_transport::PacketConstPtr& msg)
{
  ROS_DEBUG_STREAM("saving theora image");
  ros::Rate r(rate_);
  saveTheora(msg);
  r.sleep();
}

void IAISubscriber::tfCallback(const tf::tfMessageConstPtr& msg)
{
  if (prev_tf_)
  {
      tf_msg_prev_ = tf_msg_;
      tf_msg_ = msg;
  }
  else
  {
      tf_msg_ = msg;
      prev_tf_ = true;
  }
}

void IAISubscriber::saveImage(const sensor_msgs::ImageConstPtr& msg)
{
    try{
  mongo::BSONObjBuilder builder;
  string type = ros::message_traits::DataType<sensor_msgs::Image>::value();
  string timestamp = boost::posix_time::to_iso_string(msg->header.stamp.toBoost());
  builder.append("header",
                 BSON("seq" << msg->header.seq << "stamp" << timestamp << "frame_id" << msg->header.frame_id));
  builder.append("encoding", msg->encoding);
  builder.append("width", msg->width);
  builder.append("height", msg->height);
  builder.append("is_bigendian", msg->is_bigendian);
  builder.append("step", msg->step);
  builder.appendBinData("data", msg->data.size(), mongo::BinDataGeneral, &msg->data[0]);

  if (mode_ == RAW)
  {
    builder.append("il_meta", BSON("iai_id" << id_ << "topic" << topic_ << "type" << type << "mode"
                                            << "raw"
                                            << "size" << (int)msg->data.size()));
  }
  else if (mode_ == DEPTH)
  {
    builder.append("il_meta", BSON("iai_id" << id_ << "topic" << topic_ << "type" << type << "mode"
                                            << "depth"
                                            << "size" << (int)msg->data.size()));
  }

  client_connection_->insert(collection_, builder.obj());
    }catch (mongo::UserException e){
        ROS_ERROR_STREAM(e.what());
    }
}

void IAISubscriber::saveCompressedImage(const sensor_msgs::CompressedImageConstPtr& msg)
{
  try
  {
    mongo::BSONObjBuilder builder;
    string type = ros::message_traits::DataType<sensor_msgs::Image>::value();
    string timestamp = boost::posix_time::to_iso_string(msg->header.stamp.toBoost());
    builder.append("header",
                   BSON("seq" << msg->header.seq << "stamp" << timestamp << "frame_id" << msg->header.frame_id));
    builder.append("format", msg->format);
    builder.appendBinData("data", msg->data.size(), mongo::BinDataGeneral, &msg->data[0]);

    if (mode_ == COMPRESSED)
    {
      builder.append("il_meta", BSON("iai_id" << id_ << "topic" << topic_ << "type" << type << "mode"
                                              << "compressed"
                                              << "size" << (int)msg->data.size()));
    }
    else if (mode_ == COMPRESSED_DEPTH)
    {
      builder.append("il_meta", BSON("iai_id" << id_ << "topic" << topic_ << "type" << type << "mode"
                                              << "compressedDepth"
                                              << "size" << (int)msg->data.size()));
    }

    client_connection_->insert(collection_, builder.obj());
  }catch (mongo::UserException e){
      ROS_ERROR_STREAM(e.what());
  }
}

void IAISubscriber::saveTheora(const theora_image_transport::PacketConstPtr& msg)
{
    try {
        mongo::BSONObjBuilder builder;

        string type = ros::message_traits::DataType<theora_image_transport::Packet>::value();
        string timestamp = boost::posix_time::to_iso_string(msg->header.stamp.toBoost());
        builder.append("header",
                       BSON("seq" << msg->header.seq << "stamp" << timestamp << "frame_id" << msg->header.frame_id));
        builder.append("start", msg->b_o_s);
        builder.append("end", msg->e_o_s);
        builder.append("position", (int) msg->granulepos);
        builder.append("packetno", (int) msg->packetno);
        builder.appendBinData("data", msg->data.size(), mongo::BinDataGeneral, &msg->data[0]);

        builder.append("il_meta", BSON("iai_id" << id_ << "topic" << topic_ << "type" << type << "mode"
                                                << "theora"
                                                << "size" << (int) msg->data.size()));

        client_connection_->insert(collection_, builder.obj());
    }catch (mongo::UserException e){
        ROS_ERROR_STREAM(e.what());
    }
}

void IAISubscriber::createSubscriber(int mode)
{
    if (!tf_msg_str_.empty())  tf_sub_ = nh_.subscribe(tf_msg_str_, 1, &IAISubscriber::tfCallback, this);

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
