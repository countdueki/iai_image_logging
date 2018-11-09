/**
 * Created by Tammo Wübbena on 13/09/2018
 * All documentation information is found in the header file (except for @function main).
 */

#include "../include/header/iai_configurator.h"

bool IAIConfigurator::update(iai_image_logging_msgs::UpdateRequest& req, iai_image_logging_msgs::UpdateResponse& res)
{
  try
  {
    db_host_ = req.db_host;
    string cur_topic, req_topic;

    IAISubscriber* iai_sub = new IAISubscriber(*client_connection_, req);
    // iterate over camera list for existing entries
    for (auto it = subs_.begin(); it != subs_.end(); ++it)
    {
      ROS_DEBUG_STREAM("checking sub: " << (*it)->getID());
      ROS_WARN_STREAM("iai_id: " << (*it)->getID() << " with topic: " << iai_sub->getTopic()
                                 << ", reqid: " << iai_sub->getID());
      if ((*it)->getID() == iai_sub->getID())
      {
        // shutdown and delete old subscriber
        ROS_DEBUG_STREAM("...Found topic, deleting...");
        (*it)->destroy();
        subs_.erase(it);
        ROS_DEBUG_STREAM("...update camera topic...");
        updateCamera(req, res);
        ROS_DEBUG_STREAM("...add new");

        // add updated Subscriber
        subs_.push_back(iai_sub);
        ROS_INFO_STREAM("Updated Subscriber: " << iai_sub->getID());
        res.success = true;
        return true;
      }
    }
    // add new subscriber
    updateCamera(req, res);
    subs_.push_back(iai_sub);
    ROS_DEBUG_STREAM("size of subs_: " << subs_.size());
    ROS_INFO_STREAM("Added new Subscriber: " << (iai_sub->getID()));
    res.success = true;
    return true;
  }
  catch (std::bad_alloc& ba)
  {
    ROS_ERROR_STREAM("bad_alloc caught: " << ba.what());
    res.success = false;
    return false;
  }
}

bool IAIConfigurator::insert(iai_image_logging_msgs::InsertRequest& req, iai_image_logging_msgs::InsertResponse& res)
{
  iai_image_logging_msgs::UpdateRequest ureq;
  iai_image_logging_msgs::UpdateResponse ures;

  insertionConfigurator(req, res, ureq, ures);
  ROS_WARN_STREAM("configured insertion");
  update(ureq, ures);
  res.success = true;
  return true;
}

void IAIConfigurator::insertionConfigurator(iai_image_logging_msgs::InsertRequest& req,
                                            iai_image_logging_msgs::InsertResponse& res,
                                            iai_image_logging_msgs::UpdateRequest& ureq,
                                            iai_image_logging_msgs::UpdateResponse& ures)
{
  ROS_WARN_STREAM("Parameters: " << req.rate << req.topic << req.collection);
  // calculate quality
  switch (getNumberFromModeString(req.mode))
  {
    case (RAW):
      ureq.mode = "raw";
      break;
    case (DEPTH):
      ureq.mode = "depth";
      break;
    case (COMPRESSED):
      ureq.mode = "compressed";
      if (req.quality == "good")
      {
        ureq.jpeg_quality = 100;
        ureq.png_level = 1;
      }
      else if (req.quality == "medium")
      {
        ureq.jpeg_quality = 80;
        ureq.png_level = 5;
      }
      else if (req.quality == "base")
      {
        ureq.jpeg_quality = 40;
        ureq.png_level = 9;
      }
      else
      {
        ROS_ERROR("Wrong quality parameter");
      }
      break;
    case (THEORA):
      ureq.mode = "theora";
      if (req.quality == "good")
      {
        ureq.quality = 63;
      }
      else if (req.quality == "medium")
      {
        ureq.quality = 48;
      }
      else if (req.quality == "base")
      {
        ureq.quality = 20;
      }
      else
      {
        ROS_ERROR("Wrong quality parameter");
      }
      break;

    case (COMPRESSED_DEPTH):
      ureq.mode = "compressedDepth";
      if (req.quality == "good")
      {
        ureq.jpeg_quality = 100;
        ureq.png_level = 1;
      }
      else if (req.quality == "medium")
      {
        ureq.jpeg_quality = 80;
        ureq.png_level = 5;
      }
      else if (req.quality == "base")
      {
        ureq.jpeg_quality = 40;
        ureq.png_level = 9;
      }
      else
      {
        ROS_ERROR("Wrong quality parameter");
      }
      break;
  }
  // update subscriber
  ureq.topic = req.topic;
  ureq.collection = req.collection;
  ureq.db_host = req.db_host;
  ureq.motion = req.motion;
  ureq.blur = req.blur;
  ureq.similar = req.similar;
  ureq.rate = req.rate;
  ureq.format = "jpeg";
  ureq.tf_msg_str = req.tf_msg_str;
  ureq.tf_base = req.tf_base;
  ureq.tf_cam = req.tf_cam;

  update(ureq, ures);

  ROS_WARN_STREAM("Insertion done");
}

bool IAIConfigurator::remove(iai_image_logging_msgs::RemoveRequest& req, iai_image_logging_msgs::RemoveResponse& res)
{
  try
  {
    // iterate over camera list for existing entries
    for (auto it = subs_.begin(); it != subs_.end(); ++it)
    {
      ROS_DEBUG_STREAM("checking sub: " << (*it)->getID());
      ROS_DEBUG_STREAM("iai_id: " << (*it)->getID() << ", reqid: " << req.iai_id);
      if ((*it)->getID() == req.iai_id)
      {
        // shutdown and delete old subscriber
        ROS_DEBUG_STREAM("...Found topic, deleting...");
        (*it)->destroy();
        subs_.erase(it);
        ROS_INFO_STREAM("Deleted Subscriber: " << (*it)->getID());
        res.success = true;
        return true;
      }
    }
    ROS_ERROR_STREAM("Error: Subscriber not found");
    res.success = true;
    return true;
  }
  catch (std::bad_alloc& ba)
  {
    ROS_ERROR_STREAM("bad_alloc caught: " << ba.what());
    res.success = false;
    return false;
  }
}

bool IAIConfigurator::behave(iai_image_logging_msgs::BehaveRequest& req, iai_image_logging_msgs::BehaveResponse& res)
{
  for (auto it = subs_.begin(); it != subs_.end(); ++it)
  {
    if ((*it)->getID() == req.iai_id)
    {
      (*it)->setRate_(req.rate);
      (*it)->setMotion_(req.motion);
      (*it)->setBlur_(req.blur);
      (*it)->setSimilar_(req.similar);
      ROS_WARN_STREAM("Set options for Sub with IAI_ID " << req.iai_id << " to");
      ROS_WARN_STREAM("Motion: " << std::to_string((*it)->isMotion_()));
      ROS_WARN_STREAM("Blur: " << std::to_string((*it)->isBlur_()));
      ROS_WARN_STREAM("Similar: " << std::to_string((*it)->isSimilar_()));

      res.success = true;
      return true;
    }
  }

  ROS_WARN_STREAM("No Subscriber with this ID has been found");
  res.success = false;
  return false;
}

void IAIConfigurator::updateCamera(iai_image_logging_msgs::UpdateRequest& req,
                                   iai_image_logging_msgs::UpdateResponse& res)
{
  ReconfigureRequest req_;
  ReconfigureResponse res_;

  StrParam topic, collection, db_host, format;
  IntParam jpeg, png, mode;
  IntParam optimize_for, keyframe_frequency, quality, target_bitrate;

  topic.name = "topic";
  topic.value = req.topic;

  collection.name = "collection";
  collection.value = req.collection;

  db_host.name = "db_host";
  db_host.value = req.db_host;

  format.name = "format";
  format.value = req.format;

  jpeg.name = "jpeg_quality";
  jpeg.value = req.jpeg_quality;

  png.name = "png_level";
  png.value = req.png_level;

  mode.name = "mode";
  mode.value = getNumberFromModeString(req.mode);

  optimize_for.name = "optimize_for";
  optimize_for.value = req.optimize_for;

  target_bitrate.name = "target_bitrate";
  target_bitrate.value = req.target_bitrate;

  keyframe_frequency.name = "keyframe_frequency";
  keyframe_frequency.value = req.keyframe_frequency;

  quality.name = "quality";
  quality.value = req.quality;

  DoubleParam depth_max, depth_quantization;

  depth_max.name = "depth_max";
  depth_max.value = req.depth_max;

  depth_quantization.name = "depth_quantization";
  depth_quantization.value = req.depth_quantization;

  // Set general parameters
    req_.config.strs.push_back(topic);
    req_.config.strs.push_back(collection);
    req_.config.strs.push_back(db_host);
    req_.config.ints.push_back(mode);


    // Set parameters for depth images
  req_.config.doubles.push_back(depth_max);
  req_.config.doubles.push_back(depth_quantization);

  // Set parameters for compressed images
  req_.config.strs.push_back(format);
  req_.config.ints.push_back(jpeg);
  req_.config.ints.push_back(png);

  // Set parameters for theora video
  req_.config.ints.push_back(optimize_for);
  req_.config.ints.push_back(keyframe_frequency);
  req_.config.ints.push_back(quality);
  req_.config.ints.push_back(target_bitrate);

  ros::service::call("/iai_updater/set_parameters", req_, res_);  // WORKS!
  // ros::service::call("image_logger/updateCamera",req,res);
}

int IAIConfigurator::getNumberFromModeString(string mode)
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

const NodeHandle& IAIConfigurator::getNodeHandle() const
{
  return nh_;
}

const StorageSubVector& IAIConfigurator::getSubscribers() const
{
  return subs_;
}

/**
 * Starts the configuration node and sets up logger levels. The spin constantly checks for new subscribers
 * and starts them eventually
 * @param argc
 * @param argv
 * @return 0 on normal exit
 */
int main(int argc, char** argv)
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
  ros::init(argc, argv, "iai_configurator");

  // create storage and initialize
  IAIConfigurator* storage = &IAIConfigurator::Instance();

  while (storage->getNodeHandle().ok())
  {
    for (auto s : storage->getSubscribers())
    {
      s->start();
      ROS_DEBUG_STREAM("Topic: " << s->getID());
    }
    ROS_DEBUG_STREAM("storage spinned");
    ros::spinOnce();
  }
  return 0;
}