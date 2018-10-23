//
// Created by tammo on 13.09.18.
//
#include "../include/header/iai_configurator.h"

void IAIConfigurator::updateCamera(iai_image_logging_msgs::UpdateRequest& req, iai_image_logging_msgs::UpdateResponse& res)
{
  ReconfigureRequest req_;
  ReconfigureResponse res_;

  StrParam format;
  IntParam jpeg, png;
  IntParam optimize_for, keyframe_frequency, quality, target_bitrate;

  format.name = "format";
  format.value = req.format;

  jpeg.name = "jpeg_quality";
  jpeg.value = req.jpeg_quality;

  png.name = "png_level";
  png.value = req.png_level;

  optimize_for.name = "optimize_for";
  optimize_for.value = req.optimize_for;

  target_bitrate.name = "target_bitrate";
  target_bitrate.value = req.target_bitrate;

  keyframe_frequency.name = "keyframe_frequency";
  keyframe_frequency.value = req.keyframe_frequency;

  quality.name = "quality";
  quality.value = req.quality;

  /*
   *       DoubleParam depth_max, depth_quantization;

        depth_max.name = "depth_max";
        depth_max.value = req.depth_max;

        depth_quantization.name = "depth_quantization";
        depth_quantization.value = req.depth_quantization;

        req_.config.doubles.push_back(depth_max);
        req_.config.doubles.push_back(depth_quantization);*/

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

/**
       * Service: Updates the requested topic of a 'camera' in a list of 'cameras' (a vector of a
 * mmap<Subscriber,mode>)
       * If the requested topic is not found, it is added. If the camera isn't found, it is added
       * @param req Topic and database info to update
       * @param res success bool
       * @return true, if update was successfull, false else
       */
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
      ROS_DEBUG_STREAM("iai_id: " << (*it)->getID() << ", reqid: " << req.iai_id);
      if ((*it)->getID() == req.iai_id)
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
        return true;
      }
    }
    // add new subscriber
    updateCamera(req,res);
    subs_.push_back(iai_sub);
    ROS_DEBUG_STREAM("size of subs_: " << subs_.size());
    ROS_INFO_STREAM("Added new Subscriber: " << (iai_sub->getID()));
    res.success = true;
    return true;
  }
  catch (std::bad_alloc& ba)
  {
    ROS_ERROR_STREAM("bad_alloc caught: " << ba.what());
    return false;
  }
}

/**
 * Adding a Configuration (i.e. camera + topic and parameters)
 * @param req requested configuration
 * @param res success bool
 * @return true if success, false else
 */
bool IAIConfigurator::addConfig(iai_image_logging_msgs::UpdateRequest& req, iai_image_logging_msgs::UpdateResponse& res)
{
  update(req, res);
  res.success = true;
  return true;
}

/**
* Deleting a Configiguration (i.e. camera + topic)
* @param req requested configuration
* @param res success bool
* @return true if success, false else
*/
bool IAIConfigurator::delConfig(iai_image_logging_msgs::DeleteRequest& req, iai_image_logging_msgs::DeleteResponse& res)
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
        return true;
      }
    }
    ROS_ERROR_STREAM("Error: Subscriber not found");
    return true;
  }
  catch (std::bad_alloc& ba)
  {
    ROS_ERROR_STREAM("bad_alloc caught: " << ba.what());
    return false;
  }
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
 * Start storage node
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
  ros::init(argc, argv, "storage");

  // create storage and initialize
  IAIConfigurator* storage = &IAIConfigurator::Instance();

  while (storage->getNodeHandle().ok())
  {
    for (auto s : storage->getSubscribers())
    {
      s->start();
      ROS_DEBUG_STREAM("Topic: " << s->getTopic());
    }
    ROS_DEBUG_STREAM("storage spinned");
    ros::spinOnce();
  }
  return 0;
}