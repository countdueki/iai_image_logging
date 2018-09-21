//
// Created by tammo on 18.09.18.
//

#ifndef IAI_IMAGE_LOGGING_CAMERA_H
#define IAI_IMAGE_LOGGING_CAMERA_H

#include <ros/ros.h>
using std::vector;
using ros::Subscriber;

enum
{
  RAW,
  COMPRESSED,
  THEORA,
  DEPTH,
  COMPRESSED_DEPTH
};

class Camera

{
public:
  Camera()
  {
  }

  Camera(int cam_no)
  {
    cam_no_ = cam_no;
    is_empty = false;
  }

  ~Camera()
  {
  }

private:
  std::set<Subscriber> sub_raw_list;
  std::set<Subscriber> sub_compressed_list;
  std::set<Subscriber> sub_theora_list;
  bool is_empty;
  int cam_no_;

public:
  int getNo() const
  {
    return cam_no_;
  }

public:
  void init(Subscriber sub_raw, Subscriber sub_compressed, Subscriber sub_theora)
  {
    sub_raw_list.insert(sub_raw);
    sub_compressed_list.insert(sub_compressed);
    sub_theora_list.insert(sub_theora);
  }

  /*  void update(Subscriber sub_, Subscriber sub_, Subscriber sub_)
    {
      sub_raw_list.insert(sub_);
      sub_compressed_list.insert(sub_);
      sub_theora_list.insert(sub_);
    }*/

  void updateRaw(Subscriber new_sub)
  {
    int index = 0;
    bool found;
    for (Subscriber sub : sub_raw_list)
    {
      if (sub.getTopic() == new_sub.getTopic())
      {
        sub_raw_list.erase(sub);
        sub_raw_list.insert(new_sub);
        found = true;
      }
      index++;
    }
    if (!found)
    {
      sub_raw_list.insert(new_sub);
    }
  }
  void updateCompressed(Subscriber new_sub)
  {

    ROS_WARN_STREAM("got to update compressed");
    bool found = false;
    ROS_WARN_STREAM("Size of list " << sub_compressed_list.size());
    if (!sub_compressed_list.empty())
    {
      ROS_WARN_STREAM("compressed list not empty");

      for (const Subscriber& sub : sub_compressed_list)
      {
        ROS_WARN_STREAM("Got in loop");

        ROS_WARN_STREAM("New sub: " << new_sub.getTopic());
        ROS_WARN_STREAM("Sub: " << sub.getTopic());

        if (sub.getTopic() == new_sub.getTopic())
        {
          ROS_WARN_STREAM("in if in loop");

          ROS_WARN_STREAM("got to update new sub");

          sub_compressed_list.erase(sub);
          sub_compressed_list.insert(new_sub);
          found = true;
        }
      }
    }
    if (!found)
    {
      ROS_WARN_STREAM("landed in adding new sub");

      sub_compressed_list.insert(new_sub);
    }
  }
  void updateTheora(Subscriber new_sub)
  {
    int index = 0;
    bool found;
    for (Subscriber sub : sub_theora_list)
    {
      if (sub.getTopic() == new_sub.getTopic())
      {
        sub_theora_list.erase(sub);
        sub_theora_list.insert(new_sub);
        found = true;
      }
      index++;
    }
    if (!found)
    {
      sub_theora_list.insert(new_sub);
    }
  }

  bool isEmpty()
  {
    return is_empty;
  }
  void print()
  {
    ROS_WARN_STREAM("I am Camera Number" << cam_no_);
  }

  // currently not used. Update used instead
  /*  void add(iai_image_logging_msgs::UpdateRequest& req)
    {

      int index = 0;
      bool found;
      if (req.mode == RAW)
      {
        for (Subscriber sub : sub_raw_list)
        {
          if (sub.getTopic() == new_sub.getTopic())
          {
            sub_raw_list.at(index) = new_sub;
            found = true;
          }
          index++;
        }
        if (!found)
        {
          sub_raw_list.insert(new_sub);
        }
      }
      else if (req.mode == COMPRESSED || req.mode == COMPRESSED_DEPTH)
      {
        for (Subscriber sub : sub_compressed_list)
        {
          if (sub.getTopic() == new_sub.getTopic())
          {
            ROS_WARN_STREAM("got to adding new sub");

            sub_theora_list.at(index) = new_sub;
            found = true;
          }
          index++;
        }
        if (!found)
        {
          sub_compressed_list.insert(new_sub);
        }

      }
      else if ( req.mode == THEORA)
      {
        for (Subscriber sub : sub_theora_list)
        {
          if (sub.getTopic() == new_sub.getTopic())
          {
            sub_theora_list.at(index) = new_sub;
            found = true;
          }
          index++;
        }
        if (!found)
        {
          sub_theora_list.insert(new_sub);
        }
      }
    }*/

  void del(iai_image_logging_msgs::UpdateRequest& req)
  {
    int index = 0;
    bool found;
    if (req.mode == RAW)
    {
      for (Subscriber sub : sub_raw_list)
      {
        if (sub.getTopic() == req.topic)
        {
          sub_raw_list.erase(sub);
          found = true;
        }
        index++;
      }
      if (!found)
      {
        ROS_ERROR_STREAM("Subscriber not found");
      }
    }
    else if (req.mode == COMPRESSED || req.mode == COMPRESSED_DEPTH)
    {
      for (Subscriber sub : sub_compressed_list)
      {
        if (sub.getTopic() == req.topic)
        {
          sub_compressed_list.erase(sub);
          found = true;
        }
        index++;
      }
      if (!found)
      {
        ROS_ERROR_STREAM("Subscriber not found");
      }
    }
    else if (req.mode == THEORA)
    {
      for (Subscriber sub : sub_theora_list)
      {
        if (sub.getTopic() == req.topic)
        {
          sub_theora_list.erase(sub);
          found = true;
        }
        index++;
      }
      if (!found)
      {
        ROS_ERROR_STREAM("Subscriber not found");
      }
    }
  }
};

class CompositeCamera : public Camera
{
public:
  void print() const
  {
    for (Camera* cam : camera_list_)
    {
      cam->print();
    }
  }

  void add(Camera* cam)
  {
    camera_list_.insert(cam);
    size_++;
    ROS_WARN_STREAM("CAMERA ADDED!");
  }
  size_t size()
  {
    return size_;
  }

  Camera* at(int index)
  {
    for (Camera* cam : camera_list_)
    {
      if (index == cam->getNo())
        return cam;
    }
  }

private:
  std::set<Camera*> camera_list_;
  size_t size_ = 0;
};

#endif  // IAI_IMAGE_LOGGING_CAMERA_H
