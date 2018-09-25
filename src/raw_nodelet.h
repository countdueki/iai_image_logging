//
// Created by tammo on 25.09.18.
//

#ifndef IAI_IMAGE_LOGGING_RAW_NODELET_H
#define IAI_IMAGE_LOGGING_RAW_NODELET_H
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
namespace iai_nodelets {

    class RawNodelet : public nodelet::Nodelet {

    public:
        virtual void onInit();

    };
}

#endif //IAI_IMAGE_LOGGING_RAW_NODELET_H
