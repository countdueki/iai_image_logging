//
// Created by tammo on 25.09.18.
//

#include "theora_nodelet.h"


PLUGINLIB_EXPORT_CLASS(iai_nodelets::TheoraNodelet, nodelet::Nodelet)

namespace iai_nodelets
{

    void TheoraNodelet::onInit()
    {
        ros::NodeHandle nh_;
        ros::Rate hz_rate(60.0);
        NODELET_INFO("Initializing theora nodelet...");

        while(nh_.ok()){
            NODELET_INFO("and theora again...");

            ros::spinOnce();
            hz_rate.sleep();
        }
    }
}