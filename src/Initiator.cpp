//
// Created by tammo on 07.09.18.
//

#include "Initiator.h"


int main(int argc, char** argv)
{
    ros::init(argc,argv,"initiator");
    ros::NodeHandle nh;


    ros::Rate r(1.0);

    while (nh.ok())
    {

            ROS_INFO_STREAM("Successfully called service");
        ros::spinOnce();
        r.sleep();
    }
}
}