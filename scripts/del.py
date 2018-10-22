#!/usr/bin/env python

import rospy
import sys
import time
import yaml
import dynamic_reconfigure.client
import iai_image_logging_msgs.cfg.MainConfig




if __name__ == "__main__":
    rospy.init_node("image_logger_client")


    rate = rospy.Rate(1.0)

    if sys.argv[1] == "-h":
        print("enter the commands for a deletion as followed: \n")
        print("TOPIC_NAME\b")
        print("MODE_NUMBER\b")
        print("CAM_NUMBER\b")
        exit(0)

    rospy.Service
    tsleep_rate = 30;

while not rospy.is_shutdown():
    print("logging\n")
    print("topic: " + sys.argv[1] + "\b")
    print("db host: " + sys.argv[2] + "\b")
    print("collection: " + sys.argv[3] + "\b")
    print("mode: " + sys.argv[4] + "\n")
    rate.sleep()



