#!/usr/bin/env python

import rospy
import sys
import time
import yaml
import dynamic_reconfigure.client
import iai_image_logging_msgs.cfg.MainConfig



def callback(config):
    rospy.loginfo(
        "Config set to {topic}, {optimize_for}, {target_bitrate}, {format} {png_level}, {jpeg_quality}"
        "{keyframe_frequency}, {quality}, {depth_max}, {depth_quantization}, {db_host}, {collection} {mode}".format(**config))


if __name__ == "__main__":
    rospy.init_node("image_logger_client")

    client = dynamic_reconfigure.client.Client("image_logger", timeout=30, config_callback=callback)

    rate = rospy.Rate(1.0)

    if sys.argv[1] == "-h":
        print("enter the commands for a setup as followed: \n")
        print("TOPIC_NAME\b")
        print("HOST_NAME\b")
        print("COLLECTION_NAME\b")
        print("[raw,compressed,theora,depth,compressed_depth]\n")
        exit(0)

    config = iai_image_logging_msgs.cfg.MainConfig.defaults
    print(config)
    if str(sys.argv[4]) == "raw":
        mode = 0
    if sys.argv[4] == "depth":
        mode = 3
    if sys.argv[4] == "compressed":
        mode = 1
    if sys.argv[4] == "compressed_depth":
        mode = 4
    if sys.argv[4] == "theora":
        mode = 2

    client.update_configuration({'topic': sys.argv[1], 'db_host': sys.argv[2], 'collection': sys.argv[3],
                                'mode': mode})
    tsleep_rate = 30;

while not rospy.is_shutdown():
    print("logging\n")
    print("topic: " + sys.argv[1] + "\b")
    print("db host: " + sys.argv[2] + "\b")
    print("collection: " + sys.argv[3] + "\b")
    print("mode: " + sys.argv[4] + "\n")
    rate.sleep()



