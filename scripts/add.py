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

    if sys.argv[1] == "-h":
        print("enter the commands for a setup as followed: \n")
        print("TOPIC_NAME")
        print("HOST_NAME")
        print("COLLECTION_NAME")
        print("[raw,compressed,theora,depth,compressed_depth]")
        print("[png,jpeg,theora_video]")
        print("[high,low]")
        print("CAM_NO")
        exit(0)

    config = iai_image_logging_msgs.cfg.MainConfig.defaults
    #print(config)
    mode = 0
    format = "jpeg"
    jpeg_quality = 80
    theora_quality = 50
    cam_no = 0
    if str(sys.argv[4]) == "raw":
        mode = 0

    if sys.argv[4] == "depth":
        mode = 3

    if sys.argv[4] == "compressed":
        mode = 1
        if sys.argv[5] == "jpeg":
            format = "jpeg"
            if sys.argv[6] == "high":
                jpeg_quality = 100
            if sys.argv[6] == "low":
                jpeg_quality = 20
        if sys.argv[5] == "png":
            format = "png"

    if sys.argv[4] == "compressed_depth":
        mode = 4
        format = "jpeg"
        if sys.argv[5] == "jpeg":
            format = "jpeg"
            if sys.argv[6] == "high":
                jpeg_quality = 100
            if sys.argv[6] == "low":
                jpeg_quality = 20
        if sys.argv[5] == "png":
            format = "png"

    if str(sys.argv[4]) == "theora":
        print("logging theora")
        mode = 2
        if sys.argv[5] == "theora_video":
            if sys.argv[6] == "high":
                theora_quality = 63.0
            if sys.argv[6] == "low":
                theora_quality = 18.0

    cam_no = sys.argv[7]
    client.update_configuration({'topic': sys.argv[1], 'db_host': sys.argv[2], 'collection': sys.argv[3],
                                'mode': mode, 'png_level' : 1, 'jpeg_quality': jpeg_quality,
                                 'quality': theora_quality, 'format': format, 'cam_no': cam_no})

    print("LOGGING:\n")
    print("topic: " + sys.argv[1])
    print("db host: " + sys.argv[2])
    print("collection: " + sys.argv[3])
    print("mode: " + sys.argv[4])
    print("compression: " + sys.argv[5])
    print("image_quality: " + sys.argv[6] + "\n")



