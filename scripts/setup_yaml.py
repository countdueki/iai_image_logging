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
        "{keyframe_frequency}, {quality}, {depth_max}, {depth_quantization}, {db_host}, {collection}".format(**config))


if __name__ == "__main__":
    rospy.init_node("image_logger_client")

    client = dynamic_reconfigure.client.Client("image_logger", timeout=30, config_callback=callback)

    rate = rospy.Rate(1.0)

    if sys.argv[1] == "-h":
        print("please enter path to yaml file\n")
        exit(0)
    yaml_config = sys.argv[1]


    tsleep_rate = 30;

while not rospy.is_shutdown():

    with open(yaml_config, 'r') as conf_txt:
        for line in conf_txt:
            with open(line.rstrip("\n"), 'r') as stream:
                try:
                    yaml_data = yaml.load(stream)
                    config = iai_image_logging_msgs.cfg.MainConfig.defaults
                    client.update_configuration(yaml_data)
                    time.sleep(tsleep_rate)
                    rate.sleep()

                except yaml.YAMLError as exc:
                    print(exc)
