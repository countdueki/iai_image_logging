#!/usr/bin/env python

import time

import dynamic_reconfigure.client
import iai_image_logging_msgs.cfg.MainConfig
import rospy

import yaml


def callback(config):
    rospy.loginfo(
        "Config set to {topic}, {png_level}, "
        "{jpeg_quality},  {format}, {db_host}, {collection}".format(**config))


if __name__ == "__main__":
    rospy.init_node("image_logger_client")

    client = dynamic_reconfigure.client.Client(
        "image_logger", timeout=30, config_callback=callback)

    rate = rospy.Rate(1.0)

    yaml_list = "../yaml/matrix/compressed_config.txt"

    tsleep_rate = 1;

while not rospy.is_shutdown():

    with open(yaml_list, 'r') as conf_txt:
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