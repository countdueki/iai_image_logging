#!/usr/bin/env python

import rospy

import dynamic_reconfigure.client
import iai_image_logging_msgs.cfg.DefaultConfig


def callback(config):
    rospy.loginfo("Config set to {int_param}, {double_param}, {str_param}, {bool_param}, {size}".format(**config))


if __name__ == "__main__":
    rospy.init_node("image_logging_client")

    client = dynamic_reconfigure.client.Client("image_logger", timeout=30, config_callback=callback)

    r = rospy.Rate(0.1)
    i = 0
    d = 0.0
    config = iai_image_logging_msgs.cfg.DefaultConfig.defaults
    b = False
    while not rospy.is_shutdown():

      client.update_configuration({"int_param": i, "double_param": d, "str_param":str(rospy.get_rostime()), "bool_param": b, "size": 1})
      r.sleep()
