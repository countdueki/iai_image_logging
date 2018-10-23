#!/usr/bin/env python


import sys
import rospy
import yaml
from iai_image_logging_msgs.srv import *

def add_client(name):
    rospy.wait_for_service('iai_configurator/add')
    try:
        add = rospy.ServiceProxy('iai_configurator/add', Update)
        result = add(name)
        return result
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)

def usage():
    return "add_conf.py [topic, format, db_host, collection, rate, motion, blur, similar]"

def fill(update):
    update.iai_id = "camera__rgb__image_raw"
    update.topic = "camera/rgb/image_raw"
    update.collection = "iai_image_logging.iai_standard_collection"
    update.db_host = "localhost"
    update.format = "jpeg"
    update.jpeg_quality = 80
    update.png_level = 9
    update.keyframe_frequency = 80000
    update.quality = 31
    update.depth_max = 10.0
    update.depth_quantization = 100.0
    update.rate = 30.0
    update.motion = False
    update.blur = False
    update.similar = False

if __name__ == "__main__":
    name = "hello"

if len(sys.argv) == 1:
        update = UpdateRequest()
        fill(update)
        # update.topic = sys.argv[1]
        # update.format = sys.argv[2]
        # update.db_host = sys.argv[3]
        # update.collection = sys.argv[4]
        # update.rate = sys.argv[5]
        # update.motion = sys.argv[6]
        # update.blur = sys.argv[7]
        # update.similar = sys.argv[8]
        add_client(name)
        print("client updated")

    # if len(sys.argv) == 2:
    #     yaml_config = sys.argv[1]
    #
    #     with open(yaml_config, 'r') as conf_txt:
    #         try:
    #             yaml_data = yaml.load(conf_txt)
    #             update = UpdateRequest(yaml_data)
    #             # TODO load yaml data into update
    #             print (update.collection)
    #
    #         except yaml.YAMLError as exc:
    #             print(exc)
    # if sys.argv[1] == '-h':
    #     print (usage())
    #     sys.exit(1)