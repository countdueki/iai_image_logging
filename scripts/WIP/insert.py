#!/usr/bin/env python


import sys
import argparse
import rospy
import yaml
from iai_image_logging_msgs.srv import *

topic = "camera/rgb/image_raw"
mode = "raw"
quality = "good"
rate = 30.0
motion = False
blur = False
similar = False
collection = "db.standard"
host = "localhost"

def insert_client(insert):
    rospy.wait_for_service('iai_configurator/insert')
    try:
        insertion = rospy.ServiceProxy('iai_configurator/insert', Insert)
        result = insertion(insert)
        return result
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)

def usage():
    print("insert.py [topic=base_topic\n" 
          "mode=raw/compressed/compressedDepth/theora\n"
          "quality=good/medium/base\n"
          "rate=rate(double)\n"
          "motion=true/false\n"
          "blur=true/false\n"
          "similar=true/false\n"
          "collection=db_name.collection\n"
          "db_host=host\n")


def fill(insert):
    insert.topic =      topic
    insert.mode =       mode
    insert.rate =       rate
    insert.motion =     bool(motion)
    insert.blur =       bool(blur)
    insert.similar =    bool(similar)
    insert.collection = collection
    insert.db_host =    host

if __name__ == "__main__":
    # parser = argparse.ArgumentParser()
    # args = parser.parse_args()

    insert = InsertRequest()


    if sys.argv[1] == '-h':
        usage()
        sys.exit(1);

    for argument in sys.argv:
        if argument.startswith('topic='):
            topic = argument[5:]
        if argument.startswith('mode='):
            mode = argument[4:]
        if argument.startswith('quality='):
            quality = argument[4:]
        if argument.startswith('rate='):
            rate = argument[4:]
        if argument.startswith('motion='):
            motion = argument[6:]
        if argument.startswith('blur='):
            blur = argument[4:]
        if argument.startswith('similar='):
            similar = argument[7:]
        if argument.startswith('collection='):
            collection = argument[9:]
        if argument.startswith('db_host='):
            host = argument[7:]

    fill(insert)
    insert_client(insert)

    print("client updated")
    sys.exit(1)

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