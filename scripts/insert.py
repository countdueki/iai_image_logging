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
tf_msg_str = "tf_static"
tf_base = "map"
tf_cam = "head_mount_kinect_rgb_optical_frame"


def insert_client(insert):
    rospy.wait_for_service('iai_configurator/insert')
    try:
        insertion = rospy.ServiceProxy('iai_configurator/insert', Insert)
        result = insertion(insert)
        return result
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def usage():
    print("insert.py [topic=base_topic\n"
          "mode=raw/compressed/compressedDepth/theora\n"
          "quality=good/medium/base\n"
          "rate=rate(double)\n"
          "motion=true/false\n"
          "blur=true/false\n"
          "similar=true/false\n"
          "collection=db_name.collection\n"
          "db_host=host\n"
          "tf=TF_TOPIC\n"
          "tf_base=TF_BASE\n"
          "tf_cam=TF_CAMERA\n")


def fill(insert):
    insert.topic = topic
    insert.mode = mode
    insert.rate = rate
    insert.motion = motion
    insert.blur = blur
    insert.similar = similar
    insert.collection = collection
    insert.db_host = host
    insert.tf_msg_str = tf_msg_str
    insert.tf_base = tf_base
    insert.tf_cam = tf_cam



if __name__ == "__main__":
    # parser = argparse.ArgumentParser()
    # args = parser.parse_args()

    insert = InsertRequest()
    fill(insert)

    if sys.argv[1] == '-h':
        usage()
        sys.exit(1);

    for argument in sys.argv:
        if argument.startswith('topic='):
            topic = argument[6:]
            insert.topic = topic
        if argument.startswith('mode='):
            mode = argument[5:]
            insert.mode = mode
        if argument.startswith('quality='):
            quality = argument[8:]
            insert.quality = quality
        if argument.startswith('rate='):
            rate = float(argument[5:])
            insert.rate = rate
        if argument.startswith('motion='):
            if (argument[7:] == 'True') | (argument[7:] == 'true'):
                motion = True
            else:
                motion = False
            insert.motion = motion
        if argument.startswith('blur='):
            if (argument[5:] == 'True') | (argument[5:] == 'true'):
                blur = True
            else:
                blur = False
            insert.blur = blur
        if argument.startswith('similar='):
            if (argument[8:] == 'True') | (argument[8:] == 'true'):
                similar = True
            else:
                similar = False
            insert.similar = similar
        if argument.startswith('collection='):
            collection = argument[11:]
            insert.collection = collection
        if argument.startswith('db_host='):
            host = argument[8:]
            insert.db_host = host
        if argument.startswith('tf='):
            tf_msg_str = argument[3:]
            insert.tf_msg_str = tf_msg_str
        if argument.startswith('tf_base='):
            tf_base = argument[8:]
            insert.tf_base = tf_base
        if argument.startswith('tf_cam='):
            tf_cam = argument[7:]
            insert.tf_cam = tf_cam

    if len(sys.argv) == 2:
        yaml_config = sys.argv[1]

        with open(yaml_config, 'r') as conf_txt:
            try:
                yaml_data = yaml.load(conf_txt)
                insert.topic = yaml_data["topic"]
                insert.quality = yaml_data["quality"]
                insert.collection = yaml_data["collection"]
                insert.db_host = yaml_data["db_host"]
                insert.rate = yaml_data["rate"]
                insert.similar = yaml_data["similar"]
                insert.blur = yaml_data["blur"]
                insert.motion = yaml_data["motion"]
                insert.tf_msg_str = yaml_data["tf_msg_str"]
                insert.tf_base = yaml_data["tf_base"]
                insert.tf_cam = yaml_data["tf_cam"]
            except yaml.YAMLError as exc:
                print(exc)
    insert_client(insert)

    print("client updated")
    sys.exit(1)
