#!/usr/bin/env python


import sys
import rospy
import yaml
from iai_image_logging_msgs.srv import *


def change(behavior):
    rospy.wait_for_service('iai_configurator/behave')
    try:
        behave = rospy.ServiceProxy('iai_configurator/behave', Behave)
        result = behave(behavior)
        print(result.success)
        return result.success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def usage():
    print("behavior.py iai_id=IAI_ID\n"
          "id=IAI_ID\n"
          "rate=rate(double)\n"
          "motion=true/false\n"
          "blur=true/false\n"
          "similar=true/false\n")


if __name__ == "__main__":

    behavior = BehaveRequest()
    behavior.rate = 3.0  # standard value

    if sys.argv[1] == '-h':
        usage()
        sys.exit(1)

    for argument in sys.argv:
        if argument.startswith('id='):
            iai_id = argument[3:]
            behavior.iai_id = iai_id
        if argument.startswith('rate='):
            rate = float(argument[5:])
            behavior.rate = rate
        if argument.startswith('motion='):
            if (argument[7:] == 'True') | (argument[7:] == 'true'):
                motion = True
            else:
                motion = False
            behavior.motion = motion
        if argument.startswith('blur='):
            if (argument[5:] == 'True') | (argument[5:] == 'true'):
                blur = True
            else:
                blur = False
            behavior.blur = blur
        if argument.startswith('similar='):
            if (argument[8:] == 'True') | (argument[8:] == 'true'):
                similar = True
            else:
                similar = False
            behavior.similar = similar

    if len(sys.argv) == 2:
        yaml_config = sys.argv[1]

        with open(yaml_config, 'r') as conf_txt:
            try:
                yaml_data = yaml.load(conf_txt)
                behavior.iai_id = yaml_data["iai_id"]
                behavior.rate = yaml_data["rate"]
                behavior.similar = yaml_data["similar"]
                behavior.blur = yaml_data["blur"]
                behavior.motion = yaml_data["motion"]
                print(yaml_data["iai_id"])
            except yaml.YAMLError as exc:
                print(exc)
    change(behavior)
    print("behavior updated")

    if sys.argv[1] == '-h':
        print(usage())
        sys.exit(1)
