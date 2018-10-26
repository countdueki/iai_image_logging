#!/usr/bin/env python


import sys
import rospy
import yaml
from iai_image_logging_msgs.srv import *

def remove(req):
    rospy.wait_for_service('iai_configurator/remove')
    try:
        removal = rospy.ServiceProxy('iai_configurator/remove', Remove)
        result = removal(req)
        print (result.success)
        return result.success
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)

def usage():
    return "behave.py [iai_id, rate, motion, blur, similar]"


if __name__ == "__main__":

    if len(sys.argv) == 2:
        req = RemoveRequest()
        req.iai_id = sys.argv[1]

        remove(req)
        print(req.iai_id + "deleted")

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