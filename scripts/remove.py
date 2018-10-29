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
        print(result.success)
        return result.success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def usage():
    print("remove.py IAI_ID")


if __name__ == "__main__":

    if sys.argv[1] == '-h':
        usage()
        sys.exit(1)
    if len(sys.argv) == 2:
        req = RemoveRequest()
        req.iai_id = sys.argv[1]

        remove(req)
        print(req.iai_id + " deleted")
        sys.exit(1)
