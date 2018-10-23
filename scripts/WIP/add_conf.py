#!/usr/bin/env python


import sys
import rospy
import iai_image_logging_msgs.cfg.MainConfig
import iai_image_logging_msgs.srv._Update

def add_client(update):
    #rospy.wait_for_service('iai_configurator/add')
    try:
        add = rospy.ServiceProxy('iai_configurator/add', iai_image_logging_msgs.srv._Update.UpdateRequest, iai_image_logging_msgs.srv._Update.UpdateResponse)
        result = add(update)
        return result
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    #topic, compression, quality, db_host, collection
    if len(sys.argv) == 2:
        update =iai_image_logging_msgs.srv._Update
        update.UpdateRequest.iai_id = "camera_compressed"
        print (update.UpdateRequest.iai_id)
    else:
        print (usage())
        sys.exit(1)
    #print "Requesting %s+%s"%(x, y)
    #print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))