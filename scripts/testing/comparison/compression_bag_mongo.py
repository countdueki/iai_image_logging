import subprocess
import sys
import time
import os
import signal
import rospy
from iai_image_logging_msgs.srv import *

update = UpdateRequest()
topic_name = "camera__rgb__image_raw"

update.topic = "/camera/rgb/image_raw"
update.db_host = "localhost"
update.jpeg_quality = 100
update.png_level = 9
update.format = "jpeg"
update.mode = "compressed"
update.collection = "db.one" #""db." + topic_name + "__" + update.mode + "__" + update.format + "__" + "JPEG" + str(update.jpeg_quality) + "__" + "PNG" + str(update.png_level)
update.tf_msg_str = "/tf"
update.tf_base = "/whatbase"
update.tf_cam = "/camtf"
update.keyframe_frequency = 120
update.depth_max  = 10.0
update.depth_quantization = 100.0
update.target_bitrate = 80000
update.optimize_for = 1
update.quality = 34

bag_duration = 10

if __name__ == "__main__":
    launcher = subprocess.Popen(["roslaunch",  "iai_image_logging", "kinect_setup.launch"])
    time.sleep(5)

    # case 1:

    rospy.wait_for_service('iai_configurator/update')
    try:
        insertion = rospy.ServiceProxy('iai_configurator/update', Update)
        result = insertion(update)
        print(result)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

    #insert_1 = subprocess.Popen(["rosservice", "call", "/iai_configurator/update",
    #                             "{'topic':'" + topic + "', 'iai_id':'"+topic_name + "', 'format':'" + format + "' , 'jpeg_quality': " + str(jpeg_quality) + ", 'collection':'db." + collection + "' , 'mode':'" + mode + "', 'db_host':'localhost'}"])
    #rosbag_1 =  subprocess.Popen(["rosbag",  "record", topic, "-O", collection,  "--duration=" + str(bag_duration)])
    time.sleep(bag_duration)

    #os.kill(insert_1.pid, signal.SIGTERM)
    #os.kill(rosbag_1.pid, signal.SIGTERM)
    os.kill(launcher.pid, signal.SIGTERM)
    sys.exit(1)

    # rosrun dynamic_reconfigure dynparam set /iai_updater "{'topic':'/camera/rgb/image_raw', 'format':'jpeg', 'jpeg_quality':100, 'png_level':9, 'db_host':'localhost', 'collection':'db.standard','optimize_for': 0, 'target_bitrate': 800000, 'keyframe_frequency':64, 'quality': 31, 'depth_max':10.0, 'depth_quantization':100.0, 'mode':0, 'cam_no':0}"