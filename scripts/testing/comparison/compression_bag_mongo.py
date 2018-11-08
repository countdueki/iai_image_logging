import subprocess
import sys
import time
import os
import signal
import rospy
from iai_image_logging_msgs.srv import *

update = UpdateRequest()
topic_name = "camera__rgb__image_raw"

topic = "/camera/rgb/image_raw"
db_host = "localhost"
jpeg_quality = 100
png_level = 9
format = "jpeg"
mode = "compressed"
collection = "db.one" #""db." + topic_name + "__" + update.mode + "__" + update.format + "__" + "JPEG" + str(update.jpeg_quality) + "__" + "PNG" + str(update.png_level)
tf_msg_str = "/tf"
tf_base = "/whatbase"
tf_cam = "/camtf"
keyframe_frequency = 120
depth_max  = 10.0
depth_quantization = 100.0
target_bitrate = 80000
optimize_for = 1
quality = 34

bag_duration = 10

launcher = subprocess.Popen(["roslaunch",  "iai_image_logging", "kinect_setup.launch"])
time.sleep(5)

    # case 1:

insert_1 = subprocess.Popen(["rosservice", "call", "/iai_configurator/update",
                             "{topic: '"+ topic +"', format: '" + format + "', jpeg_quality: " + str(jpeg_quality)+", png_level: "+ str(png_level) +", db_host: '"+ db_host + "', collection: '"+ collection+"', mode: '"+mode+"'}"])

rosbag_1 =  subprocess.Popen(["rosbag",  "record", topic, "-O", collection,  "--duration=" + str(bag_duration)])
time.sleep(bag_duration)

    #os.kill(insert_1.pid, signal.SIGTERM)
    #os.kill(rosbag_1.pid, signal.SIGTERM)
os.kill(launcher.pid, signal.SIGTERM)
sys.exit(1)

#subprocess.Popen(["rosservice", "call", "/iai_configurator/update",
#                  "{topic: '/camera/rgb/image_raw', format: 'jpeg', iai_id: '', jpeg_quality: 1, png_level: 1, optimize_for: 1, target_bitrate: 80000, keyframe_frequency: 123, quality: 12, depth_max: 12.0, depth_quantization: 122.0, db_host: 'localhost', collection: 'db.check', mode: 'compressed', tf_msg_str: '/tf', tf_base: 'base', tf_cam: 'cam', rate: 10.0, motion: false, blur: false, similar: false}"])