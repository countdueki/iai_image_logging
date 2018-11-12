import subprocess
import sys
import time
import os
import signal
import rospy
from iai_image_logging_msgs.srv import *

update = UpdateRequest()
topic_name = "camera__rgb__image_color"

topic = "/camera/rgb/image_color"
jpeg_quality = 100
format_comp = "jpeg"
png_level = 9
mode = "theora"
db_host = "localhost"
quality = 63
collection = "db." + topic_name + "__" + mode + "__" + format_comp + "__" + "JPEG" + str(jpeg_quality) + "__" + "PNG" + str(png_level) + "__" + "THEO" + str(quality)
tf_msg_str = "/tf"
tf_base = "/whatbase"
tf_cam = "/camtf"
keyframe_frequency = 64
depth_max  = 10.0
depth_quantization = 100.0
target_bitrate = 800000
optimize_for = 1

bag_duration = 10

launcher = subprocess.Popen(["roslaunch",  "iai_image_logging", "kinect_setup.launch"])
time.sleep(5)

# set all

#subprocess.Popen(["rosservice", "call", "/iai_configurator/update",
#                  "{topic: '/camera/rgb/image_color', db_host: 'localhost', collection: 'db.check', mode: 'theora'}"])

time.sleep(2)
print("OK, starting...")


# jpeg cases
print ("Executing jpeg cases")
format_comp = "jpeg"
mode = "compressed"

# jpeg case 1:
jpeg_quality = 100
collection = "db." + topic_name + "__" + mode + "__" + "JPEG" + str(jpeg_quality)

insert_j1 = subprocess.Popen(["rosservice", "call", "/iai_configurator/update",
                             "{rate: 30, topic: '" + topic +"', format: '" + format_comp + "', jpeg_quality: " + str(jpeg_quality) + ", db_host: '" + db_host + "', collection: '" + collection + "', mode: '" + mode + "'}"])


rosbag_j1 =  subprocess.Popen(["rosbag",  "record", topic +"/"+mode, "-O", "bag/" +collection[3:],  "--duration=" + str(bag_duration)])
time.sleep(bag_duration)

# jpeg case 2:
jpeg_quality = 80
collection = "db." + topic_name + "__" + mode + "__" + "JPEG" + str(jpeg_quality)

insert_j2 = subprocess.Popen(["rosservice", "call", "/iai_configurator/update",
                             "{topic: '" + topic +"', format: '" + format_comp + "', jpeg_quality: " + str(jpeg_quality) + ", db_host: '" + db_host + "', collection: '" + collection + "', mode: '" + mode + "'}"])

rosbag_j2 =  subprocess.Popen(["rosbag",  "record", topic +"/"+mode, "-O", "bag/" +collection[3:],  "--duration=" + str(bag_duration)])
time.sleep(bag_duration)

# jpeg case 3:
jpeg_quality = 50
collection = "db." + topic_name + "__" + mode + "__" + "JPEG" + str(jpeg_quality)

insert_j3 = subprocess.Popen(["rosservice", "call", "/iai_configurator/update",
                             "{topic: '" + topic +"', format: '" + format_comp + "', jpeg_quality: " + str(jpeg_quality) + ", db_host: '" + db_host + "', collection: '" + collection + "', mode: '" + mode + "'}"])

rosbag_j3 =  subprocess.Popen(["rosbag",  "record", topic +"/"+mode, "-O", "bag/" +collection[3:],  "--duration=" + str(bag_duration)])
time.sleep(bag_duration)

# jpeg case 4:
jpeg_quality = 30
collection = "db." + topic_name + "__" + mode + "__" + "JPEG" + str(jpeg_quality)

insert_j4 = subprocess.Popen(["rosservice", "call", "/iai_configurator/update",
                             "{topic: '" + topic +"', format: '" + format_comp + "', jpeg_quality: " + str(jpeg_quality) + ", db_host: '" + db_host + "', collection: '" + collection + "', mode: '" + mode + "'}"])

rosbag_j4 =  subprocess.Popen(["rosbag",  "record", topic +"/"+mode, "-O", "bag/" +collection[3:],  "--duration=" + str(bag_duration)])
time.sleep(bag_duration)

# jpeg case 5:
jpeg_quality = 10
collection = "db." + topic_name + "__" + mode + "__" + "JPEG" + str(jpeg_quality)

insert_j5 = subprocess.Popen(["rosservice", "call", "/iai_configurator/update",
                              "{topic: '" + topic +"', format: '" + format_comp + "', jpeg_quality: " + str(jpeg_quality) + ", db_host: '" + db_host + "', collection: '" + collection + "', mode: '" + mode + "'}"])

rosbag_j5 =  subprocess.Popen(["rosbag",  "record", topic +"/"+mode, "-O", "bag/" +collection[3:],  "--duration=" + str(bag_duration)])
time.sleep(bag_duration)

# png cases
print ("Executing png cases")
format_comp = "png"

# png case 1:
png_level = 9
collection = "db." + topic_name + "__" + mode + "__" + "PNG" + str(png_level)


insert_p1 = subprocess.Popen(["rosservice", "call", "/iai_configurator/update",
                              "{topic: '" + topic +"', format: '" + format_comp + "', png_level: " + str(png_level) + ", db_host: '" + db_host + "', collection: '" + collection + "', mode: '" + mode + "'}"])

rosbag_p1 =  subprocess.Popen(["rosbag",  "record", topic +"/"+mode, "-O", "bag/" +collection[3:],  "--duration=" + str(bag_duration)])
time.sleep(bag_duration)

# png case 2:
png_level = 7
collection = "db." + topic_name + "__" + mode + "__" + "PNG" + str(png_level)

insert_p2 = subprocess.Popen(["rosservice", "call", "/iai_configurator/update",
                              "{topic: '" + topic +"', format: '" + format_comp + "', png_level: " + str(png_level) + ", db_host: '" + db_host + "', collection: '" + collection + "', mode: '" + mode + "'}"])

rosbag_p2 =  subprocess.Popen(["rosbag",  "record", topic +"/"+mode, "-O", "bag/" +collection[3:],  "--duration=" + str(bag_duration)])
time.sleep(bag_duration)

# png case 3:
png_level = 5
collection = "db." + topic_name + "__" + mode + "__" + "PNG" + str(png_level)

insert_p3 = subprocess.Popen(["rosservice", "call", "/iai_configurator/update",
                              "{topic: '" + topic +"', format: '" + format_comp + "', png_level: " + str(png_level) + ", db_host: '" + db_host + "', collection: '" + collection + "', mode: '" + mode + "'}"])

rosbag_p3 =  subprocess.Popen(["rosbag",  "record", topic +"/"+mode, "-O", "bag/" +collection[3:],  "--duration=" + str(bag_duration)])
time.sleep(bag_duration)

# png case 4:
png_level = 3
collection = "db." + topic_name + "__" + mode + "__" + "PNG" + str(png_level)

insert_p4 = subprocess.Popen(["rosservice", "call", "/iai_configurator/update",
                              "{topic: '" + topic +"', format: '" + format_comp + "', png_level: " + str(png_level) + ", db_host: '" + db_host + "', collection: '" + collection + "', mode: '" + mode + "'}"])

rosbag_p4 =  subprocess.Popen(["rosbag",  "record", topic +"/"+mode, "-O", "bag/" +collection[3:],  "--duration=" + str(bag_duration)])
time.sleep(bag_duration)

# png case 5:
png_level = 1
collection = "db." + topic_name + "__" + mode + "__" + "PNG" + str(png_level)

insert_p5 = subprocess.Popen(["rosservice", "call", "/iai_configurator/update",
                              "{topic: '" + topic +"', format: '" + format_comp + "', png_level: " + str(png_level) + ", db_host: '" + db_host + "', collection: '" + collection + "', mode: '" + mode + "'}"])

rosbag_p5 =  subprocess.Popen(["rosbag",  "record", topic +"/"+mode, "-O", "bag/" +collection[3:],  "--duration=" + str(bag_duration)])
time.sleep(bag_duration)

remove_compressed = subprocess.Popen(["rosservice", "call", "/iai_configurator/remove",
                              "camera__rgb__image_color__compressed"])

# theora cases
print ("Executing theora cases")
mode = "theora"

# theora case 1:
quality = 63
theora = "theora"
collection = "db." + topic_name + "__" + mode + "__" + "THEO" + str(quality)

insert_t1 = subprocess.Popen(["rosservice", "call", "/iai_configurator/update",
                              "{rate: 30, optimize_for: 1, topic: '"+ topic + "', optimize_for: "+ str(optimize_for) +", target_bitrate: " + str(target_bitrate)+", keyframe_frequency: "+ str(keyframe_frequency) +", db_host: '"+ db_host + "', collection: '"+ collection+"', mode: '"+mode+"', quality: " + str(quality)+ "}"])

rosbag_t1 =  subprocess.Popen(["rosbag",  "record", topic +"/"+mode, "-O", "bag/" +collection[3:],  "--duration=" + str(bag_duration)])
time.sleep(bag_duration)

# theora case 2:
quality = 50
collection = "db." + topic_name + "__" + mode + "__" + "THEO" + str(quality)

insert_t2 = subprocess.Popen(["rosservice", "call", "/iai_configurator/update",
                              "{optimize_for: 1, topic: '"+ topic + "', optimize_for: "+ str(optimize_for) +", target_bitrate: " + str(target_bitrate)+", keyframe_frequency: "+ str(keyframe_frequency) +", db_host: '"+ db_host + "', collection: '"+ collection+"', mode: '"+mode+"', quality: " + str(quality)+ "}"])

rosbag_t2 =  subprocess.Popen(["rosbag",  "record", topic +"/"+mode, "-O", "bag/" +collection[3:],  "--duration=" + str(bag_duration)])
time.sleep(bag_duration)

# theora case 3:
quality = 32
collection = "db." + topic_name + "__" + mode + "__" + "THEO" + str(quality)

insert_t3 = subprocess.Popen(["rosservice", "call", "/iai_configurator/update",
                              "{optimize_for: 1, topic: '"+ topic + "', optimize_for: "+ str(optimize_for) +", target_bitrate: " + str(target_bitrate)+", keyframe_frequency: "+ str(keyframe_frequency) +", db_host: '"+ db_host + "', collection: '"+ collection+"', mode: '"+mode+"', quality: " + str(quality)+ "}"])

rosbag_t3 =  subprocess.Popen(["rosbag",  "record", topic +"/"+mode, "-O", "bag/" +collection[3:],  "--duration=" + str(bag_duration)])
time.sleep(bag_duration)

# theora case 4:
quality = 19
collection = "db." + topic_name + "__" + mode + "__" + "THEO" + str(quality)

insert_t4 = subprocess.Popen(["rosservice", "call", "/iai_configurator/update",
                              "{optimize_for: 1, topic: '"+ topic + "', optimize_for: "+ str(optimize_for) +", target_bitrate: " + str(target_bitrate)+", keyframe_frequency: "+ str(keyframe_frequency) +", db_host: '"+ db_host + "', collection: '"+ collection+"', mode: '"+mode+"', quality: " + str(quality)+ "}"])

rosbag_t4 =  subprocess.Popen(["rosbag",  "record", topic +"/"+mode, "-O", "bag/" +collection[3:],  "--duration=" + str(bag_duration)])
time.sleep(bag_duration)

# theora case 5:
quality = 6
collection = "db." + topic_name + "__" + mode + "__" + "THEO" + str(quality)

insert_t5 = subprocess.Popen(["rosservice", "call", "/iai_configurator/update",
                              "{optimize_for: 1, topic: '"+ topic + "', optimize_for: "+ str(optimize_for) +", target_bitrate: " + str(target_bitrate)+", keyframe_frequency: "+ str(keyframe_frequency) +", db_host: '"+ db_host + "', collection: '"+ collection+"', mode: '"+mode+"', quality: " + str(quality)+ "}"])

rosbag_t5 =  subprocess.Popen(["rosbag",  "record", topic +"/"+mode, "-O", "bag/" +collection[3:],  "--duration=" + str(bag_duration)])
time.sleep(bag_duration)

subprocess.Popen(["rosservice", "call", "/iai_configurator/update",
                  "{topic: '/camera/rgb/image_color', db_host: 'localhost', collection: 'db.check', mode: 'compressed'}"])

os.kill(launcher.pid, signal.SIGTERM)
sys.exit(1)

#subprocess.Popen(["rosservice", "call", "/iai_configurator/update",
#                  "{topic: '/camera/rgb/image_raw', format: 'jpeg', iai_id: '', jpeg_quality: 1, png_level: 1, optimize_for: 0, target_bitrate: 800000, keyframe_frequency: 64, quality: 63, depth_max: 10.0, depth_quantization: 100.0, db_host: 'localhost', collection: 'db.check', mode: 'theora', tf_msg_str: '/tf', tf_base: 'base', tf_cam: 'cam', rate: 10.0, motion: false, blur: false, similar: false}"])
