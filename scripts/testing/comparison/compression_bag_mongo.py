import subprocess
import sys
import time
import os
import signal

topic = "/camera/rgb/image_raw"
topic_name = "camera__rgb__image_raw"
jpeg_quality = 100
png_level = 9
format = "jpeg"
mode = "compressed"
collection = "db.standard"

bag_duration = 10

if __name__ == "__main__":
    launcher = subprocess.Popen(["roslaunch",  "iai_image_logging", "kinect_setup.launch"])
    time.sleep(bag_duration)

    # case 1:
    topic = "/camera/rgb/image_raw"
    topic_name = "camera__rgb__image_raw"
    jpeg_quality = 100
    png_level = 9
    format = "jpeg"
    mode = "compressed"
    collection = topic_name + "__" + mode + "__" + format + "__" + "JPEG" + str(jpeg_quality) + "__" + "PNG" + str(png_level)
    insert_1 = subprocess.Popen(["rosservice", "call", "/iai_configurator/update",
                                 "{'topic':'" + topic + "', 'format':'" + format + "' , 'jpeg_quality': " + str(jpeg_quality) + ", 'collection':'db." + collection + "' , 'mode':'" + mode + "', 'db_host':'localhost'}"])
    rosbag_1 =  subprocess.Popen(["rosbag",  "record", topic, "-O", collection,  "--duration=" + str(bag_duration)])
    time.sleep(bag_duration)

    #os.kill(insert_1.pid, signal.SIGTERM)
    #os.kill(rosbag_1.pid, signal.SIGTERM)
    os.kill(launcher.pid, signal.SIGTERM)
    sys.exit(1)

    # rosrun dynamic_reconfigure dynparam set /iai_updater "{'topic':'/camera/rgb/image_raw', 'format':'jpeg', 'jpeg_quality':100, 'png_level':9, 'db_host':'localhost', 'collection':'db.standard','optimize_for': 0, 'target_bitrate': 800000, 'keyframe_frequency':64, 'quality': 31, 'depth_max':10.0, 'depth_quantization':100.0, 'mode':0, 'cam_no':0}"