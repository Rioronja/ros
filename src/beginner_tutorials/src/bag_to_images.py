#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import os
import argparse

import cv2

import rosbag
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

def main():
    """Extract a folder of images and yaw value from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images and yaw from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("sec_per_frame", help="second per frame")
    args = parser.parse_args()
    # image update frequency:   (10Hz) 100 ms
    # yaw   update frequency:   (60Hz)  16 ms
    interval = int(float(args.sec_per_frame)/0.1)
    if interval <= 1:
        print "sec_per_frame need to be larger than 0.1" 
        exit(0)

    output_dir = os.path.basename(args.bag_file)[:-4]
    output_dir_frames = output_dir + "/frames"
    if not os.path.exists(output_dir_frames):
        os.makedirs(output_dir_frames)
    image_topic = "/usb_cam/image_raw/compressed"

    output_dir_yaw = output_dir + "/yaw"
    if not os.path.exists(output_dir_yaw):
        os.makedirs(output_dir_yaw)
    yaw_topic = "/m100/yaw"


    print "interval: %d" % interval
    print "Extract images from %s on topic %s into %s" % (args.bag_file, image_topic, output_dir_frames)

    bag = rosbag.Bag(args.bag_file, "r")
    bridge = CvBridge()
    count = 0
    # save frames into output_dir_frames
    for topic, msg, t in bag.read_messages(topics=[image_topic]):
        if count%interval ==0:
            # every 100ms
            timestamp = int(t.to_sec()*10)
            cv_img = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            cv2.imwrite(os.path.join(output_dir_frames, "%d.jpg" % timestamp), cv_img)
        count += 1
    
    # for topic, msg, t in bag.read_messages(topics=[yaw_topic]):
    #     if count%interval ==0:
    #         # every 100ms
    #         timestamp = int(t.to_sec()*10)
    #         with open(os.path.join(output_dir_yaw, "%d.txt" % timestamp), 'w') as f:
    #             f.write(str(msg))
    #     count += 1
    bag.close()

    return

if __name__ == '__main__':
    main()