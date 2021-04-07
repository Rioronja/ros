#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import NavSatStatus
from sensor_msgs.msg import NavSatFix
from cv_bridge import CvBridge
import cv2
import os
import sys
import time
import signal
import socket
if len(sys.argv) != 1:
    rospy.loginfo("No need any argument!")
    sys.exit(0)
init_yaw = 0
init_flag = 1
heading_angle = 90 # It means 0 is right, 180 is left
angle = 0
image_msg = None
output_dir_frames = "/home/nems/rio/cam_localization/"
if not os.path.exists(output_dir_frames):
    os.makedirs(output_dir_frames)

def img_callback(data):
    global image_msg
    image_msg = data
def signal_handler(sig, frame):
    print("Exit!")
    sys.exit()

def gps_callback(gps_feedback):
    # global GPS = NavSatFix()
    # print("GPS feed_back: {0}".format(gps_feedback))
    global GPS 
    GPS = gps_feedback

def yaw_callback(yaw_feedback):
    # global yaw = Vector3Stamped()
    global YAW
    YAW = yaw_feedback
    # print("YAW feed_back: {0}".format(yaw_feedback.vector.z))

def ref_callback(ref_feedback):
    # global ref = PointStamped()
    global REF
    REF = ref_feedback
    # print("REF feed_back: {0}".format(ref_feedback))

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('frameGetter', anonymous=True)
    rospy.Subscriber('usb_cam/image_raw/compressed', CompressedImage, img_callback, queue_size = 3)
    rospy.Subscriber("/dji_sdk/local_position", PointStamped, ref_callback, queue_size = 3)
    rospy.Subscriber("/dji_sdk/gps_position", NavSatFix, gps_callback, queue_size = 3)
    rospy.Subscriber("/m100/yaw", Vector3Stamped, yaw_callback, queue_size = 3)
    while True:
        raw_input("Press Enter for saving image and FTM Measure!")
        msg = "Start"
        s.send(msg.encode('utf-8'))
        msg = s.recv(2048)
        curTime = int(time.time())
        img_filename = str(curTime) + ".jpg"
        data_filename = str(curTime) + ".csv"
        f = open(output_dir_frames + data_filename, 'w')
        f.write("GPS -> latitude: {0}, longitude: {1}, altitude: {2}, time: {3}\n".format(GPS.latitude, GPS.longitude, GPS.altitude, GPS.header.stamp.secs))
        f.write("Ref -> x: {0}, y: {1}, z: {2}, time: {3}\n".format(REF.point.x, REF.point.y, REF.point.z, REF.header.stamp.secs))
        f.write("Yaw -> yaw: {0}, time: {1}\n".format(YAW.vector.z, YAW.header.stamp.secs))
        f.write(msg.decode('utf-8') + '\n')
        # print("GPS -> latitude: {0}, longitude: {1}, altitude: {2}, time: {3}\n".format(GPS.latitude, GPS.longitude, GPS.altitude, GPS.header.stamp.secs))
        # print("Ref -> x: {0}, y: {1}, z: {2}, time: {3}\n".format(REF.point.x, REF.point.y, REF.point.z, REF.header.stamp))
        # print("Yaw -> yaw: {0}, time: {1}".format(YAW.vector.z, YAW.header.stamp.secs))
        f.flush()
        f.close()
        bridge = CvBridge()
        cv_img = bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
        cv2.imwrite(os.path.join(output_dir_frames, img_filename), cv_img)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    server = '192.168.51.137'
    port = 2000
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((server, port))
    listener()
