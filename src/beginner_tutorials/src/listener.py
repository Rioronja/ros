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
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import os
import sys

if len(sys.argv) != 1:
    rospy.loginfo("No need any argument!")
    sys.exit(0)

init_yaw = 0
init_flag = 1
heading_angle = 90 # It means 0 is right, 180 is left
angle = 0
image_msg = None
output_dir_frames = "/home/nems/ekko/outdoor_experiment/frames/tmp_data"
if not os.path.exists(output_dir_frames):
    os.makedirs(output_dir_frames)

degrees = {0:0, 90:0, 180:0, 270:0}

def yaw_callback(data):
    # data.z is yaw degree
    global init_flag
    global init_yaw
    global angle
    if init_flag:
        init_yaw = data.z
        init_flag = 0
    yaw_diff = data.z - init_yaw
    angle = (yaw_diff + 360 + heading_angle) % 360

    string = "Waiting for: "
    count = 0
    for degree in sorted(degrees):
        if not degrees[degree]:
            string += str(degree) + " "
            count +=1
    if count == 0:
        rospy.loginfo("{:.2f}  OK!".format(angle, string))
    else:
        rospy.loginfo("{:.2f}  {}".format(angle, string))


def img_callback(data):
    global image_msg
    image_msg = data

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('m100/yaw', Vector3, yaw_callback, queue_size = 3)
    rospy.Subscriber('usb_cam/image_raw/compressed', CompressedImage, img_callback, queue_size = 3)
    while True:
        raw_input("Press Enter for saving image!")
        rospy.loginfo(angle)
        filename = str(int(angle)) + ".jpg"
        bridge = CvBridge()
        cv_img = bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
        cv2.imwrite(os.path.join(output_dir_frames, filename), cv_img)
        if angle >= 70 and angle <=110:
            # bridge = CvBridge()
            # cv_img = bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
            cv2.imwrite(os.path.join(output_dir_frames, "90.jpg"), cv_img)
            degrees[90] = 1
        elif (angle <= 20 or angle >=340):
            # bridge = CvBridge()
            # cv_img = bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
            cv2.imwrite(os.path.join(output_dir_frames, "0.jpg"), cv_img)
            degrees[0] = 1
        elif angle >= 150 and angle <=200:
            # bridge = CvBridge()
            # cv_img = bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
            cv2.imwrite(os.path.join(output_dir_frames, "180.jpg"), cv_img)
            degrees[180] = 1
        elif angle >= 250 and angle <=290:
            # bridge = CvBridge()
            # cv_img = bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
            cv2.imwrite(os.path.join(output_dir_frames, "270.jpg"), cv_img)
            degrees[270] = 1
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()