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
import os
import sys
import time
import signal
import socket
import thread
import datetime
sys.path.reverse()
import cv2


if len(sys.argv) != 1:
    rospy.loginfo("No need any argument!")
    sys.exit(0)

init_yaw = 0
init_flag = 1
heading_angle = 90 # It means 0 is right, 180 is left
angle = 0
image_msg = None
output_dir_frames = "/home/nems/rio/image_airtime/"
curTime = datetime.datetime.now().strftime("%Y%m%d%H%M%S")

if not os.path.exists(output_dir_frames):
    os.makedirs(output_dir_frames)

def signal_handler(sig, frame):
    print("Exit!")
    exit()

def img_callback(data):
    global image_msg
    image_msg = data

def gps_callback(gps_feedback):
    global GPS 
    GPS = gps_feedback

def yaw_callback(yaw_feedback):
    global YAW
    YAW = yaw_feedback

def ref_callback(ref_feedback):
    global REF
    REF = ref_feedback

def dmesg_log():
    os.system("dmesg -wH >> {0}dmesg_log_{1}.txt".format(output_dir_frames, curTime))

def recv_print():
    data = b''
    payload_size = struct.calcsize(">l")
    while 1:
        while len(data) < payload_size:
            data += client_socket.recv(4096)
        payload = data[:payload_size]
        data = data[payload_size:]
        payload = struct.unpack(">L", payload)[0]
        while len(data) < payload:
            data += client_socket.recv(4096)
        frac = data[:payload]
        data = data[payload:]
        frac = pickle.loads(frac, fix_imports=True, encoding="bytes")
        rtt = time.time() - frac['time']
        #print(rtt) #, frac['result'])

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
    serialNum = 0
    cnt=0
    timeThen = time.time()
    
    f = open(output_dir_frames + "img_gps_log_" + curTime + ".txt", 'w')
    while True:
        ret, frame = cam.read()
        ret, frame = cv2.imencode('.jpg', frame)
        frac = {}
        frac['frame'] = frame
        frac['serialNum'] = serialNum
        frac['time'] = time.time()
        data = pickle.dumps(frac, 0)
        size = len(data)
        seq = ("{0:x}".format(serialNum)).zfill(4)
        os.system("sudo iptables -t mangle -A OUTPUT -j MARK -d 192.168.5.51 --set-mark 0xb45f{0} -p tcp".format(seq))
        client_socket.sendall(struct.pack(">L", size) + data)
        seq = ("{0:x}".format(serialNum)).zfill(4)        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        cnt += 1
        timeNow = time.time()
        if timeNow - timeThen >= 1:
            timeThen = time.time()
            f.write("FPS: {0}\n".format(cnt))
            cnt = 0
        f.write("Frame id: {0}, size: {1}\n".format(serialNum, size))
        f.write("GPS -> latitude: {0}, longitude: {1}, altitude: {2}, time: {3}\n".format(GPS.latitude, GPS.longitude, GPS.altitude, GPS.header.stamp.secs))
        f.write("Ref -> x: {0}, y: {1}, z: {2}, time: {3}\n".format(REF.point.x, REF.point.y, REF.point.z, REF.header.stamp.secs))
        f.write("Yaw -> yaw: {0}, time: {1}\n".format(YAW.vector.z, YAW.header.stamp.secs))
        f.flush()
        serialNum += 1
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect(('192.168.5.51', 9000))
    recv_thd = threading.Thread(target=recv_print)
    log_thd = threading.Thread(target=dmesg_log)
    recv_thd.deamon = True
    recv_thd.start()
    log_thd.deamon = True
    log_thd.start()
    signal.signal(signal.SIGINT, signal_handler)
    connection = client_socket.makefile('wb')
    cam = cv2.VideoCapture(0)
    listener()
