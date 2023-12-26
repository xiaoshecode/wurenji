#!/usr/bin/python
#-*- encoding: utf8 -*-

import os
import sys
import time
# from scipy.spatial.transform import Rotation as R
from collections import deque
from enum import Enum
import rospy
import cv2
import numpy as np
import math
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
sys.path.insert(0, '/root/catkin_ws/src/raspicam_node/src/cv')
from circle_detect import circle_detector

class TestNode:
    class FlightState(Enum):
        WAITING = 1
        TAKEOFF = 2
        NAVIGATING = 3
        LANDING = 4
        LANDED = 5

    def __init__(self):
        rospy.init_node('Test_node', anonymous=True)
        rospy.logwarn('Test node set up.')

        self.command_prefix_ = "rosrun ttauav_node service_client 4 "
        self.Focal_length = 1.050e+03 # pixel
        
        self.image_ = None
        self.bridge_ = CvBridge()
        self.flight_state_ = self.FlightState.WAITING
        self.navigating_queue_ = deque()
        self.navigation = np.array([0, 0, 0, 0])   # x, y, r, d

        self.is_begin_ = True
        self.is_takeoff_ = False
        self.is_adjusting = True

        self.detector_ = circle_detector()
        self.imageSub_ = rospy.Subscriber('camera/image', Image, self.imageCallback)
        
        self.max_dis = 0.4    # m
        self.thresh_dis = 30  # cm
        self.fw_cnt = 0
        self.bias_x = 40
        self.bias_h = 30

        self.rate = rospy.Rate(.15)
        rospy.logwarn(rospy.is_shutdown())
        while not rospy.is_shutdown():
            if self.is_begin_:
                self.decision()
            if self.flight_state_ == self.FlightState.LANDED:
                break
            self.rate.sleep()
        rospy.logwarn('Test node shut down.')
    
    def decision(self):
        # take off
        if self.flight_state_ == self.FlightState.WAITING:
            rospy.logwarn('Waiting for take off.')
            self.flight_state_ = self.FlightState.TAKEOFF

        elif self.flight_state_ == self.FlightState.TAKEOFF:
            if not self.is_takeoff_:
                rospy.logwarn('Taking off.')
                rospy.logwarn("[COMMAND] rosrun ttauav_node service_client 1")
                ack = os.system("rosrun ttauav_node service_client 1")
                rospy.sleep(5)
                rospy.loginfo("[ACK 1]")
                rospy.loginfo(ack)
                self.is_takeoff_ = True
            else:
                rospy.logwarn('Lifting.')
                rospy.logwarn("[COMMAND] rosrun ttauav_node service_client 4 0 0 0.4")
                ack = os.system("rosrun ttauav_node service_client 4 0 0 0.4")
                rospy.sleep(5)
                self.flight_state_ = self.FlightState.NAVIGATING
        
        elif self.flight_state_ == self.FlightState.NAVIGATING:
            rospy.logwarn('Navigating.')
            if not self.navigating_queue_:
                if self.is_adjusting:
                    if np.abs(self.navigation[0]) > self.thresh_dis:
                        self.navigating_queue_.append(['e', self.navigation[0]])
                        rospy.loginfo('[DATA INFO] Navigating: bad horizontal location adjusting %d', int(-self.navigation[0]))
                    if np.abs(self.navigation[1]) > self.thresh_dis:
                        self.navigating_queue_.append(['d', -self.navigation[1]])
                        rospy.loginfo('[DATA INFO] Navigating: bad height adjusting %d', int(-self.navigation[1]))
                
                if not self.navigating_queue_:
                    self.navigating_queue_.append(['n', 150])
                    rospy.loginfo('[DATA INFO] Navigating: Going forward')
                    self.fw_cnt += 1
                    if self.fw_cnt == 4:
                        self.flight_state_ = self.FlightState.LANDING
                    # self.flight_state_ = self.FlightState.LANDING
                    # return
                    # dis = self.navigation[3]
                    # self.navigating_queue_.append(['n', dis])
                    # rospy.loginfo('Navigating: good position, go foward %d', int(dis))
                
            next_nav = self.navigating_queue_.popleft()
            self.navigation2Command(next_nav[0], next_nav[1])

            
        elif self.flight_state_ == self.FlightState.LANDING:
            rospy.logwarn('Landing.')
            rospy.logwarn("[COMMAND] rosrun ttauav_node service_client 2")
            os.system("rosrun ttauav_node service_client 2")
            self.flight_state_ = self.FlightState.LANDED

    def imageCallback(self, msg):
        bridge = CvBridge()
        self.image_ = bridge.imgmsg_to_cv2(msg, "bgr8")
        image_cp = self.image_.copy()
        h, w = image_cp.shape[:2]
        # rospy.loginfo('h = ' + str(h))
        circles = self.detector_.detect(image_cp)

        if circles:
            r, x, y = circles[0]
            x = float(x)
            y = float(y)
            r = float(r)
            if self.is_adjusting:
                if r > 430.0:
                    self.is_adjusting = False
            distance = 171.0 / 2.0 * self.Focal_length / r
            self.navigation = np.array([(x - (w/2 - self.bias_x)) / r * 171.0 / 2.0, (y - (h/2 - self.bias_h)) / r * 171.0 / 2.0, r, distance])
            rospy.loginfo("[IMG CALLBACK] Ring detected! x: %d, y: %d, r: %d, d: %d" % (x, y, r, distance))
            rospy.loginfo("[NAVIGATION CALLBACK] 1: %f, 2: %f" % (self.navigation[0], self.navigation[1]))
            
    def navigation2Command(self, dim, dis):
        dis = dis / 100.0       # cm to m
        dis = dis / 2.0           # flight time = 2000 ms
        if dim != 'n':
          if np.abs(dis) > self.max_dis:
              if dis > 0:
                  dis = self.max_dis
              else:
                  dis = -self.max_dis
        dis_str = str(dis)      # m
        command_msg = self.command_prefix_
        if dim == 'd':
            command_msg += '0 0 ' + dis_str
        elif dim == 'e':
            command_msg += '0 ' + dis_str + ' 0'
        elif dim == 'n':
            command_msg += dis_str + ' 0 0'
        
        rospy.logwarn("[COMMAND] " + command_msg)
        ack = os.system(command_msg)
        rospy.sleep(5)
        
if __name__ == "__main__":
    TestNode()