#!/usr/bin/env python3

#================================================================
# File name: gem_gnss_pp_tracker_pid.py                                                                  
# Description: gnss waypoints tracker using pid and pure pursuit                                                                
# Author: Hang Cui
# Email: hangcui3@illinois.edu                                                                     
# Date created: 08/02/2021                                                                
# Date last modified: 08/15/2022                                                          
# Version: 1.0                                                                   
# Usage: rosrun gem_gnss gem_gnss_pp_tracker.py                                                                      
# Python version: 3.8                                                             
#================================================================

from __future__ import print_function

# Python Headers
import os 
import csv
import math
import numpy as np
from numpy import linalg as la
import scipy.signal as signal
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import subprocess
import sys

# ROS Headers
import rospy

# GEM Sensor Headers
from std_msgs.msg import String, Bool, Float32, Float64, Float64MultiArray
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class lanenet_detector():
  def __init__(self):
      self.bridge = CvBridge()  # convert ROS image to OpenCV formate
      # NOTE
      # Uncomment this line for lane detection of GEM car in Gazebo
   #    self.sub_image = rospy.Subscriber('/gem/front_single_camera/front_single_camera/image_raw', Image, self.img_callback, queue_size=1)
      # Uncomment this line for lane detection of videos in rosbag
      self.sub_image = rospy.Subscriber('D435I/color/image_raw', Image, self.img_callback, queue_size=1)    # subscribe to camera image
      self.ctrl_pub  = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)   # publish steering and speed commands
      self.debug_image_pub = rospy.Publisher('debug/yellow_line_detection', Image, queue_size=1)    # pubish yellow line detection image

  def img_callback(self, data):
      try:
          # Convert a ROS image message into an OpenCV image
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")    
      except CvBridgeError as e:
          print(e)

      yellow_lane = self.color_thresh(cv_image) # detect yellow lane
      steering_angle, arrow_cords = self.line_fit(yellow_lane)  # get steering angle
      self.drive_msg = AckermannDriveStamped()  # control vehicle using Ackermann drive
      self.drive_msg.header.frame_id = "f1tenth_control"    # set header frame id
      self.drive_msg.header.stamp = rospy.get_rostime() # record current time
      self.drive_msg.drive.speed     = 1.3 # m/s, reference speed  
      self.drive_msg.drive.steering_angle = steering_angle  # set steering angle
      self.ctrl_pub.publish(self.drive_msg) # publish vehicle control message
      debug_image = self.create_debug_image(cv_image, yellow_lane, (steering_angle, arrow_cords))   # create lane following image
      try:
          self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, "bgr8"))  # publish image to ROS topic
      except CvBridgeError as e:
          print(e)

  def create_debug_image(self, img, mask, steering_info):
      steering_angle, arrow_cords = steering_info   # get steering_angle and arrow info
      mask_indices = np.where(mask > 0) # indices where yellow line is detected
      debug_img = img.copy()    # create image copy
      if np.any(mask_indices):  # if yellow line is detected
          min_y, max_y = np.min(mask_indices[0]), np.max(mask_indices[0])   # get y bounds of line
          min_x, max_x = np.min(mask_indices[1]), np.max(mask_indices[1])   # get x bounds of line
          cv2.rectangle(debug_img, (min_x, min_y), (max_x, max_y), (255, 0, 0), 2)  # draw red rectangle around area

          cv2.arrowedLine(debug_img, arrow_cords[:2], arrow_cords[2:], (0,255,0), 10, tipLength=0.5)    # draw green arrow for steering direction
      return debug_img

  def color_thresh(self, img):
  
     # Convert RGB to hls and threshold to binary image using S channel
     #1. Convert the image from RGB to hls
     #2. Apply threshold on S channel to get binary image
     #Hint: threshold on H to remove green grass
     ## TODO    
      hls_image = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)  # convert RGB to HLS
      lower_yellow = np.array([15, 30, 115])  # yellow lower limit
      upper_yellow = np.array([35, 204, 255])  # yellow upper limit
      yellow_mask = cv2.inRange(hls_image, lower_yellow, upper_yellow)  # isolate yellow
      binary_output = np.zeros_like(yellow_mask)    # binary image
      binary_output[yellow_mask > 0] = 1    # set yellow pixels to 1

      ####
      return binary_output

  def line_fit(self, binary_warped):
      """
      Find and fit lane lines
      """
      # Assuming you have created a warped binary image called "binary_warped"
      # Take a histogram of the bottom half of the image
      contours, _ = cv2.findContours(binary_warped, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # find contours in binary image
      frame_center_x = binary_warped.shape[1] / 2   # x value of center
      frame_center_y = binary_warped.shape[0] / 2   # y value of center
      if contours:  # if contour exists
          largest_contour = max(contours, key=cv2.contourArea)  # largest contour = most yellow pixels
          M = cv2.moments(largest_contour)  # calculate moments of largest contour
          if M['m00'] != 0: # M['m00'] = area of contour
              cx = int(M['m10'] / M['m00']) # find centroid center x value
              cy = int(M['m01'] / M['m00']) # find centroid center y value
              dx = cx - frame_center_x  # x distance between center of image and contour
              dy = cy - frame_center_y  # y distance between center of image and contour
              if abs(dx) < 50 and abs(dy) < 50: # if distance is not much
                  return 0, (frame_center_x, frame_center_y, frame_center_x, frame_center_y+50) # don't steer
              steering_angle = np.arctan2(dy, dx)   # get angle from distances
              steering_angle = -steering_angle  # reverse steering angle
              end_x = int(cx + 100 * np.cos(steering_angle))    # calculate arrow x
              end_y = int(cy + 100 * np.sin(steering_angle))    # calculate arrow y
              return steering_angle, (cx, cy, end_x, end_y)
      return 0, (frame_center_x, frame_center_y, frame_center_x, frame_center_y+50)

           

#   def line_fit(self, binary_warped):
#      """
#      Find and fit lane lines
#      """
#      # Assuming you have created a warped binary image called "binary_warped"
#      # Take a histogram of the bottom half of the image
#      histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
#      # These will be the starting point for the left and right lines
#      base = np.argmax(histogram)
#      current = base
#      # Choose the number of sliding windows
#      nwindows = 9
#      # Set height of windows
#      window_height = int(binary_warped.shape[0]/nwindows)
#      # Set the width of the windows +/- margin
#      margin = 100
#      # Set minimum number of pixels found to recenter window
#      minpix = 50
#      # Create empty lists to receive left and right lane pixel indices
#      lane_inds = []

#      # Step through the windows one by one
#      for window in range(nwindows):
#         # Identify window boundaries in x and y (and right and left)
#         ##TO DO
#         minx = current - margin
#         maxx = current + margin
#         miny = binary_warped.shape[0] - (window+1) * window_height
#         maxy = binary_warped.shape[0] - window * window_height

#         ####
#         # Identify the nonzero pixels in x within the window
#         ##TO DO
#         nonzerox = ((binary_warped[miny:maxy, minx:maxx] > 0).nonzero()[1] + minx)

#         ####
#         # Append these indices to the lists
#         ##TO DO
#         lane_inds.append(nonzerox)

#         ####
#         # If you found > minpix pixels, recenter next window on their mean position
#         ##TO DO    
#         if (len(nonzerox) > minpix):
#            current = int(np.mean(nonzerox))

#      # Concatenate the arrays of indices
#      if lane_inds:
#         lane_xval = np.concatenate(lane_inds)
#         if len(lane_xval) > minpix:
#            lane_center = np.mean(lane_xval)
#            dist_x = lane_center - (binary_warped.shape[1] / 2)
#            steering_angle = np.arctan2(dist_x, binary_warped.shape[0])
#            steering_angle = -steering_angle
#            angle_threshold = 0.1
#            if abs(steering_angle) < angle_threshold:
#               return 0
#            return steering_angle
           
#      return 0



def lane_follower():
    rospy.init_node('lanenet_detector', anonymous=True) # initialize ros node
    try:
       ld = lanenet_detector()  # creat lanenet_detector instance
       rospy.spin() # node should keep running
    except rospy.ROSInterruptException:
        pass    # shutdown


if __name__ == '__main__':
    lane_follower()
