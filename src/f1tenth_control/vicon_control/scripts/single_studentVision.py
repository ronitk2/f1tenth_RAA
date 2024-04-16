import time
import math
import numpy as np
import cv2
import rospy


from single_line_fit import line_fit, tune_fit, bird_fit, final_viz
from Line import Line
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from skimage import morphology


import matplotlib .pyplot as plt
import os


def show_img(path):
  image = cv2.imread(path)
  image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
  
# Display the image
  plt.imshow(image)
  plt.axis('off')  # Don't show axes for images
  plt.show()
  


def show_img_file(image, cmap = None):
  plt.imshow(image, cmap = cmap)
  plt.axis('off')  # Don't show axes for images
  plt.show()




class lanenet_detector():
  def __init__(self):


      self.bridge = CvBridge()
      # NOTE
      # Uncomment this line for lane detection of GEM car in Gazebo
   #    self.sub_image = rospy.Subscriber('/gem/front_single_camera/front_single_camera/image_raw', Image, self.img_callback, queue_size=1)
      # Uncomment this line for lane detection of videos in rosbag
      self.sub_image = rospy.Subscriber('D435I/color/image_raw', Image, self.img_callback, queue_size=1)
      self.pub_image = rospy.Publisher("lane_detection/annotate", Image, queue_size=1)
      self.pub_bird = rospy.Publisher("lane_detection/birdseye", Image, queue_size=1)
      self.pub_waypoint = rospy.Publisher("lane_detection/waypoint", Pose2D, queue_size=1)
   #    self.left_line = Line(n=5)
   #    self.right_line = Line(n=5)
      self.center_line = Line(n=5)
      self.detected = False
      self.hist = True




  def img_callback(self, data):


      try:
          # Convert a ROS image message into an OpenCV image
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      except CvBridgeError as e:
          print(e)


      raw_img = cv_image.copy()
      mask_image, bird_image = self.detection(raw_img)


      if mask_image is not None and bird_image is not None:
          # Convert an OpenCV image into a ROS image message
          out_img_msg = self.bridge.cv2_to_imgmsg(mask_image, 'bgr8')
          out_bird_msg = self.bridge.cv2_to_imgmsg(bird_image, 'bgr8')


          # Publish image message in ROS
          self.pub_image.publish(out_img_msg)
          self.pub_bird.publish(out_bird_msg)




  def gradient_thresh(self, img, thresh_min=20, thresh_max=100):
      #       """
      #       Apply sobel edge detection on input image in x, y direction
      #       """
          #1. Convert the image to gray scale
          #2. Gaussian blur the image
          #3. Use cv2.Sobel() to find derievatives for both X and Y Axis
          #4. Use cv2.addWeighted() to combine the results
          #5. Convert each pixel to unint8, then apply threshold to get binary image




          ## TODO
   #    gray_scale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #convert img to grayscale
   #    gaussian_blur = cv2.GaussianBlur(gray_scale, (kernel_size:=5, kernel_size:=5), sigmaX:=1.5) 
   #    x_derivative = cv2.Sobel(gaussian_blur, cv2.CV_64F, 1, 0, sobel_kernel_size:=3)
   #    y_derivative = cv2.Sobel(gaussian_blur, cv2.CV_64F, 0, 1, sobel_kernel_size:=3)
   #    combined_results = cv2.addWeighted(x_derivative, alpha:=0.5, y_derivative, beta:=0.5, gamma:=0)
   #    uint8_pixels = combined_results.astype(np.uint8)
   #    binary_output = np.where((uint8_pixels >= thresh_min) & (uint8_pixels <= thresh_max), 1, 0)
   #    binary_image = binary_output * 255


      gray_scale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
      gaussian_blur = cv2.GaussianBlur(gray_scale, (5, 5), 0)
      x_derivative = cv2.Sobel(gaussian_blur, cv2.CV_64F, 1, 0, ksize=3)
      abs_x_derivative = np.absolute(x_derivative)
      scaled_x_derivative = np.uint8(255*abs_x_derivative/np.max(abs_x_derivative))
      binary_output = np.zeros_like(scaled_x_derivative)
      binary_output[(scaled_x_derivative >= thresh_min) & (scaled_x_derivative <= thresh_max)] = 1 


      return binary_output


  




  def color_thresh(self, img):
  
     # Convert RGB to hls and threshold to binary image using S channel
     #1. Convert the image from RGB to hls
     #2. Apply threshold on S channel to get binary image
     #Hint: threshold on H to remove green grass
     ## TODO  
   #    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)    
   #    hls_image = cv2.cvtColor(img, cv2.COLOR_RGB2HLS) #technically HLS, L and S between 0 and 255
   #    rgb_image = cv2.cvtColor(hls_image, cv2.COLOR_HLS2RGB)
   #    binary_output = np.where((((hls_image[:,:,2] >= 105)  & ((hls_image[:,:,0] >= 20) & (hls_image[:,:,0] <= 32) ))| ((hls_image[:,:,1] >= 180))), 1, 0)
   #    binary_image = binary_output * 255
      hls_image = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
      # Define range for yellow color and apply mask
      lower_yellow = np.array([15, 30, 115])  # Adjust these values
      upper_yellow = np.array([35, 204, 255])  # Adjust these values
      yellow_mask = cv2.inRange(hls_image, lower_yellow, upper_yellow)
      binary_output = np.zeros_like(yellow_mask)
      binary_output[yellow_mask > 0] = 1


      ####
      return binary_output






  def combinedBinaryImage(self, img):
#       """
#       Get combined binary image from color filter and sobel filter
#       """
     #1. Apply sobel filter and color filter on input image
     #2. Combine the outputs
     ## Here you can use as many methods as you want.




     ## TODO
      SobelOutput = self.gradient_thresh(img)
      ColorOutput = self.color_thresh(img)
          ####


      binaryImage = np.zeros_like(SobelOutput)
      binaryImage[(ColorOutput==1)|(SobelOutput==1)] = 1
      # Remove noise from binary image
      binaryImage = morphology.remove_small_objects(binaryImage.astype('bool'),min_size=50,connectivity=2)


      return binaryImage




  def perspective_transform(self, img, verbose=False):
      """
      Get bird's eye view from input image
      """
      #1. Visually determine 4 source points and 4 destination points
      #2. Get M, the transform matrix, and Minv, the inverse using cv2.getPerspectiveTransform()
      #3. Generate warped image in bird view using cv2.warpPerspective()


      ## TODO
      if img.max() <= 1.0:
          img = (img * 255).astype(np.uint8)
      height = img.shape[0]
      width = img.shape[1]


      source_points = np.float32([[0.4*width, 0.6*height],
                                  [0, height-10],
                                  [0.6*width, 0.6*height],
                                  [width, height-10]])
      destination_points = np.float32([[120,0],
                                      [120,480],
                                      [520,0],
                                      [520,480]])
   #2. Get M, the transform matrix, and Minv, the inverse using cv2.getPerspectiveTransform()
      M = cv2.getPerspectiveTransform(source_points, destination_points)
      Minv = cv2.getPerspectiveTransform(destination_points, source_points)


   #3. Generate warped image in bird view using cv2.warpPerspective()
      warped_img = cv2.warpPerspective(img, M, (width, height), flags=cv2.INTER_LINEAR)
      ####
      return warped_img, M, Minv






  def detection(self, img):


      binary_img = self.combinedBinaryImage(img)
      img_birdeye, M, Minv = self.perspective_transform(binary_img)


      if not self.hist:
          # Fit lane without previous result
          ret = line_fit(img_birdeye)
       #    left_fit = ret['left_fit']
       #    right_fit = ret['right_fit']
          center_fit = ret['center_fit']
          nonzerox = ret['nonzerox']
          nonzeroy = ret['nonzeroy']
       #    left_lane_inds = ret['left_lane_inds']
       #    right_lane_inds = ret['right_lane_inds']
          lane_inds = ret['lane_inds']
          waypoint_x = ret['waypoint_x']
          waypoint_y = ret['waypoint_y']
          yaw = ret['yaw']
          self.pub_waypoint.publish(waypoint_x, waypoint_y, yaw)


      else:
          # Fit lane with previous result
          if not self.detected:
              ret = line_fit(img_birdeye)


              if ret is not None:
               #    left_fit = ret['left_fit']
               #    right_fit = ret['right_fit']
                  center_fit = ret['center_fit']
                  nonzerox = ret['nonzerox']
                  nonzeroy = ret['nonzeroy']
               #    left_lane_inds = ret['left_lane_inds']
               #    right_lane_inds = ret['right_lane_inds']
                  lane_inds = ret['lane_inds']
                  waypoint_x = ret['waypoint_x']
                  waypoint_y = ret['waypoint_y']
                  yaw = ret['yaw']
                  self.pub_waypoint.publish(waypoint_x, waypoint_y, yaw)


               #    left_fit = self.left_line.add_fit(left_fit)
               #    right_fit = self.right_line.add_fit(right_fit)
                  center_fit = self.center_line.add_fit(center_fit)


                  self.detected = True


          else:
           #    left_fit = self.left_line.get_fit()
           #    right_fit = self.right_line.get_fit()
           #    ret = tune_fit(img_birdeye, left_fit, right_fit)
              center_fit = self.center_line.get_fit()
              ret = tune_fit(img_birdeye, center_fit)


              if ret is not None:
               #    left_fit = ret['left_fit']
               #    right_fit = ret['right_fit']
                  center_fit = ret['center_fit']
                  nonzerox = ret['nonzerox']
                  nonzeroy = ret['nonzeroy']
               #    left_lane_inds = ret['left_lane_inds']
               #    right_lane_inds = ret['right_lane_inds']
                  lane_inds = ret['lane_inds']
                  waypoint_x = ret['waypoint_x']
                  waypoint_y = ret['waypoint_y']
                  yaw = ret['yaw']
                  self.pub_waypoint.publish(waypoint_x, waypoint_y, yaw)


               #    left_fit = self.left_line.add_fit(left_fit)
               #    right_fit = self.right_line.add_fit(right_fit)
                  center_fit = self.center_line.add_fit(center_fit)


              else:
                  self.detected = False


          # Annotate original image
          bird_fit_img = None
          combine_fit_img = None
          if ret is not None:
              bird_fit_img = bird_fit(img_birdeye, ret, save_file=None)
              combine_fit_img = final_viz(img, center_fit, Minv)
          else:
              print("Unable to detect lanes")


          return combine_fit_img, bird_fit_img




if __name__ == '__main__':
  # init args
  rospy.init_node('lanenet_node', anonymous=True)
  lanenet_detector()
  while not rospy.core.is_shutdown():
      rospy.rostime.wallsleep(0.5)

