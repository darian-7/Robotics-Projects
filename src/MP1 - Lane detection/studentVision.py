import time
import math
import numpy as np
import cv2
import rospy
import pandas as pd
import matplotlib.pyplot as plt

from line_fit import line_fit, tune_fit, bird_fit, final_viz
from Line import Line
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from skimage import morphology


class lanenet_detector():
    def __init__(self):

        self.bridge = CvBridge()
        # NOTE
        # Uncomment this line for lane detection of GEM car in Gazebo
        #self.sub_image = rospy.Subscriber('/gem/front_single_camera/front_single_camera/image_raw', Image, self.img_callback, queue_size=1)
        # Uncomment this line for lane detection of videos in rosbag
        self.sub_image = rospy.Subscriber('camera/image_raw', Image, self.img_callback, queue_size=1)
        self.pub_image = rospy.Publisher("lane_detection/annotate", Image, queue_size=1)
        self.pub_bird = rospy.Publisher("lane_detection/birdseye", Image, queue_size=1)
        self.left_line = Line(n=5)
        self.right_line = Line(n=5)
        self.detected = False
        self.hist = True


    def img_callback(self, data):

        try:
            # Convert a ROS image message into an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #cv2.imshow("cvimage",cv_image)
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
            

    def gradient_thresh(self, img, thresh_min, thresh_max):
        """
        Apply sobel edge detection on input image in x, y direction
        """
        #1. Convert the image to gray scale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #2. Gaussian blur the image
        blur = cv2.GaussianBlur(gray,(5,5),0)
        #3. Use cv2.Sobel() to find derievatives for both X and Y Axis
        sobelx = cv2.Sobel(blur, cv2.CV_64F, 1,0,ksize =1)
        sobely = cv2.Sobel(blur, cv2.CV_64F, 0,1,ksize =1)
        #added = 0.5*sobelx + 0.5*sobely
        added = cv2.addWeighted(sobelx, 0.5, sobely, 0.5, 0)
        #cv2.imshow("added is", added)
        # pd.DataFrame(added).to_csv('sample.csv') 
        scaled_sobel = np.zeros_like(added)
        scaled_sobel[(added >=thresh_min) & (added<= thresh_max)] = 1
        #thresh = cv2.threshold(added, 10,100, cv2.THRESH_BINARY)
        #cv2.imshow("scaled sobel is", scaled_sobel*255)
        #cv2.imshow("thresh is", thresh)

        return scaled_sobel

    def color_thresh(self, img, thresh):
        """
        Convert RGB to HSL and threshold to binary image using S channel
        """
        #1. Convert the image from RGB to HSL
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #hls = img
        s = hls[:,:,1]
        h = hls[:,:,0]
        v = hls[:,:,2]

        #cv2.imshow("s is ", s)
        #
        # 
        
        
        
        
        
        
        
        
        
        #cv2.imshow("h is ", h)
        #cv2.imshow("l is ", v)

        #2. Apply threshold on S channel to get binary image
        col_v = np.zeros_like(v)
        #col_s = np.zeros_like(s)

        #h_t = np.zeros_like(h)
        #h_t[(h >=100)] = 1
        col_v[(v >=170)] = 1
        #col_s[(s <=15)] = 1

        #thresh = cv2.inRange(s,50,150)
        cv2.imshow("v thresh is ", 255*col_v)
        #cv2.imshow("s thresh is ", 255*col_s)
        
        
        return col_v

    def combinedBinaryImage(self, img):
        # Get combined binary image from color filter and sobel filter
        # """
        #1. Apply sobel filter and color filter on input image
        SobelOutput = self.gradient_thresh(img, thresh_min=10, thresh_max=100)
        ColorOutput = self.color_thresh(img, thresh=(80, 150))
        #2. Combine the outputs
        ## Here you can use as many methods as you want.

        ## TODO
        binaryImage = np.zeros_like(SobelOutput)
        binaryImage[(ColorOutput==1)|(SobelOutput==1)] = 1
        # Remove noise from binary image
        #binaryImage = morphology.remove_small_objects(binaryImage.astype('bool'),min_size=50,connectivity=2)
        #
        #
        # cv2.imshow("combined is ", 255*ColorOutput)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        return binaryImage




    def perspective_transform(self, img, verbose=False):
        """
        Get bird's eye view from input image
        """
        #1. Visually determine 4 source points and 4 destination points
        #rosbag1 points
        mg = np.uint8(img)
        dim= np.shape(img)
        pt_A = [140, dim[1]]
        pt_B = [900,dim[1]]
        pt_C = [680, 220]
        pt_D = [450, 220]
        input_pts = np.float32([pt_A, pt_B, pt_C, pt_D])
        dim= np.shape(img)
        pt_A1  = [0, dim[1]]
        pt_B1  = [dim[0],dim[1]]
        pt_C1  = [dim[0], 0]
        pt_D1  = [0, 0]
        output_pts = np.float32([pt_A1, pt_B1, pt_C1, pt_D1])
        #2. Get M, the transform matrix, and Minv, the inverse using cv2.getPerspectiveTransform()
        M = cv2.getPerspectiveTransform(input_pts,output_pts)
        #3. Generate warped image in bird view using cv2.warpPerspective()
        warped_img = cv2.warpPerspective(img,M,(640, 480),flags=cv2.INTER_LINEAR)
        Minv = np.linalg.inv(M)

        return warped_img, M, Minv


    def detection(self, img):

        binary_img = self.combinedBinaryImage(img)
        img_birdeye, M, Minv = self.perspective_transform(binary_img)

        if not self.hist:
            # Fit lane without previous result
            ret = line_fit(img_birdeye)
            left_fit = ret['left_fit']
            right_fit = ret['right_fit']
            nonzerox = ret['nonzerox']
            nonzeroy = ret['nonzeroy']
            left_lane_inds = ret['left_lane_inds']
            right_lane_inds = ret['right_lane_inds']

        else:
            # Fit lane with previous result
            if not self.detected:
                ret = line_fit(img_birdeye)

                if ret is not None:
                    left_fit = ret['left_fit']
                    right_fit = ret['right_fit']
                    nonzerox = ret['nonzerox']
                    nonzeroy = ret['nonzeroy']
                    left_lane_inds = ret['left_lane_inds']
                    right_lane_inds = ret['right_lane_inds']

                    left_fit = self.left_line.add_fit(left_fit)
                    right_fit = self.right_line.add_fit(right_fit)

                    self.detected = True

            else:
                left_fit = self.left_line.get_fit()
                right_fit = self.right_line.get_fit()
                ret = tune_fit(img_birdeye, left_fit, right_fit)

                if ret is not None:
                    left_fit = ret['left_fit']
                    right_fit = ret['right_fit']
                    nonzerox = ret['nonzerox']
                    nonzeroy = ret['nonzeroy']
                    left_lane_inds = ret['left_lane_inds']
                    right_lane_inds = ret['right_lane_inds']

                    left_fit = self.left_line.add_fit(left_fit)
                    right_fit = self.right_line.add_fit(right_fit)

                else:
                    self.detected = False

            # Annotate original image
            bird_fit_img = None
            combine_fit_img = None
            if ret is not None:
                bird_fit_img = bird_fit(img_birdeye, ret, save_file=None)
                combine_fit_img = final_viz(img, left_fit, right_fit, Minv)
            else:
                print("Unable to detect lanes")

            return combine_fit_img, bird_fit_img


if __name__ == '__main__':
    # init arg
    
    rospy.init_node('lanenet_node', anonymous=True)
    lanenet_detector()


    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)