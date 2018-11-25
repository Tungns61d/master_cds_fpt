#!/usr/bin/env python3
import sys, time
import numpy as np
#from scipy.ndimage import filters
import cv2
import roslib
import rospy
import math
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32

import PID
##################################################333


#################################################333

class detect_lane:

    def __init__(self,image):
        #self.gray = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
        self.img = image
        # Create masked edges image
        imshape = image.shape
        self.vertices = np.array([[(int(0*imshape[1]),imshape[0]),(int(0*imshape[1]), 
                    int(0.25*imshape[0])), (int(0.80*imshape[1]), int(0.3*imshape[0])), (
                    imshape[1],imshape[0])]], dtype=np.int32)
        self.angle_center = 90 # 90 = straight -89 lech phai, 89 lech trai
        self.speed_pub = rospy.Publisher("Team1_speed", Float32, queue_size = 1)
        self.steer_pub = rospy.Publisher("Team1_steerAngle", Float32, queue_size = 1)
        self.HEIGHT = 240
        self.WIDTH  = 320
        self.steer = 0
        self.speed = 15
        self.xb_center = self.WIDTH/2
        self.xt_center = self.WIDTH/2
        self.cte = 0
#####################################33


    def grayscale(self,img):
        """Applies the Grayscale transform
        This will return an image with only one color channel
        but NOTE: to see the returned image as grayscale
        (assuming your grayscaled image is called 'gray')
        you should call plt.imshow(gray, cmap='gray')"""
        return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        # Or use BGR2GRAY if you read an image with cv2.imread()
        # return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
    def canny(self,img, low_threshold, high_threshold):
        """Applies the Canny transform"""
        return cv2.Canny(img, low_threshold, high_threshold)

    def gaussian_blur(self,img, kernel_size):
        """Applies a Gaussian Noise kernel"""
        return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

    def region_of_interest(self,img, vertices):
        """
        Applies an image mask.
        
        Only keeps the region of the image defined by the polygon
        formed from `vertices`. The rest of the image is set to black.
        """
        #defining a blank mask to start with - Make a black image of the same size
        mask = np.zeros_like(img)   
        
        #defining a 3 channel or 1 channel color to fill the mask with depending on the input image
        
        if len(img.shape) > 2:
            channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255# white
            
        #filling pixels inside the polygon defined by "vertices" with the fill color  
        # Fill the defined polygon area with white
        cv2.fillPoly(mask, vertices, ignore_mask_color)
        
        #returning the image only where mask pixels are nonzero
        # Will return only the region of interest
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image


    def draw_lines(self,img, lines, color=[255, 0, 0], thickness=3):
        # Function has been written to work with Challenge video as well
        # b -0, g-1, r-2 
        """
        NOTE: this is the function you might want to use as a starting point once you want to 
        average/extrapolate the line segments you detect to map out the full
        extent of the lane (going from the result shown in raw-lines-example.mp4
        to that shown in P1_example.mp4).  
        
        Think about things like separating line segments by their 
        slope ((y2-y1)/(x2-x1)) to decide which segments are part of the left
        line vs. the right line.  Then, you can average the position of each of 
        the lines and extrapolate to the top and bottom of the lane.
        
        This function draws `lines` with `color` and `thickness`.    
        Lines are drawn on the image inplace (mutates the image).
        If you want to make the lines semi-transparent, think about combining
        this function with the weighted_img() function below
        """
        # At the bottom of the image, imshape[0] and top has been defined as 330
        imshape = img.shape 
        
        slope_left=0
        slope_right=0
        leftx=0
        lefty=0
        rightx=0
        righty=0
        i=0
        j=0
        
        
        for line in lines:
            for x1,y1,x2,y2 in line:
                slope = (y2-y1)/(x2-x1)
                if slope >0.1: #Left lane and not a straight line
                    # Add all values of slope and average position of a line
                    slope_left += slope 
                    leftx += (x1+x2)/2
                    lefty += (y1+y2)/2
                    i+= 1
                elif slope < -0.2: # Right lane and not a straight line
                    # Add all values of slope and average position of a line
                    slope_right += slope
                    rightx += (x1+x2)/2
                    righty += (y1+y2)/2
                    j+= 1
        # Left lane - Average across all slope and intercepts
        if i>0: # If left lane is detected
            avg_slope_left = slope_left/i
            avg_leftx = leftx/i
            avg_lefty = lefty/i
            # Calculate bottom x and top x assuming fixed positions for corresponding y
            xb_l = int(((int(0.97*imshape[0])-avg_lefty)/avg_slope_left) + avg_leftx)
            xt_l = int(((int(0.61*imshape[0])-avg_lefty)/avg_slope_left)+ avg_leftx)

        else: # If Left lane is not detected - best guess positions of bottom x and top x
            xb_l = int(0.21*imshape[1])
            xt_l = int(0.43*imshape[1])
        
        # Draw a line
        cv2.line(img, (xt_l, int(0.61*imshape[0])), (xb_l, int(0.97*imshape[0])), color, thickness)
        
        #Right lane - Average across all slope and intercepts
        if j>0: # If right lane is detected
            avg_slope_right = slope_right/j
            avg_rightx = rightx/j
            avg_righty = righty/j
            # Calculate bottom x and top x assuming fixed positions for corresponding y
            xb_r = int(((int(0.97*imshape[0])-avg_righty)/avg_slope_right) + avg_rightx)
            xt_r = int(((int(0.61*imshape[0])-avg_righty)/avg_slope_right)+ avg_rightx)
        
        else: # If right lane is not detected - best guess positions of bottom x and top x
            xb_r = int(0.89*imshape[1])
            xt_r = int(0.53*imshape[1])
        
        # Draw a line    
        cv2.line(img, (xt_r, int(0.61*imshape[0])), (xb_r, int(0.97*imshape[0])), color, thickness)
        
        ######
        yt = int(0.61*imshape[0])
        yt_l = yt
        yt_r = yt_l
        
        yb = int(0.97*imshape[0])
        yb_l = yb
        yb_r = yb_l
        #xb_l,yb_l,xt_l,yt_l,xb_r,yb_r,xt_r,yt_r
        xb_center = (xb_l + xb_r)/2
        xt_center = (xt_l+ xt_r)/2
        self.xb_center = xb_center
        self.xt_center = xt_center
        self.cte = self.WIDTH/2 - xb_center
        print(xb_center,'xb')
        print(xt_center,'xt')
        pol = np.polyfit((xb_center,xt_center),(yb,yt),1)
        ang_center = pol[0]
        ang_center = np.arctan(ang_center)*180/np.pi
        self.angle_center = ang_center
        print(ang_center,"angle")
        
    def hough_lines(self,img, rho, theta, threshold, min_line_len, max_line_gap):
        """
        `img` should be the output of a Canny transform.
            
        Returns an image with hough lines drawn.
        """
        lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
        line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
        self.draw_lines(line_img, lines)
        return line_img


    # Python 3 has support for cool math symbols.

    def weighted_img(self,img, initial_img, alpha=0.8, beta=1., lamda=0.):
        """
        `img` is the output of the hough_lines(), An image with lines drawn on it.
        Should be a blank image (all black) with lines drawn on it.
        
        `initial_img` should be the image before any processing.
        
        The result image is computed as follows:
        
        initial_img * alpha + img * beta + lamda
        NOTE: initial_img and img must be the same shape!
        """
        return cv2.addWeighted(initial_img, alpha, img, beta, lamda)    
    def lane_detector(self):
        gray = cv2.cvtColor(self.img, cv2.COLOR_RGB2GRAY)
        # Define a kernel size and apply Gaussian smoothing
        kernel_size = 5
        blur_gray = self.gaussian_blur(gray, kernel_size)

        # Define our parameters for Canny and apply
        low_threshold = 10
        high_threshold = 150
        edges = self.canny(blur_gray, low_threshold, high_threshold)

        # Create masked edges image
        imshape = self.img.shape
        masked_edges = self.region_of_interest(edges, self.vertices)


        # Define the Hough transform parameters and detect lines using it
        rho = 1 # distance resolution in pixels of the Hough grid
        theta = (np.pi/180) # angular resolution in radians of the Hough grid
        threshold = 15     # minimum number of votes (intersections in Hough grid cell)
        min_line_len = 60 #minimum number of pixels making up a line
        max_line_gap = 30    # maximum gap in pixels between connectable line segments

        line_img = self.hough_lines(masked_edges, rho, theta, threshold, min_line_len, max_line_gap)

        final_img = self.weighted_img(line_img, self.img, alpha=0.6, beta=1., lamda=0.)
        return edges, masked_edges, final_img
    def detect(self):
        """
            detec thanhasdasd 
        """
        edges, masked_edges, final_img = self.lane_detector()

        if (self.angle_center < 0 and self.angle_center > -83 ):
            if (self.xb_center < 145):
                self.steer = 6
            else:
            #self.steer =  (alpha*(38-deg_right))
                self.steer = -abs(-88 - self.angle_center)
            #self.steer = 5
            #cv2.putText(masked_image, 'RIGHT {0:.2f}(L: {0:.2f}-R:{0:.2f})'.format(round(self.steer,2), round(deg_left,2),round(deg_right,2)), (00, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_4)
            cv2.putText(final_img, 'LEFT {} '.format(self.steer), (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2,cv2.LINE_4)
            print('turn left')
            #plt.text(140,200,'TURN LEFT {}'.format(steer)) 
        #elif (deg_left <=38):
        elif ((self.angle_center > 0) and (self.angle_center < 83) ):
            if (self.xb_center > 145):
                self.steer = -6
            else:
            #self.steer =  -(alpha*(38-deg_left))qq
                self.steer = abs(88 - self.angle_center)
            #self.steer = 10
            #cv2.putText(masked_image, 'LEFT {0:.2f}(L: {0:.2f}-R:{0:.2f})'.format(round(self.steer,2), round(deg_left,2),round(deg_right,2)), (00, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_4)
            cv2.putText(final_img, 'RIGHT {0:.2f} '.format(self.steer), (0, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0, 255), 2,cv2.LINE_4)
            print('turn right') 
            #plt.text(140,200,'TURN RIGHT {}'.format(steer))
        else:
            self.steer =0
            #cv2.putText(masked_image, 'Str (L:{0:.2f}-R:{0:.2f})'.format(deg_left,deg_right), (0, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(final_img, 'STRAIGHT', (0, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2,cv2.LINE_4)
            print('go straight')
    #cv2.imshow('image',masked_image)
    #cv2.waitKey(0)
        if self.steer != 0:
            speed = 30
        else:
            speed = 20
        
        SPEED = Float32()
        SPEED.data = speed

        STEER = Float32()
        kp = 0.12
        ki = 0.0003
        kd = 3
        pid = PID.PID(kp,ki,kd)
        pid.update_error(self.cte)
        STEER.data = pid.total_error()
        


        #self.steer_pub.publish(self.steer)
        self.speed_pub.publish(SPEED)
        self.steer_pub.publish(STEER)

        #cv2.imshow('Detect',masked_image)
        cv2.imshow('Detect',final_img)
        k = cv2.waitKey(1)
        if k == ord('q'): # wait for 's' key to save and exit
            cv2.destroyAllWindows()
            rospy.signal_shutdown('Exit')    
        print(self.angle_center)
def main():
#####################-----Image test----###################
    #image = cv2.imread('lane2.png')
    # image = cv2.imread('../Image/center.png')
    # detect1 = detect_lane(image)
    # detect1.start() 
    # cv2.imshow('View',image)
    # cv2.waitKey(1)
    # cv2.destroyAllWindows()

############## ----Video test--- ############################
    cap = cv2.VideoCapture('project_video.mp4')
    cap = cv2.VideoCapture('../Image/Lane13.mp4')
    cap = cv2.VideoCapture('../Image/view_shaddow2.mp4')
    while (cap.isOpened()):
        ret, frame = cap.read()
        detect1 = detect_lane(frame)
        cv2.imshow('View',frame)
        k = cv2.waitKey(1)
        if k == ord('q'): # wait for 's' key to save and exit
            cv2.destroyAllWindows()
            rospy.signal_shutdown('Exit')    
        detect1.detect()
    # cap.release()
    # cv2.destroyAllWindows()
        
    
if __name__ == '__main__':
    main()  