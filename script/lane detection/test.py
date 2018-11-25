#! usr/bin/env python3
from time import time
import numpy as np
import cv2
import matplotlib.pyplot as plt


import rospy
import math
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32

import lane_detection as LD
from Line import Line

i = 1
def rad2deg(rad):
    return rad/np.pi*180
def deg2rad(deg):
    return deg*np.pi/180

class Detect:
    def __init__(self, img):
        self.image = img
        self.HEIGHT = img.shape[0]
        self.WIDTH = img.shape[1]
        self.cte = 0
        self.center_line = Line(self.WIDTH/2,self.HEIGHT,self.WIDTH/2,self.HEIGHT/2)
        self.angle_steer = 0
        self.speed = 10
        self.dt = 0.1
        self.yaw = 90   #  phi(center,ox)

        ##initialize MPC
        x_start = 140
        y_start = 240
        yaw = -1.4489
        v = 10

        #region of interest
        self.vertices = np.array([[(int(0*img.shape[1]),img.shape[0]),(int(0*img.shape[1]), 
                    int(0.15*img.shape[0])), (int(1.00*img.shape[1]), int(0.15*img.shape[0])), (
                    img.shape[1],img.shape[0])]], dtype=np.int32)
        #img = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
        self.final_img, self.left_line, self.right_line = LD.color_frame_pipeline([self.image], solid_lines=True)
        self.speed_pub = rospy.Publisher("Team1_speed", Float32, queue_size = 10)
        self.steer_pub = rospy.Publisher("Team1_steerAngle", Float32, queue_size = 10)
        # start_time = time()
        #self.sign_image, self.DIRECTION = TD.detect_sign(img)
        # end_time = time()
        # duration = end_time-start_time
        # print(duration,"duration")
        rate = rospy.Rate(30)
        # self.DIRECTION = 'straight' # 0 : straight, 1 rigth, -1 left
    def find_all(self):
        x_axis, y_axis = self.right_line.seperate_axis()
        pol_right = np.polyfit(x_axis,y_axis,1)
        rm  = pol_right[0]
        rb  = pol_right[1]
        x_axis, y_axis = self.left_line.seperate_axis()
        pol_left = np.polyfit(x_axis,y_axis,1)
        lm  = pol_left[0]
        lb  = pol_left[1]
        yb = self.HEIGHT
        yt = yb/3.5
        xt_r = (yt - rb) / rm;
        xb_r = (yb - rb) / rm;
        xt_l = (yt - lb) / lm;
        xb_l = (yb - lb) / lm;
        xb_center = (xb_r + xb_l)/2
        xt_center = (xt_r + xt_l)/2
        self.center_line = Line(xb_center,yb,xt_center,yt)
        self.left_line = Line(xb_l,yb,xt_l,yt)
        self.right_line = Line(xb_r,yb,xt_r,yt)

        self.yaw = np.arctan(self.center_line.compute_slope())
        self.center_line.draw(self.final_img,(255,0,0))
        self.left_line.draw(self.final_img,(0,255,0))
        self.right_line.draw(self.final_img,(0,255,0))
        self.cte = self.center_line.x2 - self.center_line.x1
        
        
    def drive(self):
        return
        
def main():
    rospy.init_node('test')

    s = time()
    # img = cv2.imread('Image/right3.')
    img = cv2.imread('../Image/right3.png')
    detect = Detect(img)
    detect.drive()
    e = time()
    print(e-s,'duration')
    
    # cap = cv2.VideoCapture('video/view_turn_right.mp4')
    # # cap = cv2.VideoCapture('detect_lane.mp4')
    # while (cap.isOpened()):
    #     ret, frame = cap.read()
    #     detect = Detect(frame)
    #     detect.drive()
    #     cv2.imshow('View',frame)
    #     k = cv2.waitKey(1)
    #     if k == ord('q'): # wait for 's' key to save and exit
    #         cv2.destroyAllWindows()
    #         rospy.signal_shutdown('Exit')    
        
    # cap.release()
    cv2.destroyAllWindows()
if __name__ == '__main__':
    main()