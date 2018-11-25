#!/usr/bin/env python3

import numpy as np
import cv2
import matplotlib.pyplot as plt
import time
import os

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
import PID 
polygons = np.array([
    #[(100,height),(width,height),(width,267),(320,170)]
    #[(30,height),(450,height),(280,130)]
    #[(120,240), (150,110),(280,120), (320,140), (320,240)]
    [(0,240),(0,100), (100,100),(280,100), (320,100), (320,240)] ##lane13
])


class detect_lane:
    def __init__(self,image):
        self.image  = image
        # self.HEIGHT = image.shape[0]
        # self.WIDTH  = image.shape[1]
        self.HEIGHT = 240
        self.WIDTH  = 320
        self.steer = 0
        self.speed = 15
        #self.rate = rospy.Rate(10)
        self.speed_pub = rospy.Publisher("Team1_speed", Float32, queue_size = 1)
        self.steer_pub = rospy.Publisher("Team1_steerAngle", Float32, queue_size = 1)
        self.xb_center = 0
        self.xt_center = 0
        self.cte = 0
    def canny(self):
            image_gray = cv2.cvtColor(self.image,cv2.COLOR_RGB2GRAY)
            blur = cv2.GaussianBlur(image_gray,(5,5),0)
            canny = cv2.Canny(blur,50,250)
            cv2.imshow('Canny',canny)
            cv2.waitKey(1)
            # k = cv2.waitKey(1)
            # if k == ord('q'): # wait for 's' key to save and exit
            #     cv2.destroyAllWindows()
            #     rospy.signal_shutdown('Exit')    
            return canny


    #masked
    def region_of_interest(self,image):
    #         height = image.shape[0]
    #         width = image.shape[1]
    #         x = [210, 550,717 ,1280];
    #         y = [720, 450, 450, 720];
    #         polygons = np.array([
    #             #[(100,height),(width,height),(width,267),(320,170)]
    #             #[(30,height),(450,height),(280,130)]
    #             [(210,720), (550,450), (717,450),(1280,720)]
    #         ])
            mask = np.zeros_like(image)
            cv2.fillPoly(mask,polygons,255)
            masked_image = cv2.bitwise_and(image,mask)
            return masked_image


    def find_slopes_lines(self):
        canny_image = self.canny()
        
        crop_image = self.region_of_interest(canny_image)
        lines = cv2.HoughLinesP(crop_image,2,np.pi/180,50,np.array([]),minLineLength=20,maxLineGap=100)
        ## y=mx+b
        line_image=np.zeros_like(self.image)
        x = lines[:,:,0:2]
        x = x.reshape(len(lines),2)
        y = lines[:,:,2:4]
        y = y.reshape(len(lines),2)
        
        #Test
    #     print(x.shape,'x shape')
    #     plt.scatter(x[:,0],x[:,1])
    #     plt.scatter(y[:,0],y[:,1])
    #     plt.imshow(image)
        
        
        slopes = []
        lines_update = []
        threshold = 0.1
        for l in range(len(lines)):
            if (y[l][0]-x[l][0] < 0.1):
                slope = 1000 # //oy
            else:
                slope = (y[l][1]-x[l][1])/(y[l][0]-x[l][0])

            if (abs(slope) > threshold):
                slopes.append(slope)
                lines_update.append(lines[l])
        return slopes,lines_update


    def split_hough_point(self,lines_update):
        try:
            lines_update = np.array(lines_update)
            #print(lines_update.shape,'update lines')
            xx = lines_update[:,:,0:2]
            xx = xx.reshape(len(lines_update),2)
            yy = lines_update[:,:,2:4]
            yy = yy.reshape(len(lines_update),2)
            return xx,yy
        except:
            return
        

    def detect_left_right(self):
        

        slopes,lines_update = self.find_slopes_lines()
        #print(slopes)
        #print(len(lines_update),'lines_update')
        try:         
            first,second = self.split_hough_point(lines_update)
        except:
            return
        center = self.image.shape[1]/2
        right_lines = []
        left_lines = []
        tag = []
        #global tagleft,tagright
        for l in range(len(lines_update)):
            #print(l,': Index')
            if ((slopes[l] > 0) and (first[l][0] > center) and (second[l][0] > center)) :
                right_lines.append(lines_update[l])
                tagright = 1;
                tagleft = 0;
            elif ((slopes[l] < 0)  and (first[l][0] < center) and (second[l][0] < center-10)) :
                left_lines.append(lines_update[l])
                tagleft = 1;
                tagright = 0;
        right_lines = np.array(right_lines)
    #     right_lines = right_lines.reshape(right_lines.shape[0],4)
        left_lines  = np.array(left_lines)
    #     left_lines  = left_lines.reshape(left_lines.shape[0],4)
        return right_lines,left_lines
    def detect(self):
        cv2.imshow('Detect',self.image)
        k = cv2.waitKey(1)
        try:
            right_lines, left_lines = self.detect_left_right()
        except:
            

            return 0
        try:
            
            first_r,second_r = self.split_hough_point(right_lines)
            first_l,second_l = self.split_hough_point(left_lines)
            # print('111111111111111111111111111111')
        except:
            return 0
        x_r = np.array((first_r[:,0],second_r[:,0]))
        x_r = x_r.reshape(x_r.shape[0]*x_r.shape[1])
        y_r = np.array((first_r[:,1],second_r[:,1]))
        y_r = y_r.reshape(y_r.shape[0]*y_r.shape[1])
        

        # print(first_l,'first')
        # print(second_l,'second')
        
        x_l = np.array((first_l[:,0],second_l[:,0]))
        y_l = np.array((first_l[:,1],second_l[:,1]))
        # print(x_l,'x_ shape')
        # print(y_l.shape,'y_ shape')
        x_l = x_l.reshape(x_l.shape[0]*x_l.shape[1])
        
        y_l = y_l.reshape(y_l.shape[0]*y_l.shape[1])
        # plt.figure()
        # plt.scatter(x_r,y_r,color='red')
        # plt.scatter(x_l,y_l,color='blue')
        # plt.imshow(self.image)
        # print(x_r.shape,'xr')
        # print(y_r,'yr')
        
        
        if (len(x_r)>0):
            pol = np.polyfit(x_r,y_r,1)
            rm  = pol[0]
            rb  = pol[1]
        else:
            rm = 1
            rb =1
            
        if (len(x_l)>0):
            pol = np.polyfit(x_l,y_l,1)
            lm  = pol[0]
            lb  = pol[1]
        else:
            lm = 1
            lb =1
        line_image=np.zeros_like(self.image)
        y1 = self.image.shape[0]
        y2 = y1/2.5
        right_x1 = (y1 - rb) / rm;
        right_x2 = (y2 - rb) / rm;
        left_x1 = (y1 - lb) / lm;
        left_x2 = (y2 - lb) / lm;
        
        polygons = np.array([
            #[(100,height),(width,height),(width,267),(320,170)]
            #[(30,height),(450,height),(280,130)]
        #   [(250, 700), (550,450), (717,450),(1280,720)]
        [(int(left_x1), int(y1)), (int(left_x2), int(y2)),(int(right_x2), int(y2)),(int(right_x1), int(y1))]
        ])
        
        #plt.figure()
        mask = np.zeros_like(self.image)
        cv2.fillPoly(mask,polygons,255)
        masked_image = cv2.bitwise_or(mask,self.image)
        #plt.imshow(masked_image, cmap='gray')
        deg_right = (np.arctan(rm)*180/np.pi)
        deg_left = (-np.arctan(lm)*180/np.pi)
        # print(deg_right,'rm')
        # print(deg_left,'lm') 
        alpha = 0.9
        x1_center = (left_x1+right_x1)/2
        x2_center = (left_x2+right_x2)/2
        self.xb_center = x1_center
        self.xt_center = x2_center
        self.cte = self.xb_center - self.WIDTH/2
        plt.scatter(x1_center,y1,color='yellow')
        plt.scatter(x2_center,y2,color='yellow')
        pol = np.polyfit((x1_center,x2_center),(y1,y2),1)
        ang_center = pol[0]
        ang_center = np.arctan(ang_center)*180/np.pi
        # print(ang_center,'angle_center')
        # print(x1_center,'x1_centner')
        #if (deg_right <=38):
        if (ang_center < 0 and ang_center > -87 ):
            #self.steer =  (alpha*(38-deg_right))
            self.steer = -abs(-90 - ang_center)
            #self.steer = 5
            #cv2.putText(masked_image, 'RIGHT {0:.2f}(L: {0:.2f}-R:{0:.2f})'.format(round(self.steer,2), round(deg_left,2),round(deg_right,2)), (00, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_4)
            cv2.putText(masked_image, 'LEFT {} '.format(self.steer), (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2,cv2.LINE_4)
            # print('turn left')
            #plt.text(140,200,'TURN LEFT {}'.format(steer)) 
        #elif (deg_left <=38):
        elif ((ang_center > 0) and (ang_center < 88) ):
            #self.steer =  -(alpha*(38-deg_left))
            self.steer = abs(90 - ang_center)
            #self.steer = 10
            #cv2.putText(masked_image, 'LEFT {0:.2f}(L: {0:.2f}-R:{0:.2f})'.format(round(self.steer,2), round(deg_left,2),round(deg_right,2)), (00, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_4)
            cv2.putText(masked_image, 'RIGHT {0:.2f} '.format(self.steer), (0, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0, 255), 2,cv2.LINE_4)
            # print('turn right') 
            #plt.text(140,200,'TURN RIGHT {}'.format(steer))
        else:
            self.steer =0
            #cv2.putText(masked_image, 'Str (L:{0:.2f}-R:{0:.2f})'.format(deg_left,deg_right), (0, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(masked_image, 'STRAIGHT', (0, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2,cv2.LINE_4)
            # print('go straight')
    #cv2.imshow('image',masked_image)
    #cv2.waitKey(0)
        if self.steer != 0:
            speed = 15
        else:
            speed = 15
        
        SPEED = Float32()
        SPEED.data = speed

        STEER = Float32()

        ###Use PID
        kp = .3
        ki = .0002
        kd = 0.02
        # control = PID.PID(kp,ki,kd)
        # control.update_error(self.cte)
        # STEER.data = -control.total_error()
        # print(self.cte,'cte')
        print(self.steer,'total error')


        #self.steer_pub.publish(self.steer)
        self.speed_pub.publish(SPEED)
        self.steer_pub.publish(STEER)
        # self.rate.sleep()
        # print(self.steer)
        #cv2.imshow('Detect',masked_image)
        cv2.imshow('Detect',masked_image)
        k = cv2.waitKey(1)
        if k == ord('q'): # wait for 's' key to save and exit
            cv2.destroyAllWindows()
            rospy.signal_shutdown('Exit')    
            
    
def main():

    # image = cv2.imread('lane2.png')
  
    
    # cv2.destroyAllWindows()
    
    # #Start
    # detect1 = detect_lane(image)
    # detect1.detect() 
    # cv2.imshow('View',image)
    # cv2.waitKey(1)



    # cap = cv2.VideoCapture('project_video.mp4')
    #cap = cv2.VideoCapture('../Image/Lane13.mp4')
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
    cap.release()
    cv2.destroyAllWindows()
        
    
if __name__ == '__main__':
    main()  