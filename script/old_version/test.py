#!/usr/bin/env python3
import sys, time
import numpy as np
#from scipy.ndimage import filters
import cv2
import roslib
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32



import main2_dev as Detect
#import main1_stable as Detect
###########################################################################
class image_feature:
    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # self.speed_pub = rospy.Publisher("Team1_speed", Float32, queue_size = 10)

        # self.steer_pub = rospy.Publisher("Team1_steerAngle", Float32, queue_size = 10)
        self.subscriber = rospy.Subscriber("Team1_image/compressed",
                                    CompressedImage, self.callback)


    def callback(self, ros_data):
        '''Callback function of subscribed topic.  Here images get converted and features detected'''
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)#cv2.CV_LOAD_IMAGE_COLOR)
        cv2.imshow("output", image_np)
        
        
        detect1 = Detect.detect_lane(image_np)
        detect1.detect()
        # if steer == 0:
        #     speed = 15
        # else: 
        #     speed = 10
        # #speed
        # #speed = 15

        # self.speed_pub.publish(speed)        

        # #steer
        # #steer = 0
        # self.steer_pub.publish(steer)

        # rospy.loginfo(str(steer))
        # rospy.loginfo(str(speed))
        #cv2.namedWindow("output", cv2.WINDOW_NORMAL)
        #image_np = cv2.resize(image_np, (1280, 720))
        # cv2.imshow("output", image_np)
        # cv2.waitKey(0)
        k = cv2.waitKey(1)
        if k == ord('q'):         # wait for ESC key to exit
            cv2.destroyAllWindows()
            rospy.signal_shutdown('Exit')
    
def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('image_feature')
    ic = image_feature()
    
    try:
        rospy.spin()    
            
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)