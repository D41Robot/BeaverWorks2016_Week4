#!/usr/bin/env python
import cv2
import numpy as np
import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, String
#from beaverworks77.msg import blob as BlobMsg

from cv_bridge import CvBridge, CvBridgeError
import threading

import os

import time

class ColorTracker:
    def __init__(self, debugging):
        self.node_name = "ColorTracker"
        self.thread_lock = threading.Lock()
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color",\
                Image, self.cbImage, queue_size=1)
        self.pub_image = rospy.Publisher("~echo_image",\
                Image, queue_size=1)
	RGB = None
	self.shape = 'none'

        #self.pub_detection = rospy.Publisher("/detection", BlobMsg, queue_size=10)

        self.debugging = debugging
	
        self.bridge = CvBridge()
	self.redImageCount = 1
	self.greenImageCount = 1
	self.yellowImageCount = 1
	self.blueImageCount = 1
	self.pinkImageCount = 1

        rospy.loginfo("[%s] Initialized." %(self.node_name))
		
	#os.mkdir(~/racecar-ws/challenge_photos)
	self.pub_type = rospy.Publisher("/exploration_challenge", String, queue_size=10)

    def cbImage(self,image_msg):
	time.sleep(4) #Wait some bit
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()

    def detection(self, img):
	RGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
#CHECK GREEN
        green_lower = np.array([54, 30, 60])
        green_upper = np.array([72, 255, 255])
        ret = self.detect_color_blob(img, green_lower, green_upper)
        color_code = 1
	if ret != None: #green found
	    os.chdir('/home/racecar/challenge_photos/')
	    GreenName = "Green" + str(self.greenImageCount) + ".png"
	    cv2.imwrite(GreenName, img)
	    self.greenImageCount += 1
	    self.pub_type.publish("Green " + self.shape + " " + GreenName)

        if ret == None: #CHECK RED
            red_lower = np.array([0, 160, 130])
            red_upper = np.array([15, 255, 255])
            ret = self.detect_color_blob(img, red_lower, red_upper)
	    if ret != None: #red found
	        os.chdir('/home/racecar/challenge_photos/')
		RedName = "Red" + str(self.redImageCount) + ".png"
		cv2.imwrite(RedName, img)
		self.redImageCount += 1
		self.pub_type.publish("Red " + self.shape + " " + RedName)
        
	if ret == None: #CHECK Yellow
            yellow_lower = np.array([40, 85, 50])
            yellow_upper = np.array([55, 200, 200])
            ret = self.detect_color_blob(img, yellow_lower, yellow_upper)
	    if ret != None: #yellow found
	    	os.chdir('/home/racecar/challenge_photos/')
	    	YellowName = "Yellow" + str(self.yellowImageCount) + ".png"
	    	cv2.imwrite(YellowName, img)
	    	self.yellowImageCount += 1
	   	self.pub_type.publish("Yellow " + self.shape + " " + YellowName)
				
        if ret == None: #CHECK blue
            blue_lower = np.array([200, 30, 30])
            blue_upper = np.array([220, 200, 200])
            ret = self.detect_color_blob(img, blue_lower, blue_upper)
	    if ret != None: #blue found
	        os.chdir('/home/racecar/challenge_photos/')
	        BlueName = "Blue" + str(self.blueImageCount) + ".png"
	        cv2.imwrite(BlueName, img)
	        self.blueImageCount += 1
	        self.pub_type.publish("Blue " + self.shape + " " + BlueName)
				
        if ret == None: #CHECK PINK
            pink_lower = np.array([300, 40, 80])
            pink_upper = np.array([310, 200, 200])
            ret = self.detect_color_blob(img, pink_lower, pink_upper)
	    if ret != None: #pink found
	        os.chdir('/home/racecar/challenge_photos/')
	        Kitty = "Image" + str(self.pinkImageCount)
	        cv2.imwrite(Kitty, img)
	        self.pinkImageCount += 1
	        self.pub_type.publish("Image" + self.shape + " " + Kitty)
            color_code = 2

        if ret == None: #if no blob was found
            cx = 0
            cy = 0
            area = 0
            color_code = 0
#        else: #blob was found
#            cx, cy, area = ret

#        msg = BlobMsg()
#        msg.area = area
#        msg.x = cx
#        msg.target = color_code

        #print(msg)
        #self.pub_detection.publish(msg)
        # publish message

    def detect_color_blob(self, img, lower, upper):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)

        mask = cv2.erode(mask, (3,3), iterations=1)

        contours, h = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	

        if self.debugging:
            cv2.drawContours(img, contours, -1, (0, 0, 255), 2)

        sorted_contours = sorted(contours, key = lambda c: cv2.contourArea(c), reverse=True)

        if len(sorted_contours) < 1:
            return None

        c = sorted_contours[0]

        area = cv2.contourArea(c)
        if area < 1000: # minimum area threshold
            return None

        perim = cv2.arcLength(c, True) # perimeter
        approx = cv2.approxPolyDP(c, 0.05 * perim, True)

        if len(approx) == 4:
            self.shape = 'square'
	elif len(approx) == 12:
	    self.shape = 'cross'
	elif len(approx) > 12:
	    self.shape = 'circle'
	else:
	    return None


        if self.debugging:
            cv2.drawContours(img, [c], -1, (255, 0, 0), 3) 
            cv2.drawContours(img, [approx], -1, (0, 255, 0), 5) 

            coord = (approx[0][0][0], approx[0][0][1])
            cv2.putText(img, "GREEN", coord, cv2.FONT_HERSHEY_PLAIN, 3, (255, 255, 255),  2)

        M = cv2.moments(approx)
        cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

        if self.debugging:
            cv2.circle(img, (cx, cy), 10, (255, 255, 255), -1)

        approx_area = cv2.contourArea(approx)

        return (cx, cy, approx_area)


    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):
            return
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)

        self.detection(image_cv)
        
        if self.debugging:
            try:
                self.pub_image.publish(\
                        self.bridge.cv2_to_imgmsg(image_cv, "bgr8"))
            except:
		pass
        self.thread_lock.release()


if __name__=="__main__":
    rospy.init_node('ColorTracker')
    e = ColorTracker(True)
    rospy.spin()
