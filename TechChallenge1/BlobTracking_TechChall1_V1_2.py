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
	self.lastColor = "start"

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
	ret = None
	biggest_blob = (0,None)
	Name = None
        green_lower = np.array([54, 30, 60])
        green_upper = np.array([72, 255, 255])
#            red_lower = np.array([0, 160, 130])
#            red_upper = np.array([15, 255, 255])
	red_lower = np.array([175, 25, 45])
        red_upper = np.array([180, 35, 50])
        yellow_lower = np.array([20, 50, 25])
        yellow_upper = np.array([25, 100, 100])
        blue_lower = np.array([100, 15, 15])
        blue_upper = np.array([120, 100, 100])
        pink_lower = np.array([150, 20, 40])
        pink_upper = np.array([155, 100, 100])

	if self.lastColor != "green":
            ret = self.detect_color_blob(img, green_lower, green_upper,"green")
	    if ret != None: #green found
		if biggest_blob[0] < ret[2]:
	    	    Name = "green" + self.shape + str(self.greenImageCount) + ".png"
		    biggest_blob = (ret[2],"green")
        if self.lastColor != "red": #CHECK RED
            ret = self.detect_color_blob(img, red_lower, red_upper,"red")
	    if ret != None: #red found
	     	if biggest_blob[0] < ret[2]:
		    Name = "red" + self.shape + str(self.redImageCount) + ".png"
		    biggest_blob = (ret[2],"red")
	if self.lastColor != "yellow": #CHECK Yellow
            ret = self.detect_color_blob(img, yellow_lower, yellow_upper,"yellow")
	    if ret != None: #yellow found
		if biggest_blob[0] < ret[2]:
	    	    Name = "yellow" + self.shape + str(self.yellowImageCount) + ".png"
		    biggest_blob = (ret[2],"yellow")	
        if self.lastColor != "blue": #CHECK blue
            ret = self.detect_color_blob(img, blue_lower, blue_upper,"blue")
	    if ret != None: #blue found
	        if biggest_blob[0] < ret[2]:
		    Name = "Blue" + self.shape + str(self.blueImageCount) + ".png"
		    biggest_blob = (ret[2],"blue")				
        if self.lastColor != "pink": #CHECK PINK
            ret = self.detect_color_blob(img, pink_lower, pink_upper,"pink")
	    if ret != None: #pink found
	        Name = "Image" + str(self.pinkImageCount)
	        biggest_blob = (ret[2],"pink")

	cv2.imwrite(greenName, img)

	if biggest_blob[1] =='green':
	    self.greenImageCount += 1
	elif biggest_blob[1] =='red':
	    self.redImageCount += 1
	elif biggest_blob[1] =='yellow':
	    self.yellowImageCount += 1
	elif biggest_blob[1] == 'blue':
	    self.blueImageCount += 1
	elif biggest_blob == 'pink':
	    self.pinkImageCount += 1

	if ret != None:
	    os.chdir('/home/racecar/challenge_photos/')
	    cv2.imwrite(Name, img)
	    self.pub_type.publish(Name)
	    self.lastColor = biggest_blob[1]

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

        #if self.debugging:
         #   cv2.drawContours(img, contours, -1, (0, 0, 255), 2)

        sorted_contours = sorted(contours, key = lambda c: cv2.contourArea(c), reverse=True)

        if len(sorted_contours) < 1:
            return None

        c = sorted_contours[0]

        area = cv2.contourArea(c)
        if area < 1000: # minimum area threshold
	    cv2.drawContours(img, c, -1, (0,0,255), 2)
            return None

        perim = cv2.arcLength(c, True) # perimeter
        approx = cv2.approxPolyDP(c, 0.05 * perim, True)

        if len(approx) == 4:
            self.shape = 'Square'
	elif len(approx) == 12:
	    self.shape = 'Cross'
	elif len(approx) > 12:
	    self.shape = 'Circle'
	else:
	    return None


        if self.debugging:
            cv2.drawContours(img, [c], -1, (255, 0, 0), 3) 
            cv2.drawContours(img, [approx], -1, (0, 255, 0), 5) 

            coord = (approx[0][0][0], approx[0][0][1])
            cv2.putText(img, "Color", coord, cv2.FONT_HERSHEY_PLAIN, 3, (255, 255, 255),  2)

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


