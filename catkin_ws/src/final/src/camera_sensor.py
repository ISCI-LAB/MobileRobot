#!/usr/bin/env python3

import rospy
import cv2
from std_msgs.msg import Int32
import numpy as np
import math
def findColor(image, color):

    detect = 0
    roi = image[30:450, 30:610]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    if color == 'pink':
        lower = np.array([144,50,75], dtype = np.uint8)
        upper = np.array([179,250,250], dtype = np.uint8)
    else:
        lower = np.array([49, 28, 15], dtype = np.uint8)
        upper = np.array([67,250,250], dtype = np.uint8)

    mask = cv2.inRange(hsv, lower, upper)

    #define kernel size  
    kernel = np.ones((5,5),np.uint8)

    # Remove unnecessary noise from mask

    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    _, contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    try:
        cnt = max(contours, key = lambda x: cv2.contourArea(x))
        if cv2.contourArea(cnt) >5000:
            detect = 1
    except:
        pass

    return detect


def gesture(image):
    kernel = np.ones((3,3),np.uint8)
    roi = image[30:450, 30:610]
    #define region of interest
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
     
    # define range of skin color in HSV
    lower_skin = np.array([0,20,70], dtype=np.uint8)
    upper_skin = np.array([25,255,255], dtype=np.uint8)
        
     #extract skin colur imagw  
    mask = cv2.inRange(hsv, lower_skin, upper_skin) 
        
    #extrapolate the hand to fill dark spots within
    mask = cv2.dilate(mask,kernel,iterations = 4)
        
    #blur the image
    mask = cv2.GaussianBlur(mask,(5,5),100) 

    #find contours
    _, contours,hierarchy= cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
   #find contour of max area(hand)
    try:
        cnt = max(contours, key = lambda x: cv2.contourArea(x))
    except:
        return 0
    #approx the contour a little
    epsilon = 0.0005*cv2.arcLength(cnt,True)
    approx= cv2.approxPolyDP(cnt,epsilon,True)
        
    #make convex hull around hand
    hull = cv2.convexHull(cnt)
     #define area of hull and area of hand
    areahull = cv2.contourArea(hull)
    areacnt = cv2.contourArea(cnt)
    #find the percentage of area not covered by hand in convex hull
    arearatio=((areahull-areacnt)/areacnt)*100
    
     #find the defects in convex hull with respect to hand
    hull = cv2.convexHull(approx, returnPoints=False)
    defects = cv2.convexityDefects(approx, hull)
        
    # l = no. of defects
    l=0
        
    #code for finding no. of defects due to fingers
    if defects is not None:
        for i in range(defects.shape[0]):
            s,e,f,d = defects[i,0]
            start = tuple(approx[s][0])
            end = tuple(approx[e][0])
            far = tuple(approx[f][0])
            pt= (100,180)
            
            
            # find length of all sides of triangle
            a = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
            b = math.sqrt((far[0] - start[0])**2 + (far[1] - start[1])**2)
            c = math.sqrt((end[0] - far[0])**2 + (end[1] - far[1])**2)
            s = (a+b+c)/2
            ar = math.sqrt(s*(s-a)*(s-b)*(s-c))
            
            #distance between point and convex hull
            d=(2*ar)/a
            
            # apply cosine rule here
            angle = math.acos((b**2 + c**2 - a**2)/(2*b*c)) * 57
            
        
            # ignore angles > 90 and ignore points very close to convex hull(they generally come due to noise)
            if angle <= 90 and d>30:
                l += 1
                #cv2.circle(roi, far, 3, [255,0,0], -1)
            
            #draw lines around hand
            #cv2.line(roi,start, end, [0,255,0], 2)
    return l
    

def publisher():
    pub = rospy.Publisher('camera', Int32, queue_size = 5)
    pub_green = rospy.Publisher('green', Int32, queue_size = 3)
    pub_pink = rospy.Publisher('pink', Int32, queue_size = 3)
    rospy.init_node('gesture')
    rate = rospy.Rate(20)
    cap = cv2.VideoCapture(0)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if frame is not None:
            result = gesture(frame)
            pink_result = findColor(frame, 'pink') 
            green_result = findColor(frame, 'green')
            pub.publish(result)
            pub_pink.publish(pink_result)
            pub_green.publish(green_result)
            #cv2.imshow("frame", frame)
            #cv2.waitKey(1)
        rate.sleep()

if __name__ =="__main__":
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
