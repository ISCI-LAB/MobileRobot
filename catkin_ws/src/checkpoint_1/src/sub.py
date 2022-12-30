#!/usr/bin/env python
import rospy
from std_msgs.msg import *

def callback(data):
    #rospy.loginfo("")
    print("message from Arduino is "+str(data.data))

def listener():
    rospy.init_node("listener")
    rospy.Subscriber("output", Int32, callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

