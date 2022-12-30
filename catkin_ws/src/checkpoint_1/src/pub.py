#!/usr/bin/env python
import rospy
from std_msgs.msg import *

def publisher():
    pub = rospy.Publisher('numbers', Int32, queue_size=10)
    rospy.init_node("publisher")
    while not rospy.is_shutdown():
        num_to_pub =int(input("user's input is "))
        pub.publish(num_to_pub)
        rospy.sleep(1)

        #rospy.loginfo(num_to_pub)

if __name__ =="__main__":
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
