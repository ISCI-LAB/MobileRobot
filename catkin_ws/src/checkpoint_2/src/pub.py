#!/usr/bin/env python
import rospy
#from std_msgs.msg import *
from checkpoint_2.msg import target_speed

def publisher():
    pub = rospy.Publisher('target', target_speed, queue_size=10)
    rospy.init_node("publisher")
    while not rospy.is_shutdown():
        left_v = int(input("left wheel speed:"))
	right_v = int(input("right wheel speed:"))
	msg = target_speed()
        msg.left_speed = left_v
	msg.right_speed = right_v
	
        pub.publish(msg)
        rospy.sleep(1)

        #rospy.loginfo(num_to_pub)

if __name__ =="__main__":
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
