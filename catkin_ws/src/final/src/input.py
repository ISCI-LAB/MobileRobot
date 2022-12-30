#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

def publisher():
    pub = rospy.Publisher('state', Int32, queue_size = 10)
    rospy.init_node('user_input')
    while not rospy.is_shutdown():
        try:
            state = int(input("command state:"))
        except:
            pass
        pub.publish(state)

        rospy.sleep(0.5)

if __name__ == "__main__":
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

