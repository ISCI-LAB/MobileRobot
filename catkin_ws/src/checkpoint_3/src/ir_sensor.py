#!/usr/bin/env python
import rospy
import wiringpi
from std_msgs.msg import Int32, Float64

class ir_sensor():
    def __init__(self):
        self.pub = rospy.Publisher('door', Float64, queue_size = 1)
        self.cnt = 0
        self.cnt_1 = 0
        self.cnt_0 = 0
    def read_door(self):
        val = wiringpi.digitalRead(4)
        self.cnt+=1
        if val == 1:
            self.cnt_1+=1
        else:
            self.cnt_0+=1
        if self.cnt ==120:
            self.pub.publish(self.cnt_0*100/(self.cnt_0+self.cnt_1))
            self.cnt = 0
            self.cnt_1 = 0
            self.cnt_0 = 0
def pin_setup():
    wiringpi.wiringPiSetup()
    wiringpi.pinMode(4, 0)

def publisher():
    dctor = ir_sensor()
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        dctor.read_door()
        rate.sleep()

def main():
    rospy.init_node("ir_sensor")
    pin_setup()
    publisher()

if __name__ =="__main__":
    main()

