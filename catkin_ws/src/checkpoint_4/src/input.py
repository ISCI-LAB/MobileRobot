#!/usr/bin/env python
import rospy
from checkpoint_3.msg import encoderData
from std_msgs.msg import Int32, Float32
import wiringpi
import time

class mobile_command():
    def __init__(self):
        self.msg = encoderData()
        self.pub = rospy.Publisher('target', encoderData, queue_size=10)
        rospy.Subscriber('light_sensor', Int32, self.callback)
        rospy.Subscriber('ir_sensor',Float32, self.callback_ir)
        self.target_speed = 200
        self.light_data = 1025
        self.catch_flag = 0

    def _forward(self):
        self.msg.left_speed = self.target_speed+2
        self.msg.right_speed = self.target_speed
        self.pub.publish(self.msg)

    def _stop(self):
        self.msg.left_speed = 0
        self.msg.right_speed = 0
        self.pub.publish(self.msg)

    def _backward(self, speed = 70):
        self.msg.left_speed = -speed
        self.msg.right_speed = -speed
        self.pub.publish(self.msg)

    def _spinCL(self, speed = 70):
        self.msg.left_speed = speed
        self.msg.right_speed = -speed
        self.pub.publish(self.msg)

    def _spinCCL(self, speed = 70):
        self.msg.left_speed = -speed
        self.msg.right_speed = speed
        self.pub.publish(self.msg)

    def callback(self, data):
        self.light_data = data.data

    def callback_ir(self, data):
        self.ir_data = data.data

    def _search(self):
        step = 0
        min = 1024
        while(step<75):
            self._spinCL(70)
            step+=1
            if self.light_data <min:
                min = self.light_data
            rospy.sleep(0.05)

        step = 0
        self._stop()
        rospy.sleep(0.2)
        while(step<200):
            if (self.light_data - min)<30:
                self._stop()
                rospy.sleep(0.5)
                self._spinCCL(40)
                rospy.sleep(0.5)
                break
            self._spinCL()
            step+=1
            rospy.sleep(0.05)
        self._stop()
        rospy.sleep(0.2)
        step = 30
        while(wiringpi.digitalRead(1) == 0 and step<90):
            self.target_speed = step
            self._forward()
            step +=1
            rospy.sleep(0.03)
        

    def _collision(self):
        self._stop()
        self.target_speed = 120
        rospy.sleep(0.2)
        #left_sensor = wiringpi.digitalRead(2)
        #right_sensor = wiringpi.digitalRead(3)
        self._backward()
        rospy.sleep(1.5)
        if self.catch_flag ==0:
            self._search()
        else:
            self._findDoor()
        
    def _catch(self):
        self.catch_flag = 1
        self._stop()
        rospy.sleep(0.5)
        self.target_speed = 100
        self._findDoor()

    def _findDoor(self):
        step = 0
        while(step<70 and self.ir_data < 10):
            self._spinCCL()
            step+=1
            rospy.sleep(0.05)
        self._spinCL()
        rospy.sleep(0.2)
        self._stop()
        rospy.sleep(0.3)
        while(wiringpi.digitalRead(2)==0 or wiringpi.digitalRead(3) ==0):
            if (wiringpi.digitalRead(2) ==0 and wiringpi.digitalRead(3)==0):
                self.target_speed = 80
                self._forward()
            elif(wiringpi.digitalRead(2)==0):
                self._spinCCL()
            else:
                self._spinCL()
        self.catch_flag =0
        self._backward()
        rospy.sleep(1)
        self._stop()
        rospy.sleep(5)

def publisher():
    cmder = mobile_command()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        left_sensor = wiringpi.digitalRead(2)
        right_sensor = wiringpi.digitalRead(3)
        catch_sensor = wiringpi.digitalRead(1)

        if left_sensor==1 or right_sensor==1 :
            cmder._collision()
        else:
            cmder._forward()

        if catch_sensor:
            if cmder.catch_flag == 0:
            	cmder._catch()
        rate.sleep()


def pin_setup():
    wiringpi.wiringPiSetup()
    wiringpi.pinMode(2, 0)
    wiringpi.pinMode(3, 0)
    wiringpi.pinMode(1, 0)

def main():
    rospy.init_node("publisher")
    pin_setup()
    publisher()
    rospy.spin()

if __name__ =="__main__":
    main()
