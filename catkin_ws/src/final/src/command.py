#!/usr/bin/env python
import rospy
from checkpoint_3.msg import encoderData
from std_msgs.msg import Int32, Float32
import wiringpi

class mobile_command():
    def __init__(self):
        self.msg = encoderData()
        self.pub = rospy.Publisher('target', encoderData, queue_size=10)
        #rospy.Subscriber('light_sensor', Int32, self.callback)
        #rospy.Subscriber('ir_sensor',Float32, self.callback_ir)
        rospy.Subscriber('camera', Int32, self.callback_gesture)
        rospy.Subscriber('state', Int32, self.callback_state)
        rospy.Subscriber('pink', Int32, self.callback_pink)
        rospy.Subscriber('green', Int32, self.callback_green)
        self.state = 0
        self.target_speed = 130
        self.light_data = 1025
        self.current = 0
        self.cnt = 0

    def _forward(self):
        self.msg.left_speed = self.target_speed
        self.msg.right_speed = self.target_speed
        self.pub.publish(self.msg)

    def _stop(self):
        self.msg.left_speed = 0
        self.msg.right_speed = 0
        self.pub.publish(self.msg)

    def _backward(self, speed = 100):
        self.msg.left_speed = -speed
        self.msg.right_speed = -speed
        self.pub.publish(self.msg)

    def _spinCL(self, speed = 50):
        self.msg.left_speed = speed
        self.msg.right_speed = -speed
        self.pub.publish(self.msg)

    def _spinCCL(self, speed = 50):
        self.msg.left_speed = -speed
        self.msg.right_speed = speed
        self.pub.publish(self.msg)

    def callback(self, data):
        self.light_data = data.data

    def callback_ir(self, data):
        self.ir_data = data.data
    
    def callback_state(self, data):
        self.state = data.data

    def callback_gesture(self, data):
        if data.data != self.current:
            self.current = data.data
            self.cnt = 0
        else:
            self.cnt +=1

        if self.cnt >=5:
            self.state = self.current
            self.cnt = 0
    
    def callback_pink(self, data):
        self.pink = data.data

    def callback_green(self, data):
        self.green = data.data

    def _searchColor(self, color):
        assert color =='pink' or color =='green'
        if color =='pink':
            while(self.pink == 0):
                self._spinCL()

        else:
            while(self.green == 0):
                self._spinCL()

def publisher():
    cmder = mobile_command()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        left_sensor = wiringpi.digitalRead(2)
        right_sensor = wiringpi.digitalRead(3)
        catch_sensor = wiringpi.digitalRead(1)
        #initialize stop configuration
        if cmder.state == 0:
            cmder._stop()
        
        elif cmder.state == 1:
            cmder._forward()
            rospy.sleep(1)
            cmder.state = 0

        elif cmder.state == 2:
            cmder._searchColor('green')
            cmder._stop()
            rospy.sleep(0.2)
            cmder._spinCCL()
            rospy.sleep(0.7)
            cmder._stop()
            rospy.sleep(0.5)
            while catch_sensor==0:
                cmder._forward()
                catch_sensor = wiringpi.digitalRead(1)
            cmder._stop()
            rospy.sleep(1)
            cmder._searchColor('pink')
            cmder._stop()
            rospy.sleep(0.2)
            cmder._spinCCL()
            rospy.sleep(0.5)
            cmder._stop()
            rospy.sleep(0.5)

            while (right_sensor==0 and left_sensor==0):
                cmder._forward()
                right_sensor = wiringpi.digitalRead(3)
                left_sensor = wiringpi.digitalRead(2)
            cmder.state = 0
        
        elif cmder.state ==3:
            cmder._spinCL()
            rospy.sleep(1)
            cmder.state = 0
        elif cmder.state == 4:
            cmder._spinCCL()
            rospy.sleep(1)
            cmder.state = 0
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
