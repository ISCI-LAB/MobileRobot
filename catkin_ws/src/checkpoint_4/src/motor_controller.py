#!/usr/bin/env python
import rospy
import numpy as np
from checkpoint_3.msg import encoderData

class Controller:
    def __init__(self):
        self.acc_error_R = []
        self.acc_error_L = []
        self.pre_error_R = 0
        self.pre_error_L = 0
        self.time_step = 14

    def pid_R(self, target , current):
        kp = 1.8
        ki = 0.01
        kd = 0.08
        error = current - target
        self.acc_error_R.append(error)
        try:
            error_i = np.mean(self.acc_error_R)
        except:
            error_i = 0

        feedback = kp*error + ki*error_i+kd*self.pre_error_R
        self.pre_error_R = error
        if len(self.acc_error_R) > self.time_step:
            self.acc_error_R.pop(0)
        return feedback

    def pid_L(self, target, current):
        kp = 1.4
        ki = 0.005
        kd = 0.06
        error = current - target
        self.acc_error_L.append(error)
        try:
            error_i = np.mean(self.acc_error_L)
        except:
            error_i = 0

        feedback = kp*error + ki*error_i+kd*self.pre_error_L
        self.pre_error_L = error
        if len(self.acc_error_L) > self.time_step:
            self.acc_error_L.pop(0)
        return feedback

def sat(input):
    bound = 230
    if input > bound:
        input = bound
    elif input < 0:
        input = 0
    return input

ref_left = 0
ref_right = 0
controller = Controller()

def callback_input(data):
    controller.acc_error_R = []
    controller.acc_error_L = []
    controller.pre_error_R = 0
    controller.pre_error_L = 0
    global ref_left
    ref_left = data.left_speed
    global ref_right
    ref_right  = data.right_speed

def callback_control(data):
    pub = rospy.Publisher('command', encoderData, queue_size = 10)
    current_left = data.left_speed
    current_right = data.right_speed
    output = encoderData()
    if (ref_left !=0 or ref_right !=0):
        cmd_L = ref_left - controller.pid_L(ref_left, current_left)
        cmd_R = ref_right - controller.pid_R(ref_right, current_right)
        output.left_speed = cmd_L
        output.right_speed = cmd_R-2
    else:
        output.left_speed = 0
        output.right_speed = 0
    pub.publish(output)

def main():
    rospy.init_node('control_node')
    rospy.Subscriber('target', encoderData, callback_input)
    rospy.Subscriber('encoder_output', encoderData, callback_control)
    rospy.spin()

if __name__ == "__main__":
    main()
