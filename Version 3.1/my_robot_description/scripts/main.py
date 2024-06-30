#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

angle = 0.1
speed = 10.0
rate = rospy.Duration(1/10)

front_left_wheel_revolute_topic  = '/my_robot/front_left_wheel_revolute_controller/command'
front_right_wheel_revolute_topic = '/my_robot/front_right_wheel_revolute_controller/command'
back_left_wheel_revolute_topic   = '/my_robot/back_left_wheel_revolute_controller/command'
back_right_wheel_revolute_topic  = '/my_robot/back_right_wheel_revolute_controller/command'

front_left_wheel_rotate_topic  = '/my_robot/front_left_wheel_rotate_controller/command'
front_right_wheel_rotate_topic = '/my_robot/front_right_wheel_rotate_controller/command'
back_left_wheel_rotate_topic   = '/my_robot/back_left_wheel_rotate_controller/command'
back_right_wheel_rotate_topic  = '/my_robot/back_right_wheel_rotate_controller/command'

# Steering
def front_left_revolute(event):
    pub_front_left_wheel_revolute.publish(angle)

def front_right_revolute(event):
    pub_front_right_wheel_revolute.publish(angle)
    
def back_left_revolute(event):
    pub_back_left_wheel_revolute.publish(-angle/3)
    
def back_right_revolute(event):
    pub_back_right_wheel_revolute.publish(-angle/3)

# Moving
def front_left_rotate(event):
    pub_front_left_wheel_rotate.publish(speed)

def front_right_rotate(event):
    pub_front_right_wheel_rotate.publish(speed)

def back_left_rotate(event):
    pub_back_left_wheel_rotate.publish(speed)

def back_right_rotate(event):
    pub_back_right_wheel_rotate.publish(speed)

if __name__ == '__main__':
	rospy.init_node('steering', anonymous=True)
	    
	pub_front_left_wheel_revolute  = rospy.Publisher(front_left_wheel_revolute_topic, Float64, queue_size=10)
	pub_front_right_wheel_revolute = rospy.Publisher(front_right_wheel_revolute_topic, Float64, queue_size=10)
	pub_back_left_wheel_revolute   = rospy.Publisher(back_left_wheel_revolute_topic, Float64, queue_size=10)
	pub_back_right_wheel_revolute  = rospy.Publisher(back_right_wheel_revolute_topic, Float64, queue_size=10)

	pub_front_left_wheel_rotate  = rospy.Publisher(front_left_wheel_rotate_topic, Float64, queue_size=10)
	pub_front_right_wheel_rotate = rospy.Publisher(front_right_wheel_rotate_topic, Float64, queue_size=10)
	pub_back_left_wheel_rotate   = rospy.Publisher(back_left_wheel_rotate_topic, Float64, queue_size=10)
	pub_back_right_wheel_rotate  = rospy.Publisher(back_right_wheel_rotate_topic, Float64, queue_size=10)

	rospy.Timer(rate, front_left_revolute)
	rospy.Timer(rate, front_right_revolute)
	rospy.Timer(rate, back_left_revolute)
	rospy.Timer(rate, back_right_revolute)

	rospy.Timer(rate, front_left_rotate)
	rospy.Timer(rate, front_right_rotate)
	rospy.Timer(rate, back_left_rotate)
	rospy.Timer(rate, back_right_rotate)
	    
	rospy.spin()
