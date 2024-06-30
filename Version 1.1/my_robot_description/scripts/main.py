#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

angle = 0.0
speed = 5.0
rate = rospy.Duration(1/10)

left_hinge_topic        = '/my_robot/left_wheel_hinge_position_controller/command'
right_hinge_topic       = '/my_robot/right_wheel_hinge_position_controller/command'

left_rotate_topic       = '/my_robot/left_wheel_rotate_position_controller/command'
right_rotate_topic      = '/my_robot/right_wheel_rotate_position_controller/command'
left_rear_rotate_topic  = '/my_robot/left_wheel_rear_rotate_position_controller/command'
right_rear_rotate_topic = '/my_robot/right_wheel_rear_rotate_position_controller/command'

# Steering
def left_hinge(event):
    pub_left_hinge.publish(angle)

def right_hinge(event):
    pub_right_hinge.publish(angle)

# Moving
def left_rotate(event):
    pub_left_rotate.publish(speed)

def right_rotate(event):
    pub_right_rotate.publish(speed)

def left_rear_rotate(event):
    pub_left_rear_rotate.publish(speed)

def right_rear_rotate(event):
    pub_right_rear_rotate.publish(speed)

if __name__ == '__main__':
    rospy.init_node('steering', anonymous=True)
    
    pub_left_hinge        = rospy.Publisher(left_hinge_topic, Float64, queue_size=10)
    pub_right_hinge       = rospy.Publisher(right_hinge_topic, Float64, queue_size=10)

    pub_left_rotate       = rospy.Publisher(left_rotate_topic, Float64, queue_size=10)
    pub_right_rotate      = rospy.Publisher(right_rotate_topic, Float64, queue_size=10)
    pub_left_rear_rotate  = rospy.Publisher(left_rear_rotate_topic, Float64, queue_size=10)
    pub_right_rear_rotate = rospy.Publisher(right_rear_rotate_topic, Float64, queue_size=10)

    rospy.Timer(rate, left_hinge)
    rospy.Timer(rate, right_hinge)
    
    rospy.Timer(rate, left_rotate)
    rospy.Timer(rate, right_rotate)
    rospy.Timer(rate, left_rear_rotate)
    rospy.Timer(rate, right_rear_rotate)
    
    rospy.spin()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
