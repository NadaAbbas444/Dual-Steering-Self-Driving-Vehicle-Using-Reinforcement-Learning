#! /usr/bin/env python


import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetModelStateRequest
rospy.init_node('odom_pub')

odom_pub=rospy.Publisher ('/odom', Odometry)

rospy.wait_for_service ('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

odom=Odometry()
header = Header()
header.frame_id='/odom'

model = GetModelStateRequest()
model.model_name='my_robot'

r = rospy.Rate(10)

while not rospy.is_shutdown():
    result = get_model_srv(model)

    odom.pose.pose.position = result.pose.position
    odom.pose.pose.orientation.z = result.pose.orientation.z

    #odom.twist.twist = result.twist

    header.stamp = rospy.Time.now()
    odom.header = header

    odom_pub.publish (odom)

    r.sleep() 
