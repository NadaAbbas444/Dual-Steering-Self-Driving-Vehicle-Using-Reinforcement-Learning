#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def clbk_laser(msg):
    # 1440 / 10 = 144
    regions = [
        round(min(min(msg.ranges[0:59]), 15), 2),
        round(min(min(msg.ranges[60:119]), 15), 2),
        round(min(min(msg.ranges[120:179]), 15), 2),
        # round(min(min(msg.ranges[270:359]), 15), 2),
    ]
    rospy.loginfo(regions)

def main():
    rospy.init_node('reading_laser')
    sub = rospy.Subscriber('/my_robot_description/laser/scan', LaserScan, clbk_laser)
    rospy.spin()

if __name__ == '__main__':
    main()
