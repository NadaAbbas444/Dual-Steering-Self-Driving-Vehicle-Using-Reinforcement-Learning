#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def clbk_laser(msg):
    # 1440 / 10 = 144
    regions = [
        min(min(msg.ranges[0:143]), 15),
        min(min(msg.ranges[144:287]), 15),
        min(min(msg.ranges[288:431]), 15),
        min(min(msg.ranges[432:575]), 15),
        min(min(msg.ranges[576:719]), 15),
        min(min(msg.ranges[720:863]), 15),
        min(min(msg.ranges[864:1007]), 15),
        min(min(msg.ranges[1008:1151]), 15),
        min(min(msg.ranges[1152:1295]), 15),
        min(min(msg.ranges[1296:1439]), 15),
    ]
    rospy.loginfo(regions)

def main():
    rospy.init_node('reading_laser')
    sub = rospy.Subscriber('/my_robot_description/laser/scan', LaserScan, clbk_laser)
    rospy.spin()

if __name__ == '__main__':
    main()
