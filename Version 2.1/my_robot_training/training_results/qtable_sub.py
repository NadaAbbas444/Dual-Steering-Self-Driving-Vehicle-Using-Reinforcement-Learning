#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import json

states_sub = None
actions_sub = None
qval_sub = None

def qtable_to_json(s, a, q):
    s = s.split(',')
    a = a.split(',')
    q = q.split(',')
    d = {}
    for k in range(len(s)-1):
        d[float(q[k])] = (s[k], int(a[k]))

    with open('qtable.json', 'w') as fp:
        json.dump(d, fp)

def callback_states(data):
    global states_sub
    states_sub = data.data

def callback_actions(data):
    global actions_sub
    actions_sub = data.data

def callback_qval(data):
    global qval_sub
    qval_sub = data.data

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    while not rospy.is_shutdown():
        rospy.Subscriber("qtable/states", String, callback_states)
        rospy.Subscriber("qtable/actions", String, callback_actions)
        rospy.Subscriber("qtable/qval", String, callback_qval)
        if states_sub == None:
            pass
        else:
            qtable_to_json(states_sub, actions_sub, qval_sub)
        rospy.loginfo("### DONE ###")
        rospy.sleep(1)
