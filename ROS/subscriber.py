#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import time

N_MESSAGES = 1000

TIME_START = 0
TIME_END = 0

I = 0

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global TIME_START
    global TIME_END
    global I

    msg = data.data

    if TIME_START == 0:
        TIME_START = time.time()
    
    I += 1

    if I == N_MESSAGES:
        TIME_END = time.time()

        delta = TIME_END - TIME_START

        print(N_MESSAGES/delta)

        TIME_START = 0
        TIME_END   = 0
        I = 0
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)

if __name__ == '__main__':
    listener()