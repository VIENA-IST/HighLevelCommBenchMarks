#!/usr/bin/env python

import rospy
from std_msgs.msg import String

RATES = [1000, 100, 10] # hz
N_MESSAGES = 1000

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    
    for r in RATES:

        rate = rospy.Rate(r) 

        for i in range(N_MESSAGES):
            pub.publish("test")
            rate.sleep()

            if rospy.is_shutdown():
                break

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass