#!/usr/bin/env python

import fileinput as fi
import rospy
from std_msgs.msg import String


class Chat:


    def __init__(self):
        self.pub = rospy.Publisher('qr_reader', String, queue_size=10)
        rospy.init_node('qr_reader', anonymous=True)


    def say(self,string):
        rospy.loginfo(string)
        self.pub.publish(string)

def std_talker():

    rospy.init_node('testnode')
    pub = rospy.Publisher('job/add', String, queue_size=10)

    for line in fi.input():

        if not rospy.is_shutdown():

            rospy.loginfo(line)
            pub.publish(line)
        else:
            break

if __name__ == '__main__':
    try:
        std_talker()
    except rospy.ROSInterruptException:
        pass
