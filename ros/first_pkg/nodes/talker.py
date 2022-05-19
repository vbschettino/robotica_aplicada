#!/usr/bin/env python3

"""Repeatedly count up and publish 'Hello world'+count."""

import rospy
from std_msgs.msg import String


def main():
    rospy.init_node('talker')

    pub = rospy.Publisher('/greetings', String, queue_size=10)

    count = 1

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg = String(data="Hello world " + str(count))
        pub.publish(msg)
        count += 1
        rate.sleep()


if __name__ == '__main__':
    main()
