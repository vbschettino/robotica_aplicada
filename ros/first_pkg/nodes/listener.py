#!/usr/bin/env python3

"""Print incoming messages with 'I heard:'+message."""

import rospy
from std_msgs.msg import String


def callback(msg):
    """Callback for incoming messages."""
    print("I heard: " + msg.data)


def main():
    rospy.init_node('listener')

    rospy.Subscriber('/greetings', String, callback)

    rospy.spin()


if __name__ == '__main__':
    main()
