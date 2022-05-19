#!/usr/bin/env python3

"""Republish incoming messages copied n times."""

import rospy
from std_msgs.msg import String


def callback(msg, callback_args):
    """"Callback for incoming messages."""
    publisher, num_echos = callback_args
    echo_msg = (msg.data + "...") * num_echos
    print(echo_msg)
    publisher.publish(String(data=echo_msg))


def main():
    rospy.init_node('echoer')

    num_echos = rospy.get_param('~num_echos', default=3)

    pub = rospy.Publisher('/echo_out', String, queue_size=10)
    rospy.Subscriber('/echo_in', String, callback,
                     callback_args=(pub, num_echos))

    rospy.spin()


if __name__ == '__main__':
    main()
