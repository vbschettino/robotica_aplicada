#!/usr/bin/env python3

"""Wait for turtlesim simulator to start and then spawn a second turtle on it.
Deprecated in favor of simply callling the standard rosservice node in the
launch file. Left here as an example of how to use a simple service client.
"""

import rospy
from turtlesim.srv import Spawn


if __name__ == '__main__':
    # Start node and get parameters
    rospy.init_node('turtle_spawner')
    name = rospy.get_param('~turtle_name', default='turtle2')
    pose_x = rospy.get_param('~pose_x', default=0)
    pose_y = rospy.get_param('~pose_y', default=0)
    pose_th = rospy.get_param('~pose_th', default=0)

    # Wait for service to become available then call it
    rospy.wait_for_service('/spawn')
    try:
        srv_client_handler = rospy.ServiceProxy('/spawn', Spawn)
        srv_client_handler.call(name=name, x=pose_x, y=pose_y, theta=pose_th)
    except rospy.ServiceException as e:
        print(e)
