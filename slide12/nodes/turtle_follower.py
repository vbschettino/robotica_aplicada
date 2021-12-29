#!/usr/bin/env python3

"""Make a turtle from Turtlesim follow another. Orientation is disregarded."""

from math import sin, cos

import rospy
from geometry_msgs.msg import Twist, Vector3
from turtlesim.msg import Pose


# This could (should) be in a separate, ROS-agnostic, module
class TurtleController:
    """Control turtle to reach target position."""

    def __init__(self, kp_x=1, kp_y=1):
        self.current_x = 0
        self.current_y = 0
        self.current_th = 0
        self.target_x = 0
        self.target_y = 0
        self.target_th = 0
        self.kp_x = kp_x
        self.kp_y = kp_y

    def get_control_vel(self):
        """Simple proportional control law for omnidirectional movement."""
        vel_x = self.kp_x * (self.target_x - self.current_x)
        vel_y = self.kp_y * (self.target_y - self.current_y)
        # Convert velocity from world frame to base frame
        vel_x_base = vel_x*cos(self.current_th) + vel_y*sin(self.current_th)
        vel_y_base = -vel_x*sin(self.current_th) + vel_y*cos(self.current_th)
        return vel_x_base, vel_y_base


def update_pose_callback(msg, callback_args):
    """Subscribers' callback to update turtles' poses."""
    controller, which_pose = callback_args
    if which_pose == 'controlled':
        controller.current_x = msg.x
        controller.current_y = msg.y
        controller.current_th = msg.theta
    elif which_pose == 'target':
        controller.target_x = msg.x
        controller.target_y = msg.y
        controller.target_th = msg.theta
    else:
        raise Exception(f'"{which_pose}" is not a valid pose. '
                        f'Only "controlled" and "target" poses are valid.')


def main():
    """turtle_follower node."""
    # Start node and get parameters
    rospy.init_node('turtle_follower')
    kp_x = rospy.get_param('~kp_x', 1)
    kp_y = rospy.get_param('~kp_y', 1)
    controller_rate = rospy.get_param('~controller_rate', 10)

    # Start follower controller
    controller = TurtleController(kp_x=kp_x, kp_y=kp_y)

    # Publishers and subscribers
    rospy.Subscriber('/turtle1/pose', Pose, update_pose_callback,
                     callback_args=(controller, 'target'))
    rospy.Subscriber('/turtle2/pose', Pose, update_pose_callback,
                     callback_args=(controller, 'controlled'))
    control_vel_pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

    # Spin controller at desired rate
    rate = rospy.Rate(controller_rate)
    while not rospy.is_shutdown():
        v_x, v_y = controller.get_control_vel()
        following_vel = Twist(linear=Vector3(v_x, v_y, 0),
                              angular=Vector3(0, 0, 0))
        control_vel_pub.publish(following_vel)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
