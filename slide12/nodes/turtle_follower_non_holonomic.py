#!/usr/bin/env python3

"""Make a turtle from Turtlesim follow another. Orientation is disregarded.
Consider non-holonomic turtle.
"""

import math

import rospy
from geometry_msgs.msg import Twist, Vector3
from turtlesim.msg import Pose


# This could (should) be in a separate, ROS-agnostic, module
class TurtleController:
    """Control turtle to reach target position."""

    def __init__(self, kp_x=1.5, kp_th=4):
        self.current_x = 0
        self.current_y = 0
        self.current_th = 0
        self.target_x = 0
        self.target_y = 0
        self.target_th = 0
        self.kp_x = kp_x
        self.kp_th = kp_th

    def get_control_vel(self):
        """Control law."""
        error_x = self.target_x - self.current_x
        error_y = self.target_y - self.current_y
        line_to_target_th = math.atan2(error_y, error_x)
        heading_error = line_to_target_th - self.current_th
        # Convert heading_error to the [-pi pi] range
        heading_error = ((heading_error + math.pi) % (2*math.pi)) - math.pi
        lin_vel = self.kp_x * math.sqrt(error_x**2 + error_y**2)
        ang_vel = self.kp_th * heading_error
        return lin_vel, ang_vel


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
    kp_x = rospy.get_param('~kp_x', 1.5)
    kp_th = rospy.get_param('~kp_th', 4)
    controller_rate = rospy.get_param('~controller_rate', 10)

    # Start follower controller
    controller = TurtleController(kp_x=kp_x, kp_th=kp_th)

    # Publishers and subscribers
    rospy.Subscriber('/turtle1/pose', Pose, update_pose_callback,
                     callback_args=(controller, 'target'))
    rospy.Subscriber('/turtle2/pose', Pose, update_pose_callback,
                     callback_args=(controller, 'controlled'))
    control_vel_pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

    # Spin controller at desired rate
    rate = rospy.Rate(controller_rate)
    while not rospy.is_shutdown():
        v_x, v_th = controller.get_control_vel()
        following_vel = Twist(linear=Vector3(v_x, 0, 0),
                              angular=Vector3(0, 0, v_th))
        control_vel_pub.publish(following_vel)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
