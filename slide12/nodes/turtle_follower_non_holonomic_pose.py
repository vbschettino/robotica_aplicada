#!/usr/bin/env python3

"""Make a turtle from Turtlesim follow another, including orientation.
Consider non-holonomic turtle.
"""

import math

import rospy
from geometry_msgs.msg import Twist, Vector3
from turtlesim.msg import Pose


# This could (should) be in a separate, ROS-agnostic, module
class TurtleController:
    """Control turtle to reach target pose."""

    def __init__(self, kp_rho=1.5, kp_alpha=5, kp_beta=-4):
        self.current_x = 0
        self.current_y = 0
        self.current_th = 0
        self.target_x = 0
        self.target_y = 0
        self.target_th = 0
        assert kp_rho > 0, 'kp_rho must be > 0'
        assert kp_beta < 0, 'kp_beta must be < 0'
        assert kp_alpha > kp_rho, 'kp_rho must be > kp_alpha'
        self.kp_rho = kp_rho
        self.kp_alpha = kp_alpha
        self.kp_beta = kp_beta

    def get_control_vel(self):
        """Control law. From section 4.1.1.4 in Peter Corke's book v2."""
        error_x = self.target_x - self.current_x
        error_y = self.target_y - self.current_y
        # Convert to polar coordinates
        rho = math.sqrt(error_x**2 + error_y**2)
        alpha = math.atan2(error_y, error_x) - self.current_th
        beta = self.target_th - self.current_th - alpha
        # Convert alpha and beta to the [-pi pi] range
        alpha = ((alpha + math.pi) % (2*math.pi)) - math.pi
        beta = ((beta + math.pi) % (2*math.pi)) - math.pi
        # Control law
        lin_vel = self.kp_rho * rho
        ang_vel = self.kp_alpha * alpha + self.kp_beta * beta
        if (rho < 0.1) and (abs(self.target_th - self.current_th) < 0.1):
            return 0, 0  # to avoid instability at singularity
        else:
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
    kp_rho = rospy.get_param('~kp_rho', 1.5)
    kp_alpha = rospy.get_param('~kp_alpha', 5)
    kp_beta = rospy.get_param('~kp_beta', -4)
    controller_rate = rospy.get_param('~controller_rate', 10)

    # Start follower controller
    controller = TurtleController(
        kp_rho=kp_rho, kp_alpha=kp_alpha, kp_beta=kp_beta)

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
