#!/usr/bin/env python3

""" Move an object from one place to another using MoveIt
Based on the following MoveIt tutorial:
https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html
https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py
"""

# TODO: make pickable object a different color
# help here: https://github.com/Aharobot/inmoov_ros/blob/master/robbie_moveit/nodes/get_beer.py

# TODO: actually pick the object
# help here:
# https://ros-planning.github.io/moveit_tutorials/doc/pick_place/pick_place_tutorial.html
# http://docs.ros.org/en/noetic/api/moveit_commander/html/annotated.html
# http://docs.ros.org/en/noetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
# https://python.hotexamples.com/examples/moveit_commander/MoveGroupCommander/pick/python-movegroupcommander-pick-method-examples.html
# https://github.com/Aharobot/inmoov_ros/blob/master/robbie_moveit/nodes/get_beer.py
# https://github.com/dabarov/moveit-pick-place-python/blob/main/scripts/main.py


import sys
import copy
from math import pi

import rospy
import moveit_commander
import tf
from geometry_msgs.msg import PoseStamped, Quaternion


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

    # Instantiate a `RobotCommander`_ object. Provides information such as the
    #  robot's kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    # Instantiate a `PlanningSceneInterface`_ object. Provides a interface
    #  for getting and setting the robot's internal understanding of the world
    scene = moveit_commander.PlanningSceneInterface()

    # Instantiate a `MoveGroupCommander`_ object. Provides a interface to plan
    #  and execute motions
    move_group = moveit_commander.MoveGroupCommander('panda_arm')
    move_group.set_max_velocity_scaling_factor(0.5)
    move_group.set_max_acceleration_scaling_factor(0.5)
    eef_link = move_group.get_end_effector_link()
    touch_links = robot.get_link_names(group='panda_hand')

    # Create artificial scene for the robot
    rospy.sleep(1)  # wait for scene object to initialize
    scene_objects = create_scene(scene)
    rospy.loginfo('Scene created. Objects are: ' + str(scene_objects))
    move_group.set_support_surface_name('table')
    rospy.sleep(1)

    # Define initial and target poses of the object to be picked
    object_initial_pose = scene.get_object_poses(['can'])['can']
    object_initial_pose.position.x -= 0.2  # offset, just go near the object
    grasping_rpy = (0, -pi/2, pi)  # roll, pitch and yaw for grasping
    grasping_quaternion = tf.transformations.quaternion_from_euler(
        *grasping_rpy)
    object_initial_pose.orientation = Quaternion(*grasping_quaternion)

    object_target_pose = copy.deepcopy(object_initial_pose)
    object_target_pose.position.y *= -1  # other side of the table

    # Go near the object
    move_group.clear_pose_targets()
    move_group.set_pose_target(
        object_initial_pose, end_effector_link='panda_hand')
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    rospy.loginfo('Reached object')
    rospy.sleep(1)

    # Here we should pick (grasp) the object
    scene.attach_box(eef_link, 'can', touch_links=touch_links)
    scene_objects.remove('can')
    wait_for_scene_update(scene, scene_objects,
                          expected_attached_objects=['can'])

    # Move object to target location while avoiding obstacles
    move_group.set_pose_target(
        object_target_pose, end_effector_link='panda_hand')
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    rospy.loginfo('Object in goal pose')
    rospy.sleep(1)

    # Here we should place the object
    scene.remove_attached_object(eef_link, name='can')
    scene_objects.append('can')
    wait_for_scene_update(scene, scene_objects, expected_attached_objects=None)

    # Go back to home position
    move_group.set_named_target('ready')
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    rospy.loginfo('Back to home position.')
    rospy.sleep(1)

    # Clear the scene before closing
    scene.clear()


def create_scene(scene, reference_frame='world'):
    """Create simple artificial scene for pick-and-place operation"""
    scene_objects = []

    # Create suport surface (table)
    table_name = 'table'
    table_size = (0.4, 1, 0.05)  # x, y, z
    table_pose = PoseStamped()
    table_pose.header.frame_id = reference_frame
    table_pose.pose.orientation.w = 1.0
    table_pose.pose.position.x = 0.7
    table_pose.pose.position.z = 0.4
    scene.add_box(table_name, table_pose, size=table_size)
    scene_objects.append(table_name)

    # Create pickable object (can)
    can_name = 'can'
    can_height = 0.2
    can_radius = 0.02
    can_pose = PoseStamped()
    can_pose.header.frame_id = reference_frame
    can_pose.pose.orientation.w = 1.0
    can_pose.pose.position.x = table_pose.pose.position.x
    can_pose.pose.position.y = table_pose.pose.position.y + 0.3
    can_pose.pose.position.z = table_pose.pose.position.z + \
        table_size[2]/2 + can_height/2
    scene.add_cylinder(can_name, can_pose, can_height, can_radius)
    scene_objects.append(can_name)

    # Create obstacle
    obstacle_name = 'obstacle'
    obstacle_size = (0.4, 0.07, 0.3)
    obstacle_pose = PoseStamped()
    obstacle_pose.header.frame_id = reference_frame
    obstacle_pose.pose.orientation.w = 1.0
    obstacle_pose.pose.position.x = table_pose.pose.position.x
    obstacle_pose.pose.position.y = table_pose.pose.position.y
    obstacle_pose.pose.position.z = table_pose.pose.position.z + \
        table_size[2]/2 + obstacle_size[2]/2
    scene.add_box(obstacle_name, obstacle_pose, size=obstacle_size)
    scene_objects.append(obstacle_name)

    try:
        wait_for_scene_update(scene, expected_scene_objects=scene_objects)
        return scene_objects
    except TimeoutError as error:
        rospy.logerr(error.args[0] + ' Objects not created.')
        scene.clear()
        return None


def wait_for_scene_update(scene, expected_scene_objects=None,
                          expected_attached_objects=None, timeout=5):
    """Wait until moveit scene is updated"""
    if expected_scene_objects is None:
        expected_scene_objects = []
    if expected_attached_objects is None:
        expected_attached_objects = []

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
        # Are all objects expected to be on the scene there?
        scene_objects_ok = \
            set(expected_scene_objects) == set(scene.get_known_object_names())
        # Are all objects expected to be attached attached?
        attached_objects_ok = \
            set(expected_attached_objects) == set(scene.get_attached_objects())

        if scene_objects_ok and attached_objects_ok:
            return
        else:
            rospy.sleep(0.1)
            seconds = rospy.get_time()

    # If the while loop exited without returning then we timedout
    raise TimeoutError('Scene was not updated before timeout.')


if __name__ == "__main__":
    main()
