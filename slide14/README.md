# Exercise

Launch Turtlebot simulator and use Dynamic Window Approach (DWA) to enable obstacle avoidance when teleoperating the robot.

# Resources

## DWA original paper
https://ieeexplore.ieee.org/abstract/document/580977

## ROS packages that implement DWA
- navigation stack's [base_local_planner](http://wiki.ros.org/base_local_planner) - most widely used navigation package, but more appropriate for autonomous navigation, not easy to use for safe teleoperation.
    - [tutorial](http://wiki.ros.org/navigation/Tutorials/RobotSetup)
- [nav2d_operator](http://wiki.ros.org/nav2d_operator) - also widely used, but expects a custom `cmd` message type instead of regular `geometry_msgs/Twist` (`cmd_vel`) messages.
    - [tutorial](http://wiki.ros.org/nav2d/Tutorials/RobotOperator)
- amslabtech's [dwa_planner](https://github.com/amslabtech/dwa_planner) - seems to work well, but takes as input a target local goal, instead of a command velocity.
- [safe_teleop_base](http://wiki.ros.org/safe_teleop_base) - the one we'll go with.
    - Expects a 2D [costmap](http://wiki.ros.org/costmap_2d) to be available.

## Other obstacle avoidance packages
- neonavigation's [safety_limiter](https://github.com/at-wat/neonavigation/tree/master/safety_limiter)

## Turtlebot's default parameters for navigation
- [turtlebot3_navigation/param](https://github.com/ROBOTIS-GIT/turtlebot3/tree/master/turtlebot3_navigation/param)
    - costmap_common_params_burger.yaml
    - local_costmap_params.yaml
    - base_local_planner_params.yaml

## Other resources used
- [mouse teleoperation](https://index.ros.org/p/mouse_teleop/), from [teleop_tools](https://index.ros.org/r/teleop_tools/)

