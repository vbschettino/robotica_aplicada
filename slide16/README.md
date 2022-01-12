# Exercise
1. Teleoperate Turtlebot to create a map of the Turtleworld environment with SLAM.
   - gmapping or hector_mapping
2. Restart the simulation with the robot in a new location. Move around with teleoperation until the robot localize itself in the map.
   - amcl
3. Use RViz to set goal poses. The robot should autonomously navigate there. Add new obstacles (not mapped) in Gazebo and see if the robot can navigate around then (if not, tweak you navigation parameters).
   - navigation stack / move_base

# Resources

## SLAM
- [gmapping](http://wiki.ros.org/gmapping)
   - http://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData
- [hector_mapping](http://wiki.ros.org/hector_mapping)
   - http://wiki.ros.org/hector_slam/Tutorials/SettingUpForYourRobot


## Localization
- [amcl](http://wiki.ros.org/amcl)

## Navigation
- http://wiki.ros.org/navigation
   - http://wiki.ros.org/navigation/Tutorials/RobotSetup
   - http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide

### move_base
- http://wiki.ros.org/move_base

- Costmaps configuration
   - http://wiki.ros.org/costmap_2d
   - http://wiki.ros.org/costmap_2d/Tutorials/Configuring%20Layered%20Costmaps
   - http://wiki.ros.org/costmap_2d/hydro/staticmap
   - http://wiki.ros.org/costmap_2d/hydro/obstacles
   - http://wiki.ros.org/costmap_2d/hydro/inflation

- Global planners
   - http://wiki.ros.org/navfn
   - http://wiki.ros.org/global_planner

- Local planner
   - http://wiki.ros.org/base_local_planner

## Example configuration from Turtlebot3
- http://wiki.ros.org/turtlebot3_navigation
- https://github.com/ROBOTIS-GIT/turtlebot3/tree/master/turtlebot3_navigation