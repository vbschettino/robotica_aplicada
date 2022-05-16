1. 
```bash
# run each command in an idle terminal
roscore
rostopic pub /greetings std_msgs/String "Hello world"
rostopic echo /greetings
# give proper names to the nodes (first kill, with Ctrl-C, the nodes that were already running)
rostopic pub /greetings std_msgs/String "Hello world" __name:=chatter
rostopic echo /greetings __name:=listener
# add constant message rate (kill chatter first)
rostopic pub /greetings std_msgs/String "Hello world" __name:=chatter -r 1
# add a second publisher
rostopic pub /greetings std_msgs/String "Hello ROS" __name:=chatter2 -r 1
# check node graph using CLI
rosnode list
rostopic list
rosnode info /chatter
rosnode info /listener
rostopic info /greetings
# check node graph using graphical interface
rqt # then go to Plugins>Introspection>Node Graph
```
