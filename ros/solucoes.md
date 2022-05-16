1. Entendendo e usando comunicação por tópicos

    1. 
        ```bash
        # run each command in an idle terminal
        roscore
        rostopic pub /greetings std_msgs/String "Hello world"
        rostopic echo /greetings
        ```
    2. 
        ```bash
        # first kill running nodes (not the master) with Ctrl-C and then
        rostopic pub /greetings std_msgs/String "Hello world" __name:=chatter
        rostopic echo /greetings __name:=listener
        ```
    3. 
        ```bash
        # kill chatter first
        rostopic pub /greetings std_msgs/String "Hello world" __name:=chatter -r 1
        ```
    4. 
        ```bash
        rostopic pub /greetings std_msgs/String "Hello ROS" __name:=chatter2 -r 1
        ```
    5. 
        ```bash
        # check node graph using CLI
        rosnode list
        rostopic list
        rosnode info /chatter
        rosnode info /listener
        rostopic info /greetings
        ```
    6. 
        ```bash
        rqt # then go to Plugins>Introspection>Node Graph
        ```
