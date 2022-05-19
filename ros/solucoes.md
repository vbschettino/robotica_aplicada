# Soluções para [exercícios ROS](exercicios.md)

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

2. Aplicando comunicação por tópicos e serviços

    1. Ler a documentação do [turtlesim](http://wiki.ros.org/turtlesim).

    2. 
        ```bash
        roscore
        rosrun turtlesim turtlesim_node
        rqt
        ```
    3. 
        ```bash
        rostopic pub /turtle1/cmd_vel geometry_msgs/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}" -r 1
        # Ctrl-C to stop
        ```
    4. 
        ```bash
        rosrun turtlesim turtle_teleop_key
        # with the terminal window selected, use arrow keys to move the turtle
        # then check rqt again
        ```
    5. 
        ```bash
        rosservice list
        rosservice info /clear
        rosservice call /clear
        ```
    6. 
        ```bash
        rosservice info /spawn
        rossrv show turtlesim/Spawn
        rosservice call /spawn 2 2 0 turtle_bota
        ```
    7. 
        ```bash
        rosservice call /turtle_bota/teleport_relative 2.0 0.0
        ```
    8. Ctrl-C em todos os terminais ativos.


3. Trabalhando com arquivos launch e parâmetros

    1. Primeiro inspecione o arquivo [turtle_start1.launch](turtle_start/turtle_start1.launch). No terminal, navegue até onde o arquivo está salvo e execute:

        ```bash
        roslaunch turtle_start1.launch
        # a single Ctrl-C kills all nodes and the master
        ```
    2. 
        ```bash
        roslaunch turtle_start2.launch
        ```
    3. 
        ```bash
        roslaunch turtle_start3.launch
        # Ctrl-C
        roslaunch turtle_start3.launch brightness:=200
        ```
    4. 
        ```bash
        roslaunch turtle_start4.launch
        rosservice call /spawn 2 2 0 turtle_bota
        # or just use roslaunch turtle_start4b.launch
        ```
