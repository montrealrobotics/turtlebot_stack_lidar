# Gmapping with Hokuyo Lidar on Turtlebot

###### This package stacks up necessary packeges to run gmapping on Hokuyo Lidar with Turtlebot. We have more or less followed instructions from [this tutorial](http://wiki.ros.org/turtlebot/Tutorials/indigo/Adding%20a%20lidar%20to%20the%20turtlebot%20using%20hector_models%20%28Hokuyo%20UTM-30LX%29). Though we are using urgnode as opposed to hokuyo node and connecting our Lidar through ethernet port, and not /dev/ttyACM0. So we have skipped a few instructions from the original tutorial or modified it to run with urg_node. This has been successfully tested on ros-indigo. This tutorial assumes that your laser scan in being published on /scan topic, if that is not the case, kindly remap it to /scan topic to get it to work.

# Installation and setup instruction

### Installing dependencies.  

```sh
$ sudo apt install ros-kinetic-turtlebot-msgs
$ sudo apt install ros-kinetic-laser-proc
$ sudo apt install ros-kinetic-urg-node
$ cd ~
```

### Setting up the workspace.

```sh
$ mkdir turtlebot_lidar
$ cd ~/turtlebot_lidar
$ mkdir src
$ cd ~/turtlebot_lidar/src
$ catkin_init_workspace
$ cd ..
$ catkin_make -DCMAKE_BUILD_TYPE=Release -j8
```

### Getting the stack

```sh
$ cd ~/turtlebot_lidar/src
$ git clone https://github.com/montrealrobotics/turtlebot_stack_lidar.git
$ cd ~/turtlebot_lidar
$ catkin_make -DCMAKE_BUILD_TYPE=Release -j8
```
If you get any errors at this point, you should try installing packeges and their dependencies one at a time. 

### Setting up IP and port

- If you are connecting Lidar through ethernet, go to the following file. [~/turtlebot_lidar/src/turtlebot_stack_lidar/turtlebot_bringup/launch/minimal_with_hokuyo.launch](~/turtlebot_lidar/src/turtlebot_stack_lidar/turtlebot_bringup/launch/minimal_with_hokuyo.launch)  
- Change ip address in line 92 and put your Lidar ip address. 
- If you are connecting Lidar through USB, comment line 92 and uncomment line 93-94. 
 
# Running Gmapping

##### Running basic turtlebot functionality
Open the terminal and execute following commands to start basic turtlebot functionality
```sh
$ cd ~/turtlebot_lidar
$ source devel/setup.bash
$ export TURTLEBOT_3D_SENSOR=kinect
$ roslaunch turtlebot_bringup minimal_with_hokuyo.launch 3d_sensor:=kinect
```

##### Running gmapping with lidar
Open another tab and execute following commands to run gmapping,
```sh
$ cd ~/turtlebot_lidar
$ source devel/setup.bash
$ export TURTLEBOT_3D_SENSOR=kinect
$ roslaunch turtlebot_navigation gmapping_demo_hokuyo.launch 3d_sensor:=kinect
```

##### Moving around turtlebot
Open another tab and execute following commands to control turtlebot using keyboard,
```sh
$ cd ~/turtlebot_lidar
$ source devel/setup.bash
$ export TURTLEBOT_3D_SENSOR=kinect
$ roslaunch turtlebot_teleop keyboard_teleop.launch
```
- i to move ahead
- j to turn left
- l to turn right
- , to move back

##### Visualizing the map
Open another tab and execute following commands to see the map being constructed.
```sh
$ cd ~/turtlebot_lidar
$ source devel/setup.bash
$ export TURTLEBOT_3D_SENSOR=kinect
$ roslaunch turtlebot_rviz_launchers view_navigation.launch
```

Now you can move turtlebot and see map being constructed in rviz. Happy mapping!
