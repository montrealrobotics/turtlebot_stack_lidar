# Gmapping with Hokuyo Lidar on Turtlebot

> This stack helps run gmapping on a Turtlebot equipped with a Hokuyo laser. To an extent, we followed [this tutorial](http://wiki.ros.org/turtlebot/Tutorials/indigo/Adding%20a%20lidar%20to%20the%20turtlebot%20using%20hector_models%20%28Hokuyo%20UTM-30LX%29). 

> **Key differences** wrt the above tutorial are the following.
> 1. We use `urg_node` instead of `hokuyo_node`
> 2. Our laser scanner is `UTM20LX`, not `UTM30LX`.
> 3. The laser is connected via _Ethernet_, not _USB_.

> **IMPORTANT:** This package assumes that the laser scan is being published to the topic `/scan`. If this is not the case, ensure that the laser scan topic is remapped to `/scan`.


## Installation and setup instructions

### Install the dependencies

```sh
$ sudo apt install ros-kinetic-turtlebot-msgs
$ sudo apt install ros-kinetic-laser-proc
$ sudo apt install ros-kinetic-urg-node
$ cd ~
```

### Set up the workspace

```sh
$ mkdir turtlebot_lidar
$ cd ~/turtlebot_lidar
$ mkdir src
$ cd ~/turtlebot_lidar/src
$ catkin_init_workspace
$ cd ..
$ catkin_make -DCMAKE_BUILD_TYPE=Release -j8
```

### Get the turtlebot LiDAR stack

```sh
$ cd ~/turtlebot_lidar/src
$ git clone https://github.com/montrealrobotics/turtlebot_stack_lidar.git
$ cd ~/turtlebot_lidar
$ catkin_make -DCMAKE_BUILD_TYPE=Release -j8
```
If you get any errors at this point, you should try installing packeges and their dependencies one at a time. 

### Interface the LiDAR (Configure IP address, port number)

- If you are connecting Lidar through ethernet, go to the following file. <font style="color:red"> `~/turtlebot_lidar/src/turtlebot_stack_lidar/turtlebot_bringup/launch/minimal_with_hokuyo.launch` </font>   
- Change ip address in the following line if you are connected through _ethernet_, 
```xml
	<param name="ip_address" value="192.168.1.14"/>
```
- If you are connected via USB, uncomment following lines in launch file,
```xml
	<param name="serial_port" value="/dev/ttyACM0"/>
   	<param name="serial_baud" value="115200"/> 
```

 
## Run Gmapping

### Run basic turtlebot functionalities
```sh
$ cd ~/turtlebot_lidar
$ source devel/setup.bash
$ export TURTLEBOT_3D_SENSOR=kinect
$ roslaunch turtlebot_bringup minimal_with_hokuyo.launch 3d_sensor:=kinect
```

### Run gmapping with LiDAR
```sh
$ cd ~/turtlebot_lidar
$ source devel/setup.bash
$ export TURTLEBOT_3D_SENSOR=kinect
$ roslaunch turtlebot_navigation gmapping_demo_hokuyo.launch 3d_sensor:=kinect
```

### Move the turtlebot around
```sh
$ cd ~/turtlebot_lidar
$ source devel/setup.bash
$ export TURTLEBOT_3D_SENSOR=kinect
$ roslaunch turtlebot_teleop keyboard_teleop.launch
```
- `i` to move ahead
- `j` to turn left
- `l` to turn right
- `,` to move back

### Visualize the map
```sh
$ cd ~/turtlebot_lidar
$ source devel/setup.bash
$ export TURTLEBOT_3D_SENSOR=kinect
$ roslaunch turtlebot_rviz_launchers view_navigation.launch
```

Voila! Now you can move turtlebot around and visualize a map being constructed in `rviz`. Happy mapping!
