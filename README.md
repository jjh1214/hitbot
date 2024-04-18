# Hitbot
[![license - MIT](https://img.shields.io/:license-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![ROS2 Foxy](https://img.shields.io/badge/ROS2-Foxy-green.svg)](https://index.ros.org/doc/ros2/Releases/)

# Note
This repository is ROS2-Foxy Package for the [Z-Arm of Hitbot.](https://www.hitbotrobot.com/category/product-center/4-axis-robot-arm/)

It contain python-api and .so file

There is no simulation, just a real robot is connected and used.

# build
### *This Package is implemented at ROS2-Foxy.*
```
### I assume that you have installed the ros-foxy-desktop package using the apt-get command.
### I recommand the /home/<user_home>/hitbot_ws/src

$ mkdir -p ~/hitbot_ws/src
$ cd ~/hitbot_ws/src
$ git clone https://github.com/jjh1214/Hitbot.git
$ cd ~/hitbot_ws
$ colcon build
$ . install/setup.bash

```

# Run
Connect the hitbot to the PC with a TCP/IP cable.

Set the IP address of the PC as follows

- IP adress : 192.168.0.100

- Subnet mask : 255.255.255.0

- Default gateway : 192.168.0.1

Run the following code to verify the connection

```
$ ros2 run hitbot hitbot_controller
```

Robot connetion is sucess and if you want move robot following code other terminal
```
$ ros2 run hitbot hitbot_control_pub
```