# Hitbot
[![license - MIT](https://img.shields.io/:license-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-red.svg)](https://index.ros.org/doc/ros2/Releases/)

# Note
This repository is ROS2-Foxy Package for the [Z-Arm of Hitbot.](https://www.hitbotrobot.com/category/product-center/4-axis-robot-arm/)

It contain python-api and .so file

This repository is able Rviz2 and real robot is connected and used.

# Build
### *This Package is implemented at ROS2-Humble.*
```
### I assume that you have installed the ros-humble-desktop package using the apt-get command.
### I recommand the /home/<user_home>/hitbot_ws/src
### Before activate simulation, please install moveit2 and gazebo



$ mkdir -p ~/hitbot_ws/src
$ cd ~/hitbot_ws/src
$ git clone -b humble https://github.com/jjh1214/hitbot.git
$ git clone -b humble https://github.com/jjh1214/hitbot_sim.git
$ git clone -b humble https://github.com/jjh1214/hitbot_msgs.git
$ git clone -b humble https://github.com/jjh1214/hitbot_moveit2_config.git

$ sudo apt-get install ros-humble-joint-state-publisher-gui

$ cd ~/hitbot_ws
$ colcon build
$ . install/setup.bash
$ source hitbot_ws/install/local_setup.bash



### if you want
# $ echo 'source ~/hitbot_ws/install/local_setup.bash' >> ~/.bashrc 
```

# Run - Only Real robot
Connect the hitbot to the PC with a TCP/IP cable.

Set the IP address of the PC as follows

- IP adress : 192.168.0.100

- Subnet mask : 255.255.255.0

- Default gateway : 192.168.0.1

### Run the following code to verify the connection

```
$ ros2 run hitbot hitbot_controller
```

### Robot connetion is sucess and if you want move robot following code other terminal
```
$ ros2 run hitbot hitbot_control_pub
```

# Run - Rviz2 simulation
```
$ ros2 launch hitbot_sim hitbot_rviz2.launch.py
```
![alt text](<Screenshot from 2024-05-03 09-31-35.png>)

Click Randomize and check changes the links in Rviz2.

Click Center to return to the initial position.

# Run - Moveit2 simulation
```
$ ros2 launch hitbot_moveit2_config demo.launch.py
```
![alt text](<Screenshot from 2024-05-13 13-23-45.png>)


# Run - Rviz2 simulation and Real robot
Connect the hitbot to the PC with a TCP/IP cable.

Set the IP address of the PC as follows

- IP adress : 192.168.0.100

- Subnet mask : 255.255.255.0

- Default gateway : 192.168.0.1

### Run the following code to verify the connection

```
$ ros2 run hitbot_sim hitbot_controller_joint_state
```

### Robot connetion is sucess and if you want move robot following code other terminal
```
$ ros2 launch hitbot_sim hitbot_rviz2.launch.py
```

Click Randomize and check changes the links in Real and Simulation.

Click Center to return to the initial position.