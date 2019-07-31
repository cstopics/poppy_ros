# Poppy ROS

## Prerequisites

Tested with *Ubuntu 16.04*

* **ROS Kinetic**

It was tested with ROS Kinetic: http://wiki.ros.org/. If you already have it installed, you should ensure it is updates as follows:

``` bash
sudo rm -f /etc/apt/sources.list.d/ros-latest.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

* **MoveIt!**: https://moveit.ros.org/. Instructions:

``` bash
sudo apt-get install ros-kinetic-catkin python-catkin-tools
sudo apt install ros-kinetic-moveit
```

* **Gazebo 9**: Installation steps:

``` bash
sudo apt-get remove ros-kinetic-gazebo*
sudo apt-get upgrade
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-kinetic-gazebo9*
```

* **ROS Controllers**:

``` bash
sudo apt-get install ros-kinetic-joint-state-controller  ros-kinetic-position-controllers ros-kinetic-effort-controllers
```

## Installation

Create a catkin workspace, clone the repository and build it:

``` bash
$ mkdir -p ~/ros/poppy_ws/src
$ cd ~/ros/poppy_ws/src/
$ git clone https://github.com/cstopics/poppy_ros
$ cd ..
$ catkin_make
```

## Testing

### Simulation

Launch nodes:

``` bash
roslaunch poppy_torso_gazebo poppy_torso_gazebo.launch
```

Test joints:

``` bash
rostopic pub -1 /poppy_torso/abs_z_position_controller/command std_msgs/Float64 -- 1.0
rostopic pub -1 /poppy_torso/bust_x_position_controller/command std_msgs/Float64 -- -0.2
rostopic pub -1 /poppy_torso/head_z_position_controller/command std_msgs/Float64 -- 0.3
rostopic pub -1 /poppy_torso/r_shoulder_x_position_controller/command std_msgs/Float64 -- 1.0
```
