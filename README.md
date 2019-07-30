# Poppy ROS

Tested with *Ubuntu 16.04* and *ROS Kinetic*

## Prerequisites

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

## Testing

Test joints:

``` bash
rostopic pub -1 /poppy_torso/abs_z_position_controller/command std_msgs/Float64 -- 1.0
rostopic pub -1 /poppy_torso/bust_x_position_controller/command std_msgs/Float64 -- -0.2
rostopic pub -1 /poppy_torso/head_z_position_controller/command std_msgs/Float64 -- 0.3
rostopic pub -1 /poppy_torso/r_shoulder_x_position_controller/command std_msgs/Float64 -- 1.0
```
